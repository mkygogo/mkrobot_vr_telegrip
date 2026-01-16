from dataclasses import dataclass, field
from functools import cached_property
import numpy as np
import serial
import time
import logging
from typing import Dict, Any
import cv2
import threading

try:
    from .drivers.DM_Control_Python.DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type, DM_variable
    DRIVERS_AVAILABLE = True
except ImportError:
    print("Warning: DM_CAN drivers not found. Hardware will not work.")
    DRIVERS_AVAILABLE = False
    class Motor: pass
    class MotorControl: pass
    class DM_Motor_Type: DM4340=0; DM4310=1

logger = logging.getLogger("MKDriver")

JOINT_LIMITS = {
    0: (-3.0, 3.0), 1: (0.0, 3.0), 2: (0.0, 3.0),
    3: (-1.7, 1.2), 4: (-0.4, 0.4), 5: (-2.0, 2.0),
}

def map_range(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class MKRobotStandalone:
    def __init__(self, port: str = "/dev/ttyACM0", joint_velocity_scaling: float = 0.2,camera_indices: Dict[str, int] = None):
        self.port = port
        self.serial_lock = threading.Lock()
        if camera_indices is None:
            # 这里填你之前测试成功的参数
            self.camera_indices = {
                'top':   {'index': 0, 'width': 640, 'height': 480},
                'wrist': {'index': 2, 'width': 640, 'height': 360}
            }
        else:
            self.camera_indices = camera_indices
        self.cameras = {}
        self.serial_conn = None
        self.control = None
        self.is_connected = False
        self.joint_velocity_scaling: float = 0.2
        self.max_gripper_torque: float = 1.0 # Nm (/0.00875m spur gear radius = 114N gripper force)
        
        # 参数
        self.max_step_rad = 0.8
        self.gripper_open_pos = 0.0
        self.gripper_closed_pos = -4.7

        # Constants for EMIT control
        self.DM4310_TORQUE_CONSTANT = 0.945  # Nm/A
        self.EMIT_VELOCITY_SCALE = 100  # rad/s
        self.EMIT_CURRENT_SCALE = 1000  # A
        
        self.DM4310_SPEED = 200/60*2*np.pi   # rad/s (200  rpm | 20.94 rad/s)
        self.DM4340_SPEED = 52.5/60*2*np.pi  # rad/s (52.5 rpm | 5.49  rad/s)
        # 对应: [J1, J2, J3, J4, J5, J6, Gripper]
        # 根据你的测试：除了 J2(索引1) 和 夹爪(索引6)，其他全反
        # ------------------------------------------------------------
        self.HARDWARE_DIR = np.array([-1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0], dtype=np.float32)

        # 初始化电机
        if DRIVERS_AVAILABLE:
            self.motors = {
                "joint_1": Motor(DM_Motor_Type.DM4340, 0x01, 0x11),
                "joint_2": Motor(DM_Motor_Type.DM4340, 0x02, 0x12),
                "joint_3": Motor(DM_Motor_Type.DM4340, 0x03, 0x13),
                "joint_4": Motor(DM_Motor_Type.DM4310, 0x04, 0x14),
                "joint_5": Motor(DM_Motor_Type.DM4310, 0x05, 0x15),
                "joint_6": Motor(DM_Motor_Type.DM4310, 0x06, 0x16),
                "gripper": Motor(DM_Motor_Type.DM4310, 0x07, 0x17),
            }
        else:
            self.motors = {}    

    def configure(self) -> None:
        for key, motor in self.motors.items():
            self.control.addMotor(motor)

            for _ in range(3):
                self.control.refresh_motor_status(motor)
                time.sleep(0.01)

            if self.control.read_motor_param(motor, DM_variable.CTRL_MODE) is not None:
                print(f"{key} ({motor.MotorType.name}) is connected.")

                self.control.switchControlMode(motor, Control_Type.POS_VEL)
                self.control.enable(motor)
            else:
                raise Exception(
                    f"Unable to read from {key} ({motor.MotorType.name}).")

        for joint in ["joint_1", "joint_2", "joint_3"]:
            self.control.change_motor_param(self.motors[joint], DM_variable.ACC, 10.0)
            self.control.change_motor_param(self.motors[joint], DM_variable.DEC, -10.0)
            self.control.change_motor_param(self.motors[joint], DM_variable.KP_APR, 200)
            self.control.change_motor_param(self.motors[joint], DM_variable.KI_APR, 10)

        for joint in ["gripper"]:
            self.control.change_motor_param(
                self.motors[joint], DM_variable.KP_APR, 100)

        #set 1~6 motor to zero 
        logger.info("开始设置电机零位...")
        for joint in ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]:
            try:
                logger.info(f"设置 {joint} 零位...")
                self.control.set_zero_position(self.motors[joint])
                time.sleep(0.1)
                
                # 验证零位设置
                self.control.refresh_motor_status(motor)
                new_pos = motor.getPosition()
                logger.info(f"  {joint}: 设零后位置 = {new_pos:.3f} rad")
                
            except Exception as e:
                logger.error(f"设置 {joint} 零位失败: {e}")

        #Open gripper and set zero position
        self.control.switchControlMode(
            self.motors["gripper"], Control_Type.VEL)
        self.control.control_Vel(self.motors["gripper"], 10.0)
        while True:
            self.control.refresh_motor_status(self.motors["gripper"])
            tau = self.motors["gripper"].getTorque()
            if tau > 0.6: #0.8
                self.control.control_Vel(self.motors["gripper"], 0.0)
                self.control.disable(self.motors["gripper"])
                self.control.set_zero_position(self.motors["gripper"])
                time.sleep(0.2)
                self.control.enable(self.motors["gripper"])
                break
            time.sleep(0.01)
        self.control.switchControlMode(
            self.motors["gripper"], Control_Type.Torque_Pos)

    # @property
    # def is_connected(self) -> bool:
    #     return self.bus_connected 

    def connect(self) -> None:
        if self.is_connected:
            print(f"{self} already connected")
            return

        self.serial_device = serial.Serial(
            self.port, 921600, timeout=0.5)
        time.sleep(0.3)

        self.control = MotorControl(self.serial_device)
        self.is_connected = True
        self.configure()

        for name, config in self.camera_indices.items():
            try:
                # 1. 解析参数：兼容旧的 int 格式和新的 dict 格式
                if isinstance(config, int):
                    idx = config
                    w, h = 640, 480 # 旧代码的默认值
                elif isinstance(config, dict):
                    idx = config.get('index', 0)
                    w = config.get('width', 640)
                    h = config.get('height', 480)
                else:
                    continue

                # 2. 设置摄像头
                cap = cv2.VideoCapture(idx)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                
                # 3. 存储对象
                if cap.isOpened():
                    self.cameras[name] = cap
                    print(f"📷 Camera '{name}' connected: Index={idx}, Res={w}x{h}")
                else:
                    print(f"⚠️ Warning: Camera '{name}' (Index {idx}) failed to open.")
                    
            except Exception as e:
                print(f"❌ Error initializing camera {name}: {e}")

    def get_observation(self) -> Dict[str, Any]:
        """
        对外接口: 适配 env.py
        返回: {'state': np.ndarray, 'images': dict}
        """
        #print("✅✅✅ YES! NEW CODE IS RUNNING! ✅✅✅")
        with self.serial_lock:
            raw_obs = self._get_observation()

        if raw_obs is None:
            return {"state": np.zeros(7, dtype=np.float32), "images": {}}

        # 1. 解析 State 字典转 Array
        # 你的 _get_observation 返回的是 {'joint_1.pos': val, ...}
        # 我们需要按 j1...j6, gripper 的顺序拼成 (7,) 数组
        # q_list = []
        # for i in range(1, 7):
        #     key = f"joint_{i}.pos"
        #     q_list.append(raw_obs.get(key, 0.0))
        # # 夹爪
        # q_list.append(raw_obs.get("gripper.pos", 0.0))
        q_list = [
            raw_obs.get("joint_1.pos", 0.0),
            raw_obs.get("joint_2.pos", 0.0),
            raw_obs.get("joint_3.pos", 0.0),
            raw_obs.get("joint_4.pos", 0.0),
            raw_obs.get("joint_5.pos", 0.0),
            raw_obs.get("joint_6.pos", 0.0),
            raw_obs.get("gripper.pos", 0.0)
        ]
        
        physical_state = np.array(q_list, dtype=np.float32)
        sim_state = physical_state * self.HARDWARE_DIR
        #state_array = np.array(q_list, dtype=np.float32)

        # 2. 图像直接透传 (假设 _get_observation 里的 images 逻辑已经处理了)
        # 注意：看你原来的代码，_get_observation 里 images 变量虽然生成了但没塞进 return 的字典里
        # 这里我做个补救，重新读取一次图像，或者你可以修改 _get_observation 让它返回 images
        
        # 为了不改动你的 _get_observation，我在这里单独读一次相机
        images = {}
        for name, cap in self.cameras.items():
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    images[name] = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                else:
                    images[name] = np.zeros((480, 640, 3), dtype=np.uint8)
            else:
                images[name] = np.zeros((480, 640, 3), dtype=np.uint8)

        return {"state": sim_state, "images": images}

    def _get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            print(f"{self} is not connected.")
            return

        # Read arm position
        start = time.perf_counter()

        obs_dict = {}
        for key, motor in self.motors.items():
            self.control.refresh_motor_status(motor)
            if 0: 
                pass
            if key == "gripper":
                # Normalize gripper position between 1 (closed) and 0 (open)
                obs_dict[f"{key}.pos"] = map_range(
                    motor.getPosition(), self.gripper_open_pos, self.gripper_closed_pos, 0.0, 1.0)
            else:
                obs_dict[f"{key}.pos"] = motor.getPosition()

        return obs_dict

    def check_joints_limit(self, action_array: np.ndarray) -> np.ndarray:
        if action_array is None:
            return np.zeros(7, dtype=np.float32)
            
        safe_action = action_array.copy()
        
        for i in range(6):
            if i in JOINT_LIMITS:
                min_lim, max_lim = JOINT_LIMITS[i]
                safe_action[i] = np.clip(safe_action[i], min_lim, max_lim)
        
        # 夹爪限位
        safe_action[6] = np.clip(safe_action[6], 0.0, 1.0)
        return safe_action  

    def send_action(self, action: np.ndarray):
        """
        对外接口: 适配 env.py
        输入: np.ndarray (7,) [j1...j6, gripper]
        """
        try:
            shape_info = action.shape if hasattr(action, 'shape') else 'no_shape'
            #print(f"🐛 [Driver] send_action input: shape={shape_info}, size={action.size}")
        except Exception as e:
            print(f"🐛 [Driver] Logging error: {e}")

        if not self.is_connected: return

        # 0. 确保是 numpy 数组
        if not isinstance(action, np.ndarray):
            action = np.array(action, dtype=np.float32)

        target_physical = action * self.HARDWARE_DIR

        # 1. 安全防护：限位检查
        # 你的 _send_action 里是直接执行，所以在传给它之前必须截断
        safe_action_arr = self.check_joints_limit(target_physical)

        # 2. 构造 _send_action 需要的字典格式
        # 格式: {'joint_1.pos': val, ..., 'gripper.pos': val}
        action_dict = {}
        for i in range(6):
            action_dict[f"joint_{i+1}.pos"] = safe_action_arr[i]
        
        action_dict["gripper.pos"] = safe_action_arr[6]

        with self.serial_lock:
            self._send_action(action_dict)

    def _send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            print(f"{self} is not connected.")
            return

        goal_pos = {key.removesuffix(
            ".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Send goal position to the arm
        for key, motor in self.motors.items():
            if 0:
                pass
            if key == "gripper":
                self.control.refresh_motor_status(motor)
                gripper_goal_pos_mapped = map_range(goal_pos[key], 0.0, 1.0, self.gripper_open_pos, self.gripper_closed_pos)
                self.control.control_pos_force(motor, gripper_goal_pos_mapped, self.DM4310_SPEED*self.EMIT_VELOCITY_SCALE,
                                               i_des=self.max_gripper_torque/self.DM4310_TORQUE_CONSTANT*self.EMIT_CURRENT_SCALE)
            else:
                self.control.control_Pos_Vel(
                    motor, goal_pos[key], self.joint_velocity_scaling*self.DM4340_SPEED)

        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def disconnect(self):
        if not self.is_connected:
           print(f"{self} is not connected.")
           return

        # 默认策略：断开连接前，先让电机失能(卸力)，防止意外
        try:
            for motor in self.motors.values():
                self.control.disable(motor)
        except Exception as e:
            logger.error(f"Error disabling motors during disconnect: {e}")

        # 关闭串口
        try:
            if hasattr(self.control, 'serial_'):
                self.control.serial_.close()
        except Exception as e:
            logger.error(f"Error closing serial port: {e}")
        self.is_connected = False

        for cap in self.cameras.values(): 
            cap.release()

        logger.info(f"{self} disconnected.")

    def close(self):
        self.disconnect()


