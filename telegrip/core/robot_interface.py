"""
Robot interface module for the SO100 teleoperation system.
Provides a clean wrapper around robot devices with safety checks and convenience methods.
"""

import numpy as np
import torch
import time
import logging
import os
import sys
import contextlib
from typing import Optional, Dict, Tuple

from .mk_driver import MKRobotStandalone
from .mk_arm_ik_core import MKArmIKCore

from ..config import (
    TelegripConfig, NUM_JOINTS, JOINT_NAMES,
    GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE, 
    WRIST_FLEX_INDEX, WRIST_YAW_INDEX, WRIST_ROLL_INDEX, GRIPPER_INDEX, # 增加导入
    URDF_TO_INTERNAL_NAME_MAP
)
#from .kinematics import ForwardKinematics, IKSolver

logger = logging.getLogger(__name__)


@contextlib.contextmanager
def suppress_stdout_stderr():
    """Context manager to suppress stdout and stderr output at the file descriptor level."""
    # Save original file descriptors
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()
    
    # Save original file descriptors
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)
    
    try:
        # Open devnull
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        
        # Redirect stdout and stderr to devnull
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)
        
        yield
        
    finally:
        # Restore original file descriptors
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)
        
        # Close saved file descriptors
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


class RobotInterface:
    """High-level interface for SO100 robot control with safety features."""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        self.left_robot = None
        self.right_robot = None
        self.is_connected = False
        self.is_engaged = False  # New state for motor engagement
        
        # Individual arm connection status
        self.left_arm_connected = False
        self.right_arm_connected = False
        
        # Joint state
        self.left_arm_angles = np.zeros(NUM_JOINTS)
        self.right_arm_angles = np.zeros(NUM_JOINTS)
        
        # Joint limits (will be set by visualizer)
        self.joint_limits_min_deg = np.full(NUM_JOINTS, -180.0)
        self.joint_limits_max_deg = np.full(NUM_JOINTS, 180.0)
        
        # 1. 创建共享的 Meshcat Viewer
        import meshcat
        self.shared_viewer = meshcat.Visualizer()
        print(f"\n🌍 Combined Visualizer URL: {self.shared_viewer.url()}\n")
        # 2. 为左右臂分配不同的命名空间
        self.ik_cores = {
            'left': MKArmIKCore(
                urdf_path=self.config.urdf_path, 
                mesh_dir=self.config.mesh_dir,
                visualize=True,
                viewer=self.shared_viewer,
                namespace="left_arm" # 左臂命名空间
            ),
            'right': MKArmIKCore(
                urdf_path=self.config.urdf_path, 
                mesh_dir=self.config.mesh_dir,
                visualize=True,
                viewer=self.shared_viewer,
                namespace="right_arm" # 右臂命名空间
            )
        }
        
        # 3. 设置物理偏移（防止两个模型重叠在原点）
        import pinocchio as pin
        self.shared_viewer["left_arm"].set_transform(pin.SE3(np.eye(3), np.array([0.5, 0, 0])).homogeneous)
        self.shared_viewer["right_arm"].set_transform(pin.SE3(np.eye(3), np.array([-0.5,  0, 0])).homogeneous)

        # Control timing
        self.last_send_time = 0
        
        # Error tracking - separate for each arm
        self.left_arm_errors = 0
        self.right_arm_errors = 0
        self.general_errors = 0
        self.max_arm_errors = 3  # Allow fewer errors per arm before marking as disconnected
        self.max_general_errors = 8  # Allow more general errors before full disconnection
        
        # Initial positions for safe shutdown - restored original values
        self.initial_left_arm = np.array([0, -100, 100, 60, 0, 0])
        self.initial_right_arm = np.array([0, -100, 100, 60, 0, 0])
        #必须使用正确的物理限位 (对应 config.py 或 mk_driver.py)
        # J1-J3 使用解算器限位, J4-J6 使用电机物理限位
        self.joint_limits_min_deg = np.array([-120, -10, -150, -97, -22, -114, 0])
        self.joint_limits_max_deg = np.array([120, 150, 0, 68, 22, 114, 45])
    
    # def setup_robot_configs(self) -> Tuple[SO100FollowerConfig, SO100FollowerConfig]:
    #     """Create robot configurations for both arms."""
    #     logger.info(f"Setting up robot configs with ports: {self.config.follower_ports}")
        
    #     left_config = SO100FollowerConfig(
    #         port=self.config.follower_ports["left"],
    #         use_degrees=True,  # Use degrees for easier debugging
    #         disable_torque_on_disconnect=True
    #     )
    #     # Set the robot name for calibration file lookup
    #     left_config.id = "left_follower"
        
    #     right_config = SO100FollowerConfig(
    #         port=self.config.follower_ports["right"],
    #         use_degrees=True,  # Use degrees for easier debugging
    #         disable_torque_on_disconnect=True
    #     )
    #     # Set the robot name for calibration file lookup
    #     right_config.id = "right_follower"
        
    #     return left_config, right_config
    def setup_robot_configs(self) -> Tuple[str, str]:
        """现在只返回端口地址"""
        logger.info(f"Setting up robot ports: {self.config.follower_ports}")
        return self.config.follower_ports["left"], self.config.follower_ports["right"]
    
    def connect(self) -> bool:
        """Connect to robot hardware."""
        if self.is_connected:
            logger.info("Robot interface already connected")
            return True
        
        if not self.config.enable_robot:
            logger.info("Robot interface disabled in config")
            self.is_connected = True  # Mark as "connected" for testing
            return True
        
        # Setup suppression if requested
        should_suppress = (self.config.log_level == "warning" or 
                          self.config.log_level == "critical" or 
                          self.config.log_level == "error")
        try:
            left_port, right_port = self.setup_robot_configs()
            
            # 连接左臂
            try:
                # 替换为自定义驱动
                self.left_robot = MKRobotStandalone(port=left_port) 
                self.left_robot.connect()
                self.left_arm_connected = True
                logger.info("✅ Left arm (MKRobot) connected")
            except Exception as e:
                logger.error(f"❌ Left arm failed: {e}")
                self.left_arm_connected = False
                
            # 连接右臂 (同理)
            try:
                self.right_robot = MKRobotStandalone(port=right_port) 
                self.right_robot.connect()
                self.right_arm_connected = True
                logger.info("✅ Right arm (MKRobot) connected")
            except Exception as e:
                logger.error(f"❌ Right arm failed: {e}")
                self.right_arm_connected = False
                
            # Mark as connected if at least one arm is connected
            self.is_connected = self.left_arm_connected or self.right_arm_connected
            
            if self.is_connected:
                # Initialize joint states
                self._read_initial_state()
                logger.info(f"🤖 Robot interface connected: Left={self.left_arm_connected}, Right={self.right_arm_connected}")
            else:
                logger.error("❌ Failed to connect any robot arms")
                
            return self.is_connected
            
        except Exception as e:
            logger.error(f"❌ Robot connection failed with exception: {e}")
            self.is_connected = False
            return False
        
    def _read_initial_state(self):
        try:
            for arm, robot, connected in [("left", self.left_robot, self.left_arm_connected), 
                                        ("right", self.right_robot, self.right_arm_connected)]:
                if robot and connected:
                    obs = robot.get_observation() # 👈 获取最新驱动的字典
                    if obs and "state" in obs:
                        sim_state = obs["state"] # 👈 获取 7 维 [j1..j6, gripper_norm]
                        
                        # 转换回角度用于 RobotInterface 内部存储
                        angles_deg = np.zeros(NUM_JOINTS)
                        angles_deg[:6] = np.rad2deg(sim_state[:6])
                        angles_deg[6] = sim_state[6] * 45.0 # 映射 0-1 到 0-45度
                        
                        if arm == "left": self.left_arm_angles = angles_deg
                        else: self.right_arm_angles = angles_deg

                        # 同步给 IK 核心
                        self.ik_cores[arm].set_state_from_hardware(sim_state)
        except Exception as e:
            logger.error(f"Error reading initial state: {e}")
    # def _read_initial_state(self):
    #     """Read initial joint state from robot."""
    #     try:
    #         if self.left_robot and self.left_arm_connected:
    #             observation = self.left_robot.get_observation()
    #             if observation:
    #                 # Extract joint positions from observation
    #                 self.left_arm_angles = np.array([
    #                     observation['shoulder_pan.pos'],
    #                     observation['shoulder_lift.pos'],
    #                     observation['elbow_flex.pos'],
    #                     observation['wrist_flex.pos'],
    #                     observation['wrist_yaw.pos'],
    #                     observation['wrist_roll.pos'],
    #                     observation['gripper.pos']
    #                 ])
    #                 logger.info(f"Left arm initial state: {self.left_arm_angles.round(1)}")
                    
    #         if self.right_robot and self.right_arm_connected:
    #             observation = self.right_robot.get_observation()
    #             if observation:
    #                 # Extract joint positions from observation
    #                 self.right_arm_angles = np.array([
    #                     observation['shoulder_pan.pos'],
    #                     observation['shoulder_lift.pos'],
    #                     observation['elbow_flex.pos'],
    #                     observation['wrist_flex.pos'],
    #                     observation['wrist_yaw.pos'],
    #                     observation['wrist_roll.pos'],
    #                     observation['gripper.pos']
    #                 ])
    #                 logger.info(f"Right arm initial state: {self.right_arm_angles.round(1)}")
            
    #         # 读取到 hardware_angles 后，同步给 IK 核心
    #         if self.left_arm_connected:
    #             # [修正] 构造 7 维向量（包含夹爪归一化值），因为 mk_arm_ik_core 期待 [j1..j6, gripper]
    #             q_sim_7 = np.append(self.left_arm_angles[:6], self.left_arm_angles[6]/45.0)
    #             self.ik_cores['left'].set_state_from_hardware(q_sim_7)
                
    #         if self.right_arm_connected:
    #             q_sim_7 = np.append(self.right_arm_angles[:6], self.right_arm_angles[6]/45.0)
    #             self.ik_cores['right'].set_state_from_hardware(q_sim_7)

    #     except Exception as e:
    #         logger.error(f"Error reading initial state: {e}")
    
    def setup_kinematics(self, physics_client, robot_ids: Dict, joint_indices: Dict, 
                        end_effector_link_indices: Dict, joint_limits_min_deg: np.ndarray, 
                        joint_limits_max_deg: np.ndarray):
        """Setup kinematics solvers using PyBullet components for both arms."""
        self.joint_limits_min_deg = joint_limits_min_deg.copy()
        self.joint_limits_max_deg = joint_limits_max_deg.copy()
        
        # Setup solvers for both arms
        # for arm in ['left', 'right']:
        #     self.fk_solvers[arm] = ForwardKinematics(
        #         physics_client, robot_ids[arm], joint_indices[arm], end_effector_link_indices[arm]
        #     )
            
        #     self.ik_solvers[arm] = IKSolver(
        #         physics_client, robot_ids[arm], joint_indices[arm], end_effector_link_indices[arm],
        #         joint_limits_min_deg, joint_limits_max_deg, arm_name=arm
        #     )
        
        logger.info("Kinematics solvers initialized for both arms")
    
    # def get_current_end_effector_position(self, arm: str) -> np.ndarray:
    #     """Get current end effector position for specified arm."""
    #     if arm == "left":
    #         angles = self.left_arm_angles
    #     elif arm == "right":
    #         angles = self.right_arm_angles
    #     else:
    #         raise ValueError(f"Invalid arm: {arm}")
        
    #     if self.fk_solvers[arm]:
    #         position, _ = self.fk_solvers[arm].compute(angles)
    #         return position
    #     else:
    #         default_position = np.array([0.2, 0.0, 0.15])
    #         return default_position
    def get_current_end_effector_position(self, arm: str) -> np.ndarray:
        core = self.ik_cores[arm]
        # ✅ 修复：直接使用 core.arm 内部已经维护好的 8 维向量 self.q
        # 这个向量在初始化和 update_arm_angles 时已经处理好了维度
        import pinocchio as pin
        pin.framesForwardKinematics(core.arm.model, core.arm.data, core.arm.q)
        return core.arm.data.oMf[core.arm.ik_frame_id].translation.copy()
    
    # def solve_ik(self, arm: str, target_position: np.ndarray, 
    #              target_orientation: Optional[np.ndarray] = None) -> np.ndarray:
    #     """Solve inverse kinematics for specified arm."""
    #     if arm == "left":
    #         current_angles = self.left_arm_angles
    #     elif arm == "right":
    #         current_angles = self.right_arm_angles
    #     else:
    #         raise ValueError(f"Invalid arm: {arm}")
        
    #     if self.ik_solvers[arm]:
    #         return self.ik_solvers[arm].solve(target_position, target_orientation, current_angles)
    #     else:
    #         return current_angles[:3]  # Return current angles if no IK solver
    def solve_ik(self, arm: str, target_position: np.ndarray, 
             target_orientation: Optional[np.ndarray] = None) -> np.ndarray:
        """使用 Pinocchio 求解 IK"""
        #logger.info(f"🔍 IK Request - Arm: {arm} | Target XYZ: {target_position.round(3)}")
        core = self.ik_cores[arm]
        
        # ✅ 修复：传给解算器的必须是 core.arm.q (8维)
        # 这样进入 ThreeDofIKSolver 内部时，维度就是正确的
        q_new, debug_msg, cond, success, err = core.arm.ik_solver.solve(target_position, core.arm.q)
        
        if not success:
            logger.debug(f"IK {arm} failed: {debug_msg}")
            
        return np.rad2deg(q_new[:3])
    
    def clamp_joint_angles(self, joint_angles: np.ndarray) -> np.ndarray:
        """Clamp joint angles to safe limits with margins for problem joints."""
        # Create a copy to avoid modifying the original
        processed_angles = joint_angles.copy()
        
        # First, normalize angles that can wrap around (like shoulder_pan)
        # Check if first joint (shoulder_pan) is outside limits but can be wrapped
        shoulder_pan_idx = 0
        shoulder_pan_angle = processed_angles[shoulder_pan_idx]
        min_limit = self.joint_limits_min_deg[shoulder_pan_idx]  # -120.3°
        max_limit = self.joint_limits_max_deg[shoulder_pan_idx]  # +120.3°
        
        # Try to wrap the angle to an equivalent angle within limits
        if shoulder_pan_angle < min_limit or shoulder_pan_angle > max_limit:
            # Try wrapping by ±360°
            for offset in [-360.0, 360.0]:
                wrapped_angle = shoulder_pan_angle + offset
                if min_limit <= wrapped_angle <= max_limit:
                    logger.debug(f"Wrapped shoulder_pan from {shoulder_pan_angle:.1f}° to {wrapped_angle:.1f}°")
                    processed_angles[shoulder_pan_idx] = wrapped_angle
                    break
        
        # Apply standard joint limits to all joints
        return np.clip(processed_angles, self.joint_limits_min_deg, self.joint_limits_max_deg)
    
    def update_arm_angles(self, arm: str, ik_angles: np.ndarray, 
                        wrist_flex: float,  # J4: Pitch
                        wrist_yaw: float,   # J5: Yaw
                        wrist_roll: float,  # J6: Roll
                        gripper: float):    # Gripper
        
        # 1. 组装 7 维硬件角度数组 [J1, J2, J3, J4, J5, J6, Gripper]
        target_angles = np.zeros(7) 
        target_angles[0:3] = ik_angles      # 前三轴 IK
        target_angles[3] = wrist_flex        # J4: Pitch
        target_angles[4] = wrist_yaw         # J5: Yaw
        target_angles[5] = wrist_roll        # J6: Roll
        target_angles[6] = gripper           # J7: Gripper
        
        # 限幅处理 (需确保 joint_limits_min_deg 也是 7 维)
        clamped_angles = self.clamp_joint_angles(target_angles)
        
        if arm == "left":
            self.left_arm_angles = clamped_angles
        else:
            self.right_arm_angles = clamped_angles

        # 2. 同步给仿真 (保持 7 维传给 ik_core)
        q_sim_7 = np.zeros(7)
        q_sim_7[0:6] = np.deg2rad(clamped_angles[0:6]) # 所有旋转轴转弧度
        q_sim_7[6] = gripper / 45.0                    # 夹爪归一化
        
        #self.ik_cores[arm].set_state_from_hardware(q_sim_7)
        #不再调用 set_state_from_hardware，避免重置 in_zero_mode
        # 改为直接更新 IK 核心的关节状态
        core = self.ik_cores[arm]
        core.arm.q[:6] = np.deg2rad(clamped_angles[:6])
        core.arm.q[6] = gripper / 45.0 * 0.04
        # 仅触发 FK 更新可视化，不重置状态
        import pinocchio as pin
        pin.framesForwardKinematics(core.arm.model, core.arm.data, core.arm.q)
        core.arm.target_pos = core.arm.data.oMf[core.arm.ik_frame_id].translation.copy()
        #强制刷新 Meshcat 可视化
        if core.viz:
            core.viz.display(core.arm.q)
            # 更新目标红球显示
            core.viz.viewer[core.namespace]["target"].set_transform(
                pin.SE3(np.eye(3), core.arm.target_pos).homogeneous
            )

    # def update_arm_angles(self, arm: str, ik_angles: np.ndarray, wrist_flex: float, wrist_roll: float, gripper: float):
    #     """Update joint angles for specified arm with IK solution and direct wrist/gripper control."""
    #     if arm == "left":
    #         target_angles = self.left_arm_angles
    #     elif arm == "right":
    #         target_angles = self.right_arm_angles
    #     else:
    #         raise ValueError(f"Invalid arm: {arm}")
        
    #     # Update first 3 joints with IK solution
    #     target_angles[:3] = ik_angles
        
    #     # Set wrist angles directly
    #     target_angles[3] = wrist_flex
    #     target_angles[4] = wrist_roll
        
    #     # Handle gripper separately (clamp to gripper limits)
    #     target_angles[5] = np.clip(gripper, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSED_ANGLE)
        
    #     # 1. 应用关节限制
    #     clamped_angles = self.clamp_joint_angles(target_angles)
    #     clamped_angles[5] = target_angles[5] # 保持夹爪原始意图
        
    #     # 2. 更新状态
    #     if arm == "left":
    #         self.left_arm_angles = clamped_angles
    #     else:
    #         self.right_arm_angles = clamped_angles

    #     # 3. 同步到仿真显示 (修复点)
    #     # 不要直接调 .viz.display，因为那是 8 维的
    #     # 使用 ik_core 提供的逻辑来处理这种 6 到 8 的转换
    #     core = self.ik_cores[arm]
    #     target_angles_rad = np.deg2rad(clamped_angles)
        
    #     # 构造 8 维向量
    #     q_full = np.zeros(core.arm.model.nq) # 长度为 8
    #     q_full[:6] = target_angles_rad[:6]
    #     if core.arm.model.nq >= 8:
    #         # 简单映射夹爪到两个手指
    #         gripper_val = (clamped_angles[5] / 45.0) * 0.04 
    #         q_full[6] = gripper_val
    #         q_full[7] = -gripper_val

    #     # 执行显示
    #     if core.viz:
    #         core.viz.display(q_full)        
    
    def engage(self) -> bool:
        """Engage robot motors (start sending commands)."""
        if not self.is_connected:
            logger.warning("Cannot engage robot: not connected")
            return False
        
        self.is_engaged = True
        logger.info("🔌 Robot motors ENGAGED - commands will be sent")
        return True
    
    def disengage(self) -> bool:
        """Disengage robot motors (stop sending commands)."""
        if not self.is_connected:
            logger.info("Robot already disconnected")
            return True
        
        try:
            # Return to safe position before disengaging
            self.return_to_initial_position()
            
            # Disable torque
            self.disable_torque()
            
            self.is_engaged = False
            logger.info("🔌 Robot motors DISENGAGED - commands stopped")
            return True
            
        except Exception as e:
            logger.error(f"Error disengaging robot: {e}")
            return False
    
    def send_command(self) -> bool:
        """Send current joint angles to robot using dictionary format."""
        if not self.is_connected or not self.is_engaged:
            return False
        
        current_time = time.time()
        if current_time - self.last_send_time < self.config.send_interval:
            return True  # Don't send too frequently
        
        try:
            # Send commands with dictionary format - no joint direction mapping
            success = True
            
            # Send left arm command
            if self.left_robot and self.left_arm_connected:
                try:
                    # action_dict = {
                    #     "shoulder_pan.pos": float(self.left_arm_angles[0]),
                    #     "shoulder_lift.pos": float(self.left_arm_angles[1]),
                    #     "elbow_flex.pos": float(self.left_arm_angles[2]),
                    #     "wrist_flex.pos": float(self.left_arm_angles[3]),
                    #     "wrist_yaw.pos": float(self.left_arm_angles[4]), # 确保这里是 yaw
                    #     "wrist_roll.pos": float(self.left_arm_angles[5]), # 增加 roll 指令
                    #     "gripper.pos": float(self.left_arm_angles[6])  # 夹爪移到索引 6
                    # }
                    # self.left_robot.send_action(action_dict)
                    # 准备驱动需要的数组: [j1..j6弧度, gripper归一化0-1]
                    drive_action = np.zeros(7, dtype=np.float32)
                    drive_action[:6] = np.deg2rad(self.left_arm_angles[:6])
                    drive_action[6] = self.left_arm_angles[6] / 45.0
                    # 调用最新驱动的接口，它内部会处理 HARDWARE_DIR 和限位
                    self.left_robot.send_action(drive_action)
                except Exception as e:
                    logger.error(f"Error sending left arm command: {e}")
                    self.left_arm_errors += 1
                    if self.left_arm_errors > self.max_arm_errors:
                        self.left_arm_connected = False
                        logger.error("❌ Left arm disconnected due to repeated errors")
                    success = False
            
            # Send right arm command
            if self.right_robot and self.right_arm_connected:
                try:
                    # action_dict = {
                    #     "shoulder_pan.pos": float(self.right_arm_angles[0]),
                    #     "shoulder_lift.pos": float(self.right_arm_angles[1]),
                    #     "elbow_flex.pos": float(self.right_arm_angles[2]),
                    #     "wrist_flex.pos": float(self.right_arm_angles[3]),
                    #     "wrist_yaw.pos": float(self.right_arm_angles[4]), # 确保这里是 yaw
                    #     "wrist_roll.pos": float(self.right_arm_angles[5]), # 增加 roll 指令
                    #     "gripper.pos": float(self.right_arm_angles[6])  # 夹爪移到索引 6
                    # }
                    # self.right_robot.send_action(action_dict)
                    drive_action = np.zeros(7, dtype=np.float32)
                    drive_action[:6] = np.deg2rad(self.right_arm_angles[:6])
                    drive_action[6] = self.right_arm_angles[6] / 45.0
                    # 调用最新驱动的接口，它内部会处理 HARDWARE_DIR 和限位
                    self.right_robot.send_action(drive_action)
                except Exception as e:
                    logger.error(f"Error sending right arm command: {e}")
                    self.right_arm_errors += 1
                    if self.right_arm_errors > self.max_arm_errors:
                        self.right_arm_connected = False
                        logger.error("❌ Right arm disconnected due to repeated errors")
                    success = False
            
            self.last_send_time = current_time
            return success
            
        except Exception as e:
            logger.error(f"Error sending robot command: {e}")
            self.general_errors += 1
            if self.general_errors > self.max_general_errors:
                self.is_connected = False
                logger.error("❌ Robot interface disconnected due to repeated errors")
            return False
    
    def set_gripper(self, arm: str, closed: bool):
        """Set gripper state for specified arm."""
        angle = GRIPPER_CLOSED_ANGLE if closed else GRIPPER_OPEN_ANGLE
        
        if arm == "left":
            self.left_arm_angles[GRIPPER_INDEX] = angle
        elif arm == "right":
            self.right_arm_angles[GRIPPER_INDEX] = angle
        else:
            raise ValueError(f"Invalid arm: {arm}")
    
    def get_arm_angles(self, arm: str) -> np.ndarray:
        """Get current joint angles for specified arm."""
        if arm == "left":
            angles = self.left_arm_angles.copy()
        elif arm == "right":
            angles = self.right_arm_angles.copy()
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        return angles
    
    def get_arm_angles_for_visualization(self, arm: str) -> np.ndarray:
        """Get current joint angles for specified arm, for PyBullet visualization."""
        # Return raw angles without any correction for proper diagnosis
        return self.get_arm_angles(arm)
    
    # def get_actual_arm_angles(self, arm: str) -> np.ndarray:
    #     """Get actual joint angles from robot hardware (not commanded angles)."""
    #     try:
    #         if arm == "left" and self.left_robot and self.left_arm_connected:
    #             observation = self.left_robot.get_observation()
    #             if observation:
    #                 return np.array([
    #                     observation['shoulder_pan.pos'],
    #                     observation['shoulder_lift.pos'],
    #                     observation['elbow_flex.pos'],
    #                     observation['wrist_flex.pos'],
    #                     observation['wrist_yaw.pos'],
    #                     observation['wrist_roll.pos'],
    #                     observation['gripper.pos']
    #                 ])
    #         elif arm == "right" and self.right_robot and self.right_arm_connected:
    #             observation = self.right_robot.get_observation()
    #             if observation:
    #                 return np.array([
    #                     observation['shoulder_pan.pos'],
    #                     observation['shoulder_lift.pos'],
    #                     observation['elbow_flex.pos'],
    #                     observation['wrist_flex.pos'],
    #                     observation['wrist_yaw.pos'],
    #                     observation['wrist_roll.pos'],
    #                     observation['gripper.pos']
    #                 ])
    #     except Exception as e:
    #         logger.debug(f"Error reading actual arm angles for {arm}: {e}")
        
    #     # Fallback to commanded angles if we can't read actual angles
    #     return self.get_arm_angles(arm)
    def get_actual_arm_angles(self, arm: str) -> np.ndarray:
        try:
            robot = self.left_robot if arm == "left" else self.right_robot
            if robot and self.get_arm_connection_status(arm):
                obs = robot.get_observation()
                sim_state = obs["state"]
                
                # 返回角度格式供 UI/可视化使用
                actual_deg = np.zeros(7)
                actual_deg[:6] = np.rad2deg(sim_state[:6])
                actual_deg[6] = sim_state[6] * 45.0
                return actual_deg
        except Exception as e:
            logger.debug(f"Error reading actual angles: {e}")
        return self.get_arm_angles(arm)


    def return_to_initial_position(self):
        """Return both arms to initial position."""
        logger.info("⏪ Returning robot to initial position...")
        
        try:
            # Set initial positions - no direction mapping
            self.left_arm_angles = self.initial_left_arm.copy()
            self.right_arm_angles = self.initial_right_arm.copy()
            
            # Send commands for a few iterations to ensure movement
            for i in range(10):
                self.send_command()
                time.sleep(0.1)
                
            logger.info("✅ Robot returned to initial position")
        except Exception as e:
            logger.error(f"Error returning to initial position: {e}")
    
    def disable_torque(self, arm: str = None):
        """Disable torque on robot joints.

        Args:
            arm: 'left', 'right', or None for both arms
        """
        if not self.is_connected:
            return

        try:
            if arm is None or arm == "left":
                if self.left_robot and self.left_arm_connected:
                    logger.info("Disabling torque on LEFT arm...")
                    self.left_robot.bus.disable_torque()

            if arm is None or arm == "right":
                if self.right_robot and self.right_arm_connected:
                    logger.info("Disabling torque on RIGHT arm...")
                    self.right_robot.bus.disable_torque()

        except Exception as e:
            logger.error(f"Error disabling torque: {e}")
    
    def disconnect(self):
        """Disconnect from robot hardware."""
        if not self.is_connected:
            return
        
        logger.info("Disconnecting from robot...")
        
        # Return to initial positions if engaged
        if self.is_engaged:
            try:
                self.return_to_initial_position()
            except Exception as e:
                logger.error(f"Error returning to initial position: {e}")
        
        # Disconnect both arms
        if self.left_robot:
            try:
                self.left_robot.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting left arm: {e}")
            self.left_robot = None
            
        if self.right_robot:
            try:
                self.right_robot.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting right arm: {e}")
            self.right_robot = None
        
        self.is_connected = False
        self.is_engaged = False
        self.left_arm_connected = False
        self.right_arm_connected = False
        logger.info("🔌 Robot disconnected")
    
    def get_arm_connection_status(self, arm: str) -> bool:
        """Get connection status for specific arm based on device file existence."""
        # Only check device file existence - ignore overall robot connection status
        if arm == "left":
            device_path = self.config.follower_ports["left"]
            return os.path.exists(device_path)
        elif arm == "right":
            device_path = self.config.follower_ports["right"] 
            return os.path.exists(device_path)
        else:
            return False

    def update_arm_connection_status(self):
        """Update individual arm connection status based on device file existence."""
        if self.is_connected:
            self.left_arm_connected = os.path.exists(self.config.follower_ports["left"])
            self.right_arm_connected = os.path.exists(self.config.follower_ports["right"])
    
    @property
    def status(self) -> Dict:
        """Get robot status information."""
        return {
            "connected": self.is_connected,
            "left_arm_connected": self.left_arm_connected,
            "right_arm_connected": self.right_arm_connected,
            "left_arm_angles": self.left_arm_angles.tolist(),
            "right_arm_angles": self.right_arm_angles.tolist(),
            "joint_limits_min": self.joint_limits_min_deg.tolist(),
            "joint_limits_max": self.joint_limits_max_deg.tolist(),
        } 