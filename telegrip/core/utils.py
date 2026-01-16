import numpy as np

def compute_relative_position(current_vr_pos: dict, origin_vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """
    计算从 VR 起点到当前位置的相对位移，并映射到机器人坐标系。
    移除了对 PyBullet 的所有依赖。
    
    映射逻辑说明（基于原版 kinematics.py）:
    - Robot X (前) = -VR delta_X * scale
    - Robot Y (左) =  VR delta_Z * scale
    - Robot Z (上) =  VR delta_Y * scale
    """
    # 1. 计算 VR 空间中的增量
    dx = current_vr_pos['x'] - origin_vr_pos['x']
    dy = current_vr_pos['y'] - origin_vr_pos['y']
    dz = current_vr_pos['z'] - origin_vr_pos['z']

    # 2. 按照你原代码中的 vr_to_robot_coordinates 逻辑进行转换
    # 注意：原代码注释中写的是 VR +Z -> Robot +X，但代码实现是 -dx -> Robot X
    # 这里我们严格遵循代码实现，确保你的遥操作手感不变
    return np.array([
        -dx * scale,  # Robot X
         dz * scale,  # Robot Y
         dy * scale   # Robot Z
    ])