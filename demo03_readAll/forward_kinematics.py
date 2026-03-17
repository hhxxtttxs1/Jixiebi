import numpy as np

# 定义旋转矩阵函数
def rotate_x(theta):
    """绕X轴旋转的旋转矩阵"""
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])

def rotate_y(theta):
    """绕Y轴旋转的旋转矩阵"""
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

def rotate_z(theta):
    """绕Z轴旋转的旋转矩阵"""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

# 定义平移矩阵函数
def translate(x, y, z):
    """平移变换矩阵"""
    return np.array([x, y, z])

# 机械臂运动学正解函数
def forward_kinematics(joint_angles):
    """
    机械臂运动学正解
    joint_angles: 6个关节的角度列表 [theta1, theta2, theta3, theta4, theta5, theta6]
    返回末端夹爪的位置坐标 [x, y, z]
    """
    # 提取关节角度
    theta1, theta2, theta3, theta4, theta5, theta6 = joint_angles
    
    # 初始化位置
    position = np.array([0, 0, 0])
    orientation = np.eye(3)  # 初始姿态为单位矩阵
    
    # 关节1: 腰部旋转 (绕X轴)
    # 变换: 先平移，再旋转
    trans1 = translate(-0.013, 0, 0.0265)
    rot1 = rotate_x(theta1) @ rotate_y(-np.pi/2)  # 考虑URDF中的rpy="0 -1.57 0"
    position = position + orientation @ trans1
    orientation = orientation @ rot1
    
    # 关节2: 大臂控制 (绕Y轴)
    trans2 = translate(0.081, 0, 0)
    rot2 = rotate_y(np.pi/2) @ rotate_y(theta2)  # 考虑URDF中的rpy="0 1.57 0"
    position = position + orientation @ trans2
    orientation = orientation @ rot2
    
    # 关节3: 小臂控制 (绕Y轴)
    trans3 = translate(0, 0, 0.118)
    rot3 = rotate_y(theta3)
    position = position + orientation @ trans3
    orientation = orientation @ rot3
    
    # 关节4: 腕部控制 (绕Y轴)
    trans4 = translate(0, 0, 0.118)
    rot4 = rotate_y(theta4)
    position = position + orientation @ trans4
    orientation = orientation @ rot4
    
    # 关节5: 腕部旋转 (绕Z轴)
    trans5 = translate(0, 0, 0.0635)
    rot5 = rotate_z(theta5)
    position = position + orientation @ trans5
    orientation = orientation @ rot5
    
    # 关节6: 爪子控制 (绕X轴)
    trans6 = translate(0, -0.0132, 0.021)
    rot6 = rotate_x(theta6)
    position = position + orientation @ trans6
    orientation = orientation @ rot6
    
    return position

# 测试代码
if __name__ == "__main__":
    # 测试用例1: 所有关节角度为0
    joint_angles_0 = [0, 0, 0, 0, 0, 0]
    end_effector_pos_0 = forward_kinematics(joint_angles_0)
    print("测试用例1 - 所有关节角度为0:")
    print(f"末端夹爪位置: {end_effector_pos_0}")
    
    # 测试用例2: 部分关节角度不为0
    joint_angles_1 = [np.pi/4, np.pi/6, np.pi/3, np.pi/4, np.pi/2, np.pi/6]
    end_effector_pos_1 = forward_kinematics(joint_angles_1)
    print("\n测试用例2 - 部分关节角度不为0:")
    print(f"关节角度: {joint_angles_1}")
    print(f"末端夹爪位置: {end_effector_pos_1}")
