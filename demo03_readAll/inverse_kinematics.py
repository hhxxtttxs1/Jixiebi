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

# 计算雅可比矩阵
def calculate_jacobian(joint_angles, delta=1e-6):
    """
    计算雅可比矩阵
    joint_angles: 当前关节角度
    delta: 微小扰动值
    返回雅可比矩阵 (3x6)
    """
    current_pos = forward_kinematics(joint_angles)
    jacobian = np.zeros((3, 6))
    
    # 对每个关节进行微小扰动，计算位置变化
    for i in range(6):
        perturbed_angles = joint_angles.copy()
        perturbed_angles[i] += delta
        perturbed_pos = forward_kinematics(perturbed_angles)
        jacobian[:, i] = (perturbed_pos - current_pos) / delta
    
    return jacobian

# 机械臂运动学逆解函数（Levenberg-Marquardt算法）
def inverse_kinematics(target_position, initial_angles=None, max_iterations=1000, tolerance=1e-5, lambda_init=1e-3):
    """
    机械臂运动学逆解（Levenberg-Marquardt算法）
    target_position: 目标位置 [x, y, z]
    initial_angles: 初始关节角度，默认为全0
    max_iterations: 最大迭代次数
    tolerance: 位置误差阈值
    lambda_init: 初始阻尼因子
    返回求解得到的关节角度
    """
    # 初始化关节角度
    if initial_angles is None:
        joint_angles = np.zeros(6)
    else:
        joint_angles = np.array(initial_angles)
    
    lambda_ = lambda_init
    
    for i in range(max_iterations):
        # 计算当前位置
        current_pos = forward_kinematics(joint_angles)
        
        # 计算位置误差
        error = target_position - current_pos
        error_norm = np.linalg.norm(error)
        
        # 检查是否达到精度要求
        if error_norm < tolerance:
            print(f"迭代 {i} 次后达到精度要求")
            return joint_angles
        
        # 计算雅可比矩阵
        jacobian = calculate_jacobian(joint_angles)
        
        # Levenberg-Marquardt算法
        J = jacobian
        JTJ = J.T @ J
        JTe = J.T @ error
        
        # 添加阻尼项
        identity = np.eye(JTJ.shape[0])
        delta_angles = np.linalg.solve(JTJ + lambda_ * identity, JTe)
        
        # 尝试更新
        new_angles = joint_angles + delta_angles
        new_angles = np.clip(new_angles, -np.pi, np.pi)
        new_pos = forward_kinematics(new_angles)
        new_error = target_position - new_pos
        new_error_norm = np.linalg.norm(new_error)
        
        # 调整阻尼因子
        if new_error_norm < error_norm:
            # 接受更新，减小阻尼因子
            joint_angles = new_angles
            lambda_ *= 0.1
        else:
            # 拒绝更新，增大阻尼因子
            lambda_ *= 10
        
        # 每100次迭代打印一次误差
        if i % 100 == 0:
            print(f"迭代 {i} 次，误差: {error_norm}, 阻尼因子: {lambda_}")
    
    print(f"达到最大迭代次数 {max_iterations}，未达到精度要求")
    return joint_angles

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
    
    # 测试逆解函数 - 测试用例1: 目标位置为所有关节角度为0时的位置
    print("\n测试逆解函数 - 测试用例1:")
    target_pos_0 = end_effector_pos_0
    print(f"目标位置: {target_pos_0}")
    
    # 使用全0作为初始角度
    solved_angles_0 = inverse_kinematics(target_pos_0)
    print(f"求解得到的关节角度: {solved_angles_0}")
    
    # 验证求解结果
    verified_pos_0 = forward_kinematics(solved_angles_0)
    print(f"验证位置: {verified_pos_0}")
    print(f"位置误差: {np.linalg.norm(target_pos_0 - verified_pos_0)}")
    
    # 测试逆解函数 - 测试用例2: 目标位置为测试用例2的位置
    print("\n测试逆解函数 - 测试用例2:")
    target_pos_1 = end_effector_pos_1
    print(f"目标位置: {target_pos_1}")
    
    # 使用全0作为初始角度
    solved_angles_1 = inverse_kinematics(target_pos_1)
    print(f"求解得到的关节角度: {solved_angles_1}")
    
    # 验证求解结果
    verified_pos_1 = forward_kinematics(solved_angles_1)
    print(f"验证位置: {verified_pos_1}")
    print(f"位置误差: {np.linalg.norm(target_pos_1 - verified_pos_1)}")
    
    # 测试逆解函数 - 测试用例3: 随机目标位置
    print("\n测试逆解函数 - 测试用例3 (随机目标位置):")
    # 生成一个合理的随机目标位置
    # 基于机械臂的工作空间，我们可以选择一个在合理范围内的位置
    random_target = np.array([0.1, 0.1, 0.3])
    print(f"目标位置: {random_target}")
    
    # 使用全0作为初始角度
    solved_angles_2 = inverse_kinematics(random_target)
    print(f"求解得到的关节角度: {solved_angles_2}")
    
    # 验证求解结果
    verified_pos_2 = forward_kinematics(solved_angles_2)
    print(f"验证位置: {verified_pos_2}")
    print(f"位置误差: {np.linalg.norm(random_target - verified_pos_2)}")