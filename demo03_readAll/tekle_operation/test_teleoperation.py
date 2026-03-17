#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
遥操作系统测试脚本
Teleoperation System Test Script
"""
import time
import sys
from teleoperation_system import TeleoperationSystem


def test_connections():
    """测试连接功能"""
    print("=" * 50)
    print("测试连接功能")
    print("=" * 50)
    
    # 配置参数
    TEACHER_PORT = "/dev/ttyUSB0"
    STUDENT_PORT = "/dev/ttyACM0"
    
    # 创建遥操作系统
    teleop = TeleoperationSystem(
        teacher_port=TEACHER_PORT,
        student_port=STUDENT_PORT
    )
    
    # 测试教师端连接
    print("1. 测试教师端连接...")
    if teleop.connect_teacher():
        print("✓ 教师端连接成功")
        
        # 测试读取角度
        print("2. 测试读取教师端角度...")
        try:
            angles = teleop.read_teacher_angles()
            print("读取到的角度:")
            for servo_id, angle in angles.items():
                if angle is not None:
                    print(f"  舵机 {servo_id}: {angle:.2f}°")
                else:
                    print(f"  舵机 {servo_id}: 读取失败")
        except Exception as e:
            print(f"✗ 读取角度失败: {e}")
        
        # 断开教师端
        teleop.teacher_reader.disconnect()
        print("✓ 教师端已断开")
    else:
        print("✗ 教师端连接失败")
    
    print()
    
    # 测试学生端连接
    print("3. 测试学生端连接...")
    if teleop.connect_student():
        print("✓ 学生端连接成功")
        
        # 测试设置角度
        print("4. 测试设置学生端角度...")
        test_angles = {1: 0.0, 2: 10.0, 3: -10.0, 4: 20.0, 5: -20.0, 6: 0.0}
        success_count = teleop.send_angles_to_student(test_angles)
        print(f"成功设置 {success_count}/{len(test_angles)} 个舵机角度")
        
        # 断开学生端
        teleop.disconnect_all()
        print("✓ 学生端已断开")
    else:
        print("✗ 学生端连接失败")
    
    print("\n连接测试完成")


def test_short_teleop():
    """测试短时间遥操作"""
    print("=" * 50)
    print("测试短时间遥操作 (10秒)")
    print("=" * 50)
    
    # 配置参数
    TEACHER_PORT = "/dev/ttyUSB0"
    STUDENT_PORT = "/dev/ttyACM0"
    
    # 创建遥操作系统
    teleop = TeleoperationSystem(
        teacher_port=TEACHER_PORT,
        student_port=STUDENT_PORT
    )
    
    # 设置较短的测试时间
    original_frequency = teleop.update_frequency
    teleop.update_frequency = 5  # 降低到5Hz便于观察
    
    try:
        # 启动遥操作
        if teleop.connect_teacher() and teleop.connect_student():
            print("开始10秒遥操作测试...")
            teleop.is_running = True
            
            start_time = time.time()
            while time.time() - start_time < 10:  # 运行10秒
                try:
                    # 读取教师端角度
                    teacher_angles = teleop.read_teacher_angles()
                    
                    # 发送到学生端
                    sent_count = teleop.send_angles_to_student(teacher_angles)
                    
                    # 打印状态
                    print(f"教师端: ", end="")
                    for servo_id in range(1, 4):  # 只显示前3个舵机
                        angle = teacher_angles[servo_id]
                        if angle is not None:
                            print(f"#{servo_id}:{angle:.1f}° ", end="")
                    print(f"| 发送: {sent_count}/6")
                    
                    time.sleep(0.2)  # 200ms间隔
                    
                except Exception as e:
                    print(f"错误: {e}")
                    break
            
            teleop.is_running = False
            print("✓ 短时间遥操作测试完成")
        else:
            print("✗ 连接失败，无法进行遥操作测试")
    
    finally:
        teleop.disconnect_all()
        teleop.update_frequency = original_frequency


def main():
    """主测试函数"""
    print("遥操作系统测试")
    print("请确保:")
    print("1. 教师端设备连接到 /dev/ttyUSB0")
    print("2. 学生端设备连接到 /dev/ttyACM0")
    print("3. 两个设备都已通电并正常工作")
    print()
    
    try:
        choice = input("选择测试类型:\n1. 连接测试\n2. 短时间遥操作测试\n3. 跳过测试\n请输入 (1/2/3): ").strip()
        
        if choice == "1":
            test_connections()
        elif choice == "2":
            test_short_teleop()
        elif choice == "3":
            print("跳过测试")
        else:
            print("无效选择")
    
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")


if __name__ == '__main__':
    main()
