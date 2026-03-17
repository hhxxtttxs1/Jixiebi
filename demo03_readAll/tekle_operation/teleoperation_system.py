#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
遥操作系统 - 读取教师端舵机角度并发送到学生端
Teleoperation System - Reads teacher servo angles and sends to student servos
"""
import time

import threading

# Import teacher servo reader (from read_all_servos.py)
from read_all_servos import ServoAngleReader

# Import student servo controller (from write_all_student_angle.py)
from write_all_student_angle import ServoController


class TeleoperationSystem:
    def __init__(self, teacher_port="/dev/ttyUSB0", student_port="/dev/ttyACM0"):
        """
        初始化遥操作系统
        
        Args:
            teacher_port: 教师端串口
            student_port: 学生端串口
        """
        self.teacher_port = teacher_port
        self.student_port = student_port
        self.servo_ids = list(range(1, 7))  # 1-6号舵机
        
        # 初始化教师端舵机角度读取器
        self.teacher_reader = ServoAngleReader(teacher_port)
        
        # 初始化学生端舵机控制器
        self.student_controller = None
        
        # 系统状态
        self.is_running = False
        self.teacher_connected = False
        self.student_connected = False
        
        # 角度缓存
        self.current_angles = {i: 0.0 for i in self.servo_ids}
        self.angle_lock = threading.Lock()
        
        # 控制参数
        self.update_frequency = 50  # Hz (每秒更新20次)
        self.angle_tolerance = 1.0   # 角度容差(度)，小于此值不发送

    def _transform_student_angle(self, servo_id, angle):
        """将教师端角度转换为学生端角度"""
        if servo_id in self.inverted_student_servo_ids:
            return -angle
        return angle
        
    def connect_teacher(self):
        """连接教师端舵机"""
        try:
            if self.teacher_reader.connect():
                self.teacher_connected = True
                print(f"✓ 成功连接教师端端口: {self.teacher_port}")
                return True
            else:
                print(f"✗ 连接教师端端口失败: {self.teacher_port}")
                return False
        except Exception as e:
            print(f"✗ 连接教师端时发生错误: {e}")
            return False
    
    def connect_student(self):
        """连接学生端舵机"""
        try:
            self.student_controller = ServoController(port=self.student_port)
            self.student_connected = True
            print(f"✓ 成功连接学生端端口: {self.student_port}")
            
            # 启用所有舵机的扭矩
            print("正在启用学生端舵机扭矩...")
            for servo_id in self.servo_ids:
                if self.student_controller.enable_torque(servo_id):
                    print(f"  舵机 {servo_id} 扭矩已启用")
                else:
                    print(f"  舵机 {servo_id} 扭矩启用失败")
                time.sleep(0.05)
            
            return True
        except Exception as e:
            print(f"✗ 连接学生端时发生错误: {e}")
            return False
    
    def disconnect_all(self):
        """断开所有连接"""
        print("正在断开连接...")
        
        if self.teacher_connected:
            self.teacher_reader.disconnect()
            self.teacher_connected = False
            print("✓ 教师端已断开连接")
        
        if self.student_connected and self.student_controller:
            self.student_controller.close()
            self.student_connected = False
            print("✓ 学生端已断开连接")
    
    def read_teacher_angles(self):
        """读取教师端所有舵机角度"""
        angles = {}
        for servo_id in self.servo_ids:
            angle = self.teacher_reader.read_angle(servo_id)
            angles[servo_id] = angle if angle is not None else 0.0
        return angles
    
    def send_angles_to_student(self, angles):
        """发送角度到学生端舵机"""
        success_count = 0
        for servo_id, angle in angles.items():
            if angle is None:
                continue
            
            # 检查角度变化是否超过容差
            with self.angle_lock:
                angle_diff = abs(angle - self.current_angles[servo_id])
                
            
            if angle_diff < self.angle_tolerance:
                continue  # 角度变化太小，跳过
            
            # 对3,4号舵机角度进行去反
            if servo_id in [3, 4]:
                angle = -angle
            
            # 发送角度到学生端
            if self.student_controller.set_servo_angle(servo_id, angle):
                with self.angle_lock:
                    self.current_angles[servo_id] = angle
                success_count += 1
            else:
                print(f"⚠ 设置舵机 {servo_id} 角度失败: {angle:.2f}°")
        
        return success_count
    
    def teleoperation_loop(self):
        """遥操作主循环"""
        print(f"开始遥操作循环 (更新频率: {self.update_frequency} Hz)")
        print("按 Ctrl+C 停止遥操作")
        print("-" * 50)
        
        last_print_time = time.time()
        print_interval = 2.0  # 每2秒打印一次状态
        
        while self.is_running:
            try:
                loop_start_time = time.time()
                
                # 读取教师端角度
                teacher_angles = self.read_teacher_angles()
                
                # 发送到学生端
                sent_count = self.send_angles_to_student(teacher_angles)
                
                # 定期打印状态
                current_time = time.time()
                if current_time - last_print_time >= print_interval:
                    print(f"教师端角度: ", end="")
                    for servo_id in self.servo_ids:
                        angle = teacher_angles[servo_id]
                        if angle is not None:
                            print(f"#{servo_id}:{angle:.1f}° ", end="")
                    print(f"| 已发送: {sent_count}/{self.servo_ids}")
                    last_print_time = current_time
                
                # 控制循环频率
                loop_time = time.time() - loop_start_time
                target_period = 1.0 / self.update_frequency
                if loop_time < target_period:
                    time.sleep(target_period - loop_time)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"✗ 遥操作循环中发生错误: {e}")
                time.sleep(0.1)  # 错误后短暂暂停
    
    def start(self):
        """启动遥操作系统"""
        print("=" * 60)
        print("机械臂遥操作系统启动")
        print("=" * 60)
        
        # 连接教师端
        if not self.connect_teacher():
            return False
        
        # 连接学生端
        if not self.connect_student():
            self.disconnect_all()
            return False
        
        # 启动遥操作
        self.is_running = True
        try:
            self.teleoperation_loop()
        except KeyboardInterrupt:
            print("\n收到停止信号")
        finally:
            self.is_running = False
            self.disconnect_all()
        
        return True
    
    def stop(self):
        """停止遥操作系统"""
        self.is_running = False


def main():
    """主函数"""
    # 配置参数 - 根据实际情况修改
    TEACHER_PORT = "/dev/ttyUSB0"  # 教师端串口
    STUDENT_PORT = "/dev/ttyACM0"  # 学生端串口
    
    # 创建遥操作系统
    teleop = TeleoperationSystem(
        teacher_port=TEACHER_PORT,
        student_port=STUDENT_PORT
    )
    
    try:
        # 启动遥操作
        success = teleop.start()
        if success:
            print("遥操作系统正常结束")
        else:
            print("遥操作系统启动失败")
    except Exception as e:
        print(f"系统错误: {e}")
    finally:
        teleop.stop()


if __name__ == '__main__':
    main()
