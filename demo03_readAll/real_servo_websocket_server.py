import asyncio
import websockets
import json
import time
from read_all_servos import ServoAngleReader

class RealServoWebSocketServer:
    def __init__(self, port='/dev//ttyUSB0', websocket_port=8000):
        """初始化真实舵机WebSocket服务器
        
        Args:
            port: 串口端口
            websocket_port: WebSocket服务器端口
        """
        self.port = port
        self.websocket_port = websocket_port
        self.reader = ServoAngleReader(port)
        self.is_connected = False
    
    def connect_to_servos(self):
        """连接到舵机"""
        if self.reader.connect():
            self.is_connected = True
            print(f"成功连接到舵机串口 {self.port}")
            return True
        else:
            print(f"无法连接到舵机串口 {self.port}")
            return False
    
    def disconnect_from_servos(self):
        """断开与舵机的连接"""
        if self.is_connected:
            self.reader.disconnect()
            self.is_connected = False
            print("已断开与舵机的连接")
    
    async def handle_connection(self, websocket):
        """处理WebSocket连接"""
        print(f"客户端已连接: ")
        
        try:
            while True:
                if not self.is_connected:
                    # 如果未连接到舵机，尝试重新连接
                    if not self.connect_to_servos():
                        # 构建错误数据结构
                        error_data = {
                            "error": "无法连接到舵机",
                            "servos": [0.0 for _ in range(6)]
                        }
                        await websocket.send(json.dumps(error_data))
                        print("发送错误数据: 无法连接到舵机")
                        await asyncio.sleep(1)
                        continue
                
                # 读取真实舵机角度
                servo_angles = []
                for motor_id in range(1, 7):
                    angle = self.reader.read_angle(motor_id)
                    # 如果读取失败，使用0.0作为默认值
                    if angle is not None:
                        # 根据测试结果，1号、2号和5号舵机的角度需要取反
                        if motor_id in [1, 2, 5]:
                            angle = -angle
                        servo_angles.append(angle)
                    else:
                        servo_angles.append(0.0)
                
                # 构建数据结构
                data = {
                    "servos": servo_angles
                }
                
                # 发送数据给客户端
                await websocket.send(json.dumps(data))
                print(f"发送真实舵机角度数据: {servo_angles}")
                
                # 等待0.1秒后发送下一组数据（提高实时性）
                await asyncio.sleep(0.1)
        except websockets.exceptions.ConnectionClosedError:
            print("客户端连接已关闭")
        except Exception as e:
            print(f"发生错误: {e}")
    
    async def start_server(self):
        """启动WebSocket服务器"""
        # 先尝试连接到舵机
        self.connect_to_servos()
        
        server = await websockets.serve(
            self.handle_connection,
            "localhost",  # 服务器地址
            self.websocket_port  # 服务器端口
        )
        
        print(f"真实舵机WebSocket服务器已启动，监听端口{self.websocket_port}...")
        
        # 保持服务器运行
        await server.wait_closed()
    
    def run(self):
        """运行服务器"""
        try:
            asyncio.run(self.start_server())
        except KeyboardInterrupt:
            print("\n服务器已停止")
        finally:
            # 断开与舵机的连接
            self.disconnect_from_servos()

if __name__ == "__main__":
    # 配置参数
    port = '/dev//ttyUSB0'  # 根据实际情况修改串口
    websocket_port = 8000  # WebSocket服务器端口
    
    # 创建并运行服务器
    server = RealServoWebSocketServer(port=port, websocket_port=websocket_port)
    server.run()
