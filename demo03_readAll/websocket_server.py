import asyncio
import websockets
import json
import random
import time

# 模拟舵机角度数据
def generate_servo_angles():
    """生成6个舵机的角度数据，范围-90到90度"""
    return [random.uniform(-90, 90) for _ in range(6)]

async def handle_connection(websocket):
    """处理WebSocket连接"""
    print(f"客户端已连接: ")
    
    try:
        while True:
            # 生成随机舵机角度
            servo_angles = generate_servo_angles()
            
            # 构建数据结构
            data = {
                "servos": servo_angles
            }
            
            # 发送数据给客户端
            await websocket.send(json.dumps(data))
            print(f"发送舵机角度数据: {servo_angles}")
            
            # 等待1秒后发送下一组数据
            await asyncio.sleep(1)
    except websockets.exceptions.ConnectionClosedError:
        print("客户端连接已关闭")
    except Exception as e:
        print(f"发生错误: {e}")

async def main():
    """启动WebSocket服务器"""
    server = await websockets.serve(
        handle_connection,
        "localhost",  # 服务器地址
        8000         # 服务器端口
    )
    
    print("WebSocket服务器已启动，监听端口8000...")
    
    # 保持服务器运行
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())