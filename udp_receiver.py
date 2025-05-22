import socket
import time
from datetime import datetime

def main():
    # 创建 UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # 绑定到所有接口的 12345 端口
    server_address = ('', 12345)
    sock.bind(server_address)
    
    print("🤖 Robot Detection UDP Server")
    print("=" * 50)
    print("Server started on port 12345")
    print("Waiting for detection messages from robot...")
    print("=" * 50)
    
    try:
        detection_count = 0
        while True:
            # 接收数据
            data, address = sock.recvfrom(1024)
            message = data.decode('utf-8')
            
            # 显示接收时间
            current_time = datetime.now().strftime("%H:%M:%S")
            detection_count += 1
            
            print(f"\n[{current_time}] Detection #{detection_count}")
            print(f"From Robot IP: {address[0]}")
            
            # 解析消息格式: "ROBOT_DETECTION:TYPE:INFO:TIME:timestamp"
            if "ROBOT_DETECTION:" in message:
                try:
                    parts = message.split(":")
                    if len(parts) >= 4:
                        object_type = parts[1]
                        object_info = parts[2]
                        robot_timestamp = float(parts[4]) if len(parts) > 4 else 0
                        
                        print(f"🔍 OBJECT DETECTED: {object_type}")
                        if object_info:
                            print(f"📝 Details: {object_info}")
                        
                        # 显示延迟信息
                        if robot_timestamp > 0:
                            delay = time.time() - robot_timestamp
                            print(f"⏱️  Network delay: {delay:.3f} seconds")
                        
                    else:
                        print(f"📨 Raw message: {message}")
                except Exception as e:
                    print(f"❌ Parse error: {e}")
                    print(f"📨 Raw message: {message}")
            else:
                print(f"📨 Unknown message format: {message}")
            
            print("-" * 30)
            
    except KeyboardInterrupt:
        print(f"\n\n🛑 Server stopping...")
        print(f"Total detections received: {detection_count}")
    finally:
        sock.close()
        print("UDP server closed.")

if __name__ == "__main__":
    main()