import socket
import struct
import threading
import time
import sys

# ==================== 全局标志 ====================
running = True          # 控制线程退出

# ==================== 控制包发送函数 ====================
def send_control_packet(conn, pitchmotor_direction, pitchmotor_speed_rpm,
                        rollmotor_direction, rollmotor_speed_rpm,
                        thrustmotor_direction, thrustmotor_speed_rpm,
                        pump_direction, pump_on_off,
                        ask_for_status, clear_rx_ring):
    """打包并发送控制包"""
    pack_format = '<B I B I B I B B B B'   # 小端紧凑打包
    try:
        packet = struct.pack(pack_format,
                             pitchmotor_direction & 0xFF,
                             pitchmotor_speed_rpm & 0xFFFFFFFF,
                             rollmotor_direction & 0xFF,
                             rollmotor_speed_rpm & 0xFFFFFFFF,
                             thrustmotor_direction & 0xFF,
                             thrustmotor_speed_rpm & 0xFFFFFFFF,
                             pump_direction & 0xFF,
                             pump_on_off & 0xFF,
                             ask_for_status & 0xFF,
                             clear_rx_ring & 0xFF)
        conn.sendall(packet)
        print(f"[控制包] 已发送，长度 {len(packet)} 字节")
        return True
    except Exception as e:
        print(f"[发送错误] {e}")
        return False

# ==================== 控制包输入交互 ====================
def input_control_packet(conn):
    """引导用户输入控制包各字段并发送"""
    print("\n========== 输入控制包字段 ==========")
    fields = [
        ('pitchmotor_direction', 'uint8 (0-255)'),
        ('pitchmotor_speed_rpm', 'uint32 (0-4294967295)'),
        ('rollmotor_direction', 'uint8 (0-255)'),
        ('rollmotor_speed_rpm', 'uint32 (0-4294967295)'),
        ('thrustmotor_direction', 'uint8 (0-255)'),
        ('thrustmotor_speed_rpm', 'uint32 (0-4294967295)'),
        ('pump_direction', 'uint8 (0-255)'),
        ('pump_on_off', 'uint8 (0-255)'),
        ('ask_for_status', 'uint8 (0-255)'),
        ('clear_rx_ring', 'uint8 (0-255)')
    ]
    values = []
    for name, hint in fields:
        while True:
            try:
                val_str = input(f"{name} ({hint}): ").strip()
                val = int(val_str)
                if 'uint8' in hint and (val < 0 or val > 255):
                    print("  输入超出 0-255 范围，请重新输入。")
                    continue
                values.append(val)
                break
            except ValueError:
                print("  请输入有效的整数。")
    send_control_packet(conn, *values)

# ==================== 接收线程函数 ====================
def receive_thread(conn):
    """负责不断接收并打印来自客户端的数据"""
    global running
    while running:
        try:
            # 设置超时，以便能定期检查 running 标志
            conn.settimeout(1.0)
            data = conn.recv(4096)
            if not data:
                print("[信息] 客户端已断开连接。")
                break
            # 尝试解码为文本，如果是二进制则显示十六进制
            try:
                text = data.decode('utf-8').strip()
                print(f"\n[客户端] {text}")
            except UnicodeDecodeError:
                print(f"\n[客户端] (二进制) {data.hex()}")
        except socket.timeout:
            # 超时后继续循环，以便检查 running 标志
            continue
        except ConnectionResetError:
            print("[信息] 客户端异常断开。")
            break
        except Exception as e:
            print(f"[接收错误] {e}")
            break
    # 线程结束前清理
    conn.close()
    print("[接收线程] 已退出。")

# ==================== 主程序 ====================
def main():
    global running

    # 创建服务端 socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('192.168.57.100', 8800))
    server_socket.listen(1)
    print("服务端已绑定 192.168.57.100:8800，等待客户端连接...")

    conn, addr = server_socket.accept()
    print(f"接收到客户端连接：{addr}")

    # 启动接收线程
    recv_thread = threading.Thread(target=receive_thread, args=(conn,), daemon=True)
    recv_thread.start()

    # ========== 主线程负责输入和发送 ==========
    print("\n========== 交互控制台 ==========")
    print("输入文本直接发送（文本格式）")
    print("输入 'pack' 发送控制包")
    print("输入 'quit' 或 'exit' 退出程序")
    print("==================================\n")

    try:
        while True:
            user_input = input(">>> ").strip()
            if not user_input:
                continue

            if user_input.lower() in ('quit', 'exit'):
                print("退出程序。")
                break
            elif user_input.lower() == 'pack':
                input_control_packet(conn)
            else:
                # 以文本方式发送
                try:
                    conn.send(user_input.encode('utf-8'))
                    print("[文本] 已发送")
                except Exception as e:
                    print(f"[发送错误] {e}")
                    break
    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，退出。")
    finally:
        # 通知接收线程退出
        running = False
        # 等待线程结束（最多2秒）
        recv_thread.join(timeout=2)
        # 关闭连接
        conn.close()
        server_socket.close()
        print("服务端已关闭。")

if __name__ == "__main__":
    main()