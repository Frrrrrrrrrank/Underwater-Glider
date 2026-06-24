import socket
import struct
import sys


# ==================== 控制包发送函数 ====================
def send_control_packet(conn, pitchmotor_direction, pitchmotor_speed_rpm,
                        rollmotor_direction, rollmotor_speed_rpm,
                        thrustmotor_direction, thrustmotor_speed_rpm,
                        pump_direction, pump_on_off,
                        ask_for_status, clear_rx_ring):
    """
    发送控制数据包到客户端（通过已建立的 TCP 连接 conn）。
    参数对应结构体 EthernetUartReadoutPack 的各个字段，均为整数。
    返回 True 发送成功，False 发送失败。
    """
    pack_format = '<B I B I B I B B B B'  # 小端紧凑打包
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


# ==================== 交互式输入控制包 ====================
def input_control_packet(conn):
    """引导用户输入控制包的各字段，并发送"""
    print("\n========== 输入控制包字段 ==========")
    # 定义字段名称、类型和提示
    fields = [
        ('pitchmotor_enable', 'uint8 (0-255)'),
        ('pitchmotor_speed_rpm', 'uint32 (0-4294967295)'),
        ('rollmotor_enable', 'uint8 (0-255)'),
        ('rollmotor_speed_rpm', 'uint32 (0-4294967295)'),
        ('thrustmotor_direction', 'uint8 (0-255)'),
        ('thrustmotor_speed_rpm', 'uint32 (0-4294967295)'),
        ('pump_direction', 'uint8 (0-255)'),
        ('pump_on_off', 'uint8 (0-255)'),
        ('ask_for_status', 'uint8 (0-255)'),
        ('eth_control_flag', 'uint8 (0-255)')
    ]
    values = []
    for name, hint in fields:
        while True:
            try:
                val_str = input(f"{name} ({hint}): ").strip()
                val = int(val_str)
                # 范围检查（仅对 uint8 限 0-255，uint32 已由截断处理，但提醒）
                if 'uint8' in hint and (val < 0 or val > 255):
                    print("  输入超出 0-255 范围，请重新输入。")
                    continue
                values.append(val)
                break
            except ValueError:
                print("  请输入有效的整数。")
    # 打包发送
    if conn:
        send_control_packet(conn, *values)


# ==================== 主程序 ====================
def main():
    # 创建socket对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 允许地址重用（避免重启时端口占用）
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # 绑定IP和端口
    server_socket.bind(('192.168.57.100', 8800))
    print("服务端已绑定 192.168.57.100:8800")

    # 开始监听，允许一个客户端连接
    server_socket.listen(1)
    print("服务端已启动，等待客户端连接...")

    # 接受客户端连接
    conn, addr = server_socket.accept()
    print(f"接收到客户端连接：{addr}")

    # 持续接收和发送数据
    while True:
        # 非阻塞地检查是否有数据（这里简单使用阻塞，但可用超时）
        try:
            data = conn.recv(1024).decode('utf-8')
            if not data:
                break
            print(f"客户端：{data}")
        except Exception as e:
            print(f"接收异常：{e}")
            break

        # 获取用户要发送的内容
        user_input = input(">>> ").strip()
        if not user_input:
            continue

        if user_input.lower() == 'pack':
            # 进入控制包输入模式
            input_control_packet(conn)
        elif user_input.lower() in ('quit', 'exit'):
            print("退出程序。")
            break
        else:
            # 以文本方式发送
            try:
                conn.send(user_input.encode('utf-8'))
                print(f"[文本] 已发送")
            except Exception as e:
                print(f"发送错误：{e}")
                break

    # 关闭连接
    conn.close()
    server_socket.close()
    print("服务端已关闭。")


if __name__ == "__main__":
    main()