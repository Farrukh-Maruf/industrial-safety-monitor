# tcp_client.py
import socket, struct, select
from config import LIDAR_IP, TCP_PORT, START_MARK, END_MARK

tcp_socket = None
isConnected = False

def open_tcp():
    global tcp_socket, isConnected
    try:
        tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_socket.setblocking(True)
        tcp_socket.connect((LIDAR_IP, TCP_PORT))
        isConnected = True
        print(f"[TCP] Connected to {LIDAR_IP}:{TCP_PORT}")
        return True
    except Exception as e:
        print(f"[TCP] connect error: {e}")
        return False

def close_tcp():
    global tcp_socket, isConnected
    if isConnected:
        try:
            tcp_socket.shutdown(socket.SHUT_RDWR)
        except:
            pass
        tcp_socket.close()
        isConnected = False
        print("[TCP] Closed")

def transmit_cmd(cmd_id: int, data: bytes = b"", image_type: int = None, once_or_flow: int = None):
    if not isConnected:
        print("[CMD] TCP not connected")
        return False
    if cmd_id == 0x01:
        data = bytes([image_type, once_or_flow])
    payload_size = 1 + len(data) + 2
    packet = b"".join([
        struct.pack(">I", START_MARK),
        struct.pack(">I", payload_size),
        bytes([cmd_id]),
        data,
        bytes([0x00, 0x01]),
        struct.pack(">I", END_MARK),
    ])
    tcp_socket.sendall(packet)
    print(f"[CMD] Sent cmd=0x{cmd_id:02X}")
    return True

def receive_cmd_package():
    try:
        buf = tcp_socket.recv(1500)
        if not buf or len(buf) < 15:
            return None
        startmark = struct.unpack(">I", buf[0:4])[0]
        endmark = struct.unpack(">I", buf[-4:])[0]
        if startmark == START_MARK and endmark == END_MARK:
            payloadlen = struct.unpack(">I", buf[4:8])[0]
            if payloadlen != (len(buf) - 12):
                return None
            return {
                "cmdid": buf[8],
                "data": buf[9:-6],
                "statecode": buf[-6],
                "ptclVer": buf[-5]
            }
    except:
        pass
    return None

def tcp_receiver(stop_event):
    while not stop_event.is_set() and isConnected:
        rlist, _, _ = select.select([tcp_socket], [], [], 1.0)
        if not rlist:
            continue
        resp = receive_cmd_package()
        if resp:
            print(f"[RESP] cmd=0x{resp['cmdid']:02X}, state=0x{resp['statecode']:02X}, ver={resp['ptclVer']}")
