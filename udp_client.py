# udp_client.py
# 11/7: Lidar ON/OFF 기능 추가에 따른 수정

# (기존 import 및 클래스 정의는 동일)
import socket
import threading
import struct
import time
from config import FRAME_BUF_SIZE, UDP_HEADER_OFFSET

class Frame:
    def __init__(self):
        self.width = 0
        self.height = 0
        self.dataType = 0
        self.frame_id = 0
        self.frameData = b''

udp_socket = None

# 더블 버퍼
buffers = [None, None]
write_index = 0
read_index = 1
lock = threading.Lock()
frame_available = threading.Event()

# 조립 상태
assembling_frame = None
packet_data = {}
expected_packets = 0
received_packets = 0
expected_size = 0
last_frame_id = -1

debug_mode = False

def set_debug_mode(flag: bool):
    global debug_mode
    debug_mode = flag

def reset_assembly():
    global assembling_frame, packet_data, expected_packets, received_packets, expected_size
    assembling_frame = None
    packet_data = {}
    expected_packets = 0
    received_packets = 0
    expected_size = 0

def flush_udp_socket():
    if not udp_socket: return
    udp_socket.setblocking(False)
    while True:
        try: udp_socket.recvfrom(65535)
        except BlockingIOError: break

def open_udp(ip='', port=7687):
    global udp_socket
    try:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        udp_socket.bind((ip, port))
        udp_socket.setblocking(False)
        print(f"[UDP] Listening on :{port} (SO_RCVBUF={udp_socket.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)} bytes)")
        flush_udp_socket()
        reset_assembly()
        return True
    except Exception as e:
        print(f"[UDP] Failed to open: {e}")
        return False

def close_udp():
    global udp_socket
    if udp_socket:
        udp_socket.close()
        udp_socket = None
        print("[UDP] Closed")

def put_frame(frame):
    global write_index, read_index
    with lock:
        buffers[write_index] = frame
        write_index, read_index = read_index, write_index
    frame_available.set()

def get_frame(wait=False, timeout=5):
    start_time = time.time()
    while True:
        frame_available.wait(0.001)
        with lock:
            frame = buffers[read_index]
            if frame is not None:
                buffers[read_index] = None
                frame_available.clear()
                return frame
        if not wait or (time.time() - start_time) > timeout:
            return None

def assemble_packet(p: bytes):
    global assembling_frame, packet_data, expected_packets, received_packets, expected_size, last_frame_id

    if len(p) < UDP_HEADER_OFFSET:
        if debug_mode: print("[WARN] Packet size too small.")
        return None

    try:
        # 패킷 헤더를 빅 엔디안(>)으로 파싱
        image_sn = struct.unpack(">H", p[0:2])[0]
        total_size = struct.unpack(">I", p[2:6])[0]
        sent_size = struct.unpack(">I", p[8:12])[0]
        packet_total = struct.unpack(">I", p[12:16])[0]
        packet_number = struct.unpack(">I", p[16:20])[0]
    except struct.error as e:
        if debug_mode: print(f"[ERROR] Failed to parse UDP header: {e}")
        return None

    payload_size = len(p) - UDP_HEADER_OFFSET

    # 프레임 ID가 바뀌면 기존 프레임 조립을 실패로 간주하고 리셋
    if image_sn != last_frame_id and last_frame_id != -1:
        if debug_mode: print(f"[WARN] Frame ID changed. Resetting assembly. Old ID: {last_frame_id}, New ID: {image_sn}")
        reset_assembly()

    last_frame_id = image_sn

    # 새 프레임 시작
    if assembling_frame is None:
        if total_size > FRAME_BUF_SIZE:
            if debug_mode: print(f"[ERROR] Frame size {total_size} exceeds buffer size {FRAME_BUF_SIZE}. Dropping frame.")
            return None
        
        assembling_frame = Frame()
        assembling_frame.frame_id = image_sn
        expected_packets = packet_total
        expected_size = total_size
        
    # 이미 받은 패킷인지 확인
    if packet_number in packet_data:
        if debug_mode: print(f"[WARN] Duplicate packet received: {packet_number}")
        return None
    
    # 패킷 번호가 유효한지 확인 (0부터 packet_total-1 까지)
    if not (0 <= packet_number < expected_packets):
        if debug_mode: print(f"[WARN] Invalid packet number {packet_number}. Dropping.")
        return None

    # 데이터 저장
    packet_data[packet_number] = p
    received_packets += 1
    
    # 모든 패킷이 도착했는지 확인
    if received_packets == expected_packets:
        # 최종 프레임 데이터 조립
        full_data = bytearray(expected_size)
        
        for i in range(expected_packets):
            if i not in packet_data:
                continue
            
            payload = packet_data[i]
            # UDP 패킷 헤더에서 sent_size를 다시 읽어와서 오프셋을 정확히 맞춤
            sent_size_for_packet = struct.unpack(">I", payload[8:12])[0]
            
            # 여기서 payload는 UDP 패킷 전체(헤더 포함)
            full_data[sent_size_for_packet:sent_size_for_packet+len(payload[20:])] = payload[20:]
            
        # 프레임 메타데이터 파싱 (여기를 수정)
        try:
            # 매뉴얼 9페이지 참조 (Data format of one frame)
            # cmd: 8+0, output type: 8+1, frame number: 8+2, resolution: 8+4
            
            # 해상도 값은 빅 엔디안으로 파싱
            res = struct.unpack(">I", full_data[8+4:8+8])[0]
            width = res >> 16
            height = res & 0xFFFF
            data_type = full_data[8+1] # output type

        except Exception as e:
            if debug_mode: print(f"[ERROR] Failed to parse frame metadata: {e}")
            reset_assembly()
            return None

        assembling_frame.width = width
        assembling_frame.height = height
        assembling_frame.dataType = data_type
        
        pixel_bytes = 0
        if data_type == 1:
            pixel_bytes = width * height * 2
        elif data_type == 0:
            pixel_bytes = width * height * 4
        elif data_type == 3:
            pixel_bytes = width * height * 2
            
        assembling_frame.frameData = bytes(full_data[8+8:8+8+pixel_bytes])
        
        if debug_mode:
            print(f"[FRAME COMPLETE] id={assembling_frame.frame_id} "
                  f"{assembling_frame.width}x{assembling_frame.height} "
                  f"type={assembling_frame.dataType} pixel_bytes={pixel_bytes} "
                  f"packets={received_packets}/{expected_packets}")

        frame_out = assembling_frame
        reset_assembly()
        return frame_out

    return None

def udp_receive_loop():
    while udp_socket:
        try:
            data, addr = udp_socket.recvfrom(65535)
            frame = assemble_packet(data)
            if frame:
                put_frame(frame)
        
        # 1. [수정] non-blocking 소켓에서 데이터가 없을 때 발생하는
        #    'BlockingIOError'를 먼저, 그리고 정상적으로 처리합니다.
        except BlockingIOError:
            time.sleep(0.001) # CPU 점유를 막기 위해 잠시 대기

        # 2. [수정] 그 외의 소켓 오류 (예: stop() 호출로 소켓이 닫힌 경우)를
        #    별도로 처리합니다.
        except (OSError, socket.error, AttributeError) as e:
            # AttributeError: udp_socket이 None으로 변경된 경우 대비
            
            # lidar_worker.py의 stop() 함수가 호출되지 않았는데
            # 오류가 발생한 경우만 로그를 남깁니다. (로그가 너무 많이 쌓이는 것을 방지)
            if udp_socket is not None: 
                print(f"[UDP] 수신 루프 중단 (오류: {e}).")
            
            break # while 루프 탈출
            
    print("[UDP] 수신 스레드가 종료되었습니다.")