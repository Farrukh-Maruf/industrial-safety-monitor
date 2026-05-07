# lidar_worker.py
# 3dlidar_gy.py의 로직을 Qt 스레드(QObject) 기반으로 재구성한 파일
# 11/7: Lidar ON/OFF 기능 추가에 따른 수정

import os
import time
import cv2
import threading
import numpy as np
import udp_client
from tcp_client import open_tcp, close_tcp, transmit_cmd
from udp_client import open_udp, close_udp, get_frame, udp_receive_loop, set_debug_mode, frame_available
from config import LIDAR_IP, TCP_PORT, UDP_PORT

from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QImage

# ==========================================================
# 3dlidar_gy.py에서 가져온 헬퍼 함수
# ==========================================================

def parse_frame(frame):
    """DataType에 따라 frameData를 numpy 배열로 변환 (좌우 반전 적용)"""
    if not frame: return None, None
    w, h = frame.width, frame.height
    if not frame.frameData or w == 0 or h == 0:
        return None, None
    mirror_idx = np.arange(w*h).reshape(h, w)
    mirror_idx = (mirror_idx // w) * w + (w - 1 - (mirror_idx % w))
    mirror_idx = mirror_idx.flatten()
    if frame.dataType == 0:  # Depth + Amplitude interleaved
        raw = np.frombuffer(frame.frameData, dtype='<u2')
        depth = raw[0::2]
        depth_mirrored = depth[mirror_idx].reshape(h, w)
        return depth_mirrored, None
    return None, None

def process_and_prepare_image(depth_data):
    """
    수신된 Depth 데이터를 받아 이상치 제거, 정규화 처리를 수행합니다.
    """
    height, width = depth_data.shape
    data_array = depth_data.flatten()
    Q1, Q3 = np.percentile(data_array, 25), np.percentile(data_array, 75)
    IQR = Q3 - Q1
    lower_bound, upper_bound = Q1 - 1.5 * IQR, Q3 + 1.5 * IQR
    data_imputed = depth_data.copy()
    
    # 이상치 처리
    for y in range(height):
        for x in range(width):
            if not (lower_bound <= data_imputed[y, x] <= upper_bound):
                neighbors = []
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        if dx == 0 and dy == 0: continue
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            neighbors.append(depth_data[ny, nx])
                if neighbors:
                    valid_neighbors = [n for n in neighbors if lower_bound <= n <= upper_bound]
                    if valid_neighbors:
                        data_imputed[y, x] = np.mean(valid_neighbors)

    # 스파이크 값 처리
    data_imputed_final = data_imputed.copy()
    threshold = 30000.0  # 30m 이상을 스파이크로 간주
    for y in range(height):
        for x in range(width):
            if data_imputed_final[y, x] > threshold:
                y_start, y_end = max(0, y - 25), min(height, y + 26)
                x_start, x_end = max(0, x - 25), min(width, x + 26)
                local_region = data_imputed[y_start:y_end, x_start:x_end]
                valid_values = local_region[local_region <= threshold]
                if valid_values.size > 0:
                    data_imputed_final[y, x] = np.mean(valid_values)

    # 8비트로 정규화
    normalized_data = ((data_imputed_final / threshold) * 255.0).astype(np.uint8)
    
    return data_imputed_final, normalized_data

# ==========================================================
# LidarWorker 클래스 정의
# ==========================================================

class LidarWorker(QObject):
    """
    백그라운드 스레드에서 Lidar 데이터 수신 및 처리를 담당하고,
    처리된 이미지를 QImage로 변환하여 메인 스레드에 시그널로 전달합니다.
    """
    frame_ready = Signal(QImage)
    frame_shared = Signal(object)
    finished = Signal()
    
    IMAGE_TYPE_DEPTH_AMP = 2
    TARGET_WIDTH, TARGET_HEIGHT = 640, 240

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_running = True
        set_debug_mode(False)
        self.udp_thread = None
        self.start_time = time.time()  # 프로그램 시작 시간 기록
        
        # 3dlidar_gy.py의 callback_data 구조를 차용
        self.callback_data = {
            'depth_frame': None, 
            'original_shape': None, 
            'click_pos': None,
            'distance': None, 
            'target_size': (self.TARGET_WIDTH, self.TARGET_HEIGHT)
        }

    def run(self):
        print(f"LiDAR 캡처 스레드 시작 (TCP {LIDAR_IP}:{TCP_PORT}, UDP :{UDP_PORT})")
        
        while self.is_running: # 자동 재연결을 위한 외부 루프
            try:
                # 1. 연결 시도
                if not open_tcp(): raise ConnectionError("TCP 연결 실패")
                if not open_udp(): raise ConnectionError("UDP 연결 실패")
                
                print("LiDAR 연결 성공. UDP 수신 스레드 시작.")
                # 2. UDP 수신 스레드 시작
                self.udp_thread = threading.Thread(target=udp_receive_loop, daemon=True)
                self.udp_thread.start()
                
                # 3. 메인 뷰어 로직 실행 (프레임 요청 및 처리)
                while self.is_running:
                    # 데이터 전송 요청
                    transmit_cmd(0x01, image_type=self.IMAGE_TYPE_DEPTH_AMP, once_or_flow=0)
                    
                    # ▼▼▼ [수정] 타임아웃을 5초 -> 1초로 줄여 반응성 향상 ▼▼▼
                    frame = get_frame(wait=True, timeout=1.0)
                    
                    # ▼▼▼ [추가] get_frame 직후 플래그 즉시 확인 ▼▼▼
                    # stop()에 의해 get_frame이 즉시 반환된 경우, 0.5초 대기 전에 루프를 탈출해야 함
                    if not self.is_running:
                        break

                    if frame:
                        depth, _ = parse_frame(frame)
                        if depth is not None and depth.size > 0:
                            
                            # [수정] 원본 Depth 데이터와 Shape을 콜백용으로 저장
                            final_processed_data, processed_image = process_and_prepare_image(depth)
                            self.callback_data['depth_frame'] = final_processed_data
                            self.callback_data['original_shape'] = depth.shape

                            equalized_image = cv2.equalizeHist(processed_image)
                            
                            resized_image = cv2.resize(
                                equalized_image, (self.TARGET_WIDTH, self.TARGET_HEIGHT), 
                                interpolation=cv2.INTER_LINEAR
                            )
                            resized_image_bgr = cv2.cvtColor(resized_image, cv2.COLOR_GRAY2BGR)
                            
                            # 1. 클릭 좌표 및 Depth 그리기
                            if self.callback_data['click_pos'] is not None and self.callback_data['distance'] is not None:
                                pos = self.callback_data['click_pos']
                                dist = self.callback_data['distance']
                                depth_m = dist / 1000.0  # mm를 m로 변환
                                
                                text = f"depth : {depth_m:.3f} m"
                                
                                cv2.circle(resized_image_bgr, pos, 3, (0, 255, 0), -1) # 녹색 원
                                cv2.putText(resized_image_bgr, text, (10, 20),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                            
                            # EnlargeDialog는 numpy 배열(BGR)을 받습니다.
                            self.frame_shared.emit(resized_image_bgr.copy())

                            h, w, ch = resized_image_bgr.shape
                            bytes_per_line = ch * w
                            
                            q_img = QImage(resized_image_bgr.data, w, h, bytes_per_line, 
                                           QImage.Format.Format_BGR888)
                            
                            self.frame_ready.emit(q_img.copy())
                    
                    # 'live' 모드와 유사하게 0.5초 대기
                    # QThread.msleep()은 QObject가 스레드로 옮겨진 후 사용 가능
                    if self.is_running:
                        time.sleep(0.5) 
                        
            except (ConnectionResetError, ConnectionError, BrokenPipeError) as e:
                if not self.is_running: break # stop()에 의해 중단된 경우
                print(f"\n🚨 LiDAR 네트워크 연결이 끊겼습니다: {e}")
                print("5초 후 자동으로 재연결을 시도합니다...")
                
            except Exception as e:
                if not self.is_running: break
                print(f"알 수 없는 오류 발생: {e}")
                
            finally:
                # 4. 모든 자원 정리
                print("LiDAR 연결을 정리합니다.")
                close_udp()
                close_tcp()
                if self.udp_thread and self.udp_thread.is_alive():
                    # 데몬 스레드라 자동 종료되지만, 명시적 정리 시도
                    pass
            
            # 'q'로 종료한 게 아니라면 5초 대기 후 루프 다시 시작
            if self.is_running:
                time.sleep(5)
                
        print("LiDAR 스레드가 종료됩니다.")
        self.finished.emit()

    def stop(self):
        """메인 윈도우에서 종료를 요청할 때 호출됩니다."""
        print("LiDAR 스트림 중지 요청 수신...")
        self.is_running = False
        
        # ▼▼▼ [추가] get_frame() 함수의 대기를 강제로 해제 ▼▼▼
        frame_available.set()
        
        # 데이터 전송 중지 명령
        try:
            transmit_cmd(0x02)
        except Exception as e:
            print(f"LiDAR 중지 명령 전송 실패: {e}")
            
        # 네트워크 연결 강제 종료 (블로킹 해제)
        close_udp()
        close_tcp()