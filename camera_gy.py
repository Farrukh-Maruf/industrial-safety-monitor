# camera_gy.py
# 카메라 및 LiDAR 스트림 관리 모듈
# ... (기존 주석) ...
# 11/3 : Background Subtraction 옵션 추가 및 ROI 실시간 갱신 기능 구현
# 11/4 : 하드코딩된 RTSP 주소를 제거하고 DB에서 직접 읽어오도록 수정
# 11/4 : Background Subtraction 제거
# 11/5 : 프로그램 수정보완
# 11/5 : 모델 싱글톤 패턴 적용 (YOLO 모델을 CameraController에서 한 번만 로드)
# 11/6 : X_dist, Y_dist, x_ref DB 저장 및 로드 추가
# 11/7 : ROI 추가시 마우스좌표 표시 추가, Lidar 및 Camera 연결 ON/OFF 기능 추가
# 11/12 : ROI 직선 방정식 계산 함수 추가, extended ROI 그리기 기능 구현, 사람 탐지 로직 개선(ROI 중심으로)
# 11/14 : camera 레이블 프레임에 표시 기능 추가, 사람의 경우 BBox 비율 조정 기능 추가
# 11/26 : 새로운 weight값 적용(0:person, 1:truck, 2:hook)
# 11/27 : ROI 에 따른 객체 위치 인식 표시, truck/hook 반영
# 12/11 : ROI 위쪽에 있는 truck 도 map에 표시하도록 수정, substream 해상도 변경(1280x960)
# 12/12 : 영상 취득 전용 클래스(FrameGrabber) 추가, VideoWorker에서 프레임 읽기 전용으로 변경
# 12/15 : 후크(Hook) 위치 인식 로직 개선 (라인보다 30px 위까지 허용)
# 12/17 : .pt 파일대신 .onnx 파일 사용하도록 수정


import os
# os.environ["OPENCV_VIDEOIO_DEBUG"] = "1" # OpenCV 비디오 I/O 디버그 활성화
import cv2
import sqlite3  # DB 연결을 위해 추가
import ast      # DB의 문자열 좌표를 파싱하기 위해 추가
import numpy as np
import time  
import math
from PySide6.QtCore import QEvent, QObject, QRect, QSize, Qt, QThread, Signal, QPoint
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap, QBrush, QPolygon
from PySide6.QtWidgets import QLabel
from enlarge_image import EnlargeDialog

import threading
from typing import Optional, List, Tuple
import os
import torch

# [수정] PyTorch, Ultralytics 제거 -> ONNX Runtime 추가
# import torch 
# from ultralytics import YOLO 
import onnxruntime as ort


# [camera_gy.py] YOLO_ONNX 클래스 추가

class YOLO_ONNX:
    def __init__(self, model_path, use_gpu=True):
        # 0: person, 1: truck, 2: hook (사용자 모델에 맞게 설정)
        self.classes = {0: 'person', 1: 'truck', 2: 'hook'}
        
        # GPU 사용 설정 (CUDA가 있으면 CUDA, 없으면 CPU)
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if use_gpu else ['CPUExecutionProvider']
        
        print(f"[YOLO_ONNX] Loading model: {model_path} with providers: {providers}")
        try:
        
            self.session = ort.InferenceSession(model_path, providers=providers)
        except Exception as e:
            print(f"[YOLO_ONNX] 모델 로드 실패: {e}")
            self.session = None
            return

        # 모델 입력 정보 가져오기
        self.input_name = self.session.get_inputs()[0].name
        input_shape = self.session.get_inputs()[0].shape

        # ▼▼▼ [수정] 동적 축(Dynamic Axes) 처리 로직 ▼▼▼
        # input_shape[2]가 숫자가 아니라 'height' 같은 문자열일 경우 기본값 640 사용
        
        # 높이(Height) 처리
        if isinstance(input_shape[2], str):
            self.img_h = 640
            print(f"[YOLO_ONNX] Dynamic Height detected. Using default: {self.img_h}")
        else:
            self.img_h = int(input_shape[2])

        # 너비(Width) 처리
        if isinstance(input_shape[3], str):
            self.img_w = 640
            print(f"[YOLO_ONNX] Dynamic Width detected. Using default: {self.img_w}")
        else:
            self.img_w = int(input_shape[3])
            
        print(f"[YOLO_ONNX] Input Shape Set to: {self.img_w}x{self.img_h}")
        
        

    def predict(self, img, conf=0.5, iou=0.5):
        if self.session is None: return []

        # 1. 전처리 (Resize & Normalize)
        # 속도를 위해 단순 resize 사용 (Letterbox 생략)
        img_resized = cv2.resize(img, (self.img_w, self.img_h))
        
        # BGR -> RGB 변환, 채널 변경 (HWC -> CHW), 정규화 (0~1)
        blob = cv2.dnn.blobFromImage(img_resized, 1/255.0, (self.img_w, self.img_h), swapRB=True, crop=False)

        # 2. 추론 (Inference)
        outputs = self.session.run(None, {self.input_name: blob})

        # 3. 후처리 (Post-Processing)
        # YOLOv8 Output Shape: [1, 4+cls, 8400] -> [1, 84, 8400] (Transposed needed)
        predictions = np.squeeze(outputs[0]).T  # [8400, 84]

        # Confidence Filtering
        scores = np.max(predictions[:, 4:], axis=1) # 클래스 확률 중 최대값
        mask = scores > conf
        predictions = predictions[mask]
        scores = scores[mask]
        
        if len(scores) == 0: return []

        class_ids = np.argmax(predictions[:, 4:], axis=1)
        boxes = predictions[:, :4] # cx, cy, w, h

        # 좌표 복원 (입력 이미지 크기에 맞춰 스케일링)
        scale_h = img.shape[0] / self.img_h
        scale_w = img.shape[1] / self.img_w
        
        # cx,cy,w,h -> x1,y1,w,h (OpenCV NMS용 좌상단 좌표로 변환)
        boxes[:, 0] = (boxes[:, 0] - boxes[:, 2] / 2) * scale_w # x1
        boxes[:, 1] = (boxes[:, 1] - boxes[:, 3] / 2) * scale_h # y1
        boxes[:, 2] *= scale_w # w
        boxes[:, 3] *= scale_h # h

        # NMS (Non-Maximum Suppression) 실행
        indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), conf, iou)

        results = []
        for i in indices:
            idx = i # opencv 버전에 따라 i[0]일 수도 있음
            if isinstance(idx, (list, tuple, np.ndarray)):
                idx = idx[0]
            
            x, y, w, h = boxes[idx]
            # 결과 포맷: [x1, y1, x2, y2, confidence, class_id]
            results.append([int(x), int(y), int(x+w), int(y+h), scores[idx], class_ids[idx]])

        return results
    

def _get_line_equation(p1: tuple[int, int], p2: tuple[int, int]) -> tuple[int, int, int]:
    """
    두 점 (x1, y1), (x2, y2)를 지나는 직선의 방정식 (Ax + By + C = 0)의
    계수 (A, B, C)를 반환합니다.
    """
    x1, y1 = p1
    x2, y2 = p2

    A = y2 - y1
    B = x1 - x2
    C = (x2 * y1) - (x1 * y2)

    return A, B, C

def _calculate_point_at_y130(A, B, C):
    """
    주어진 직선 Ax + By + C = 0 에 대해 y=130일 때의 
    포인트 (x, 130.0) 튜플을 반환합니다.
    계수가 None이거나 A=0 (수평선)이라서 x를 구할 수 없으면 None을 반환합니다.
    """
    if A is None or B is None or C is None:
        return None
    if A == 0 or A == 0.0:
        print("[LineCalc] A=0 (수평선)이므로 y=130에서의 포인트를 계산할 수 없습니다.")
        return None
    try:
        x_val = (-130.0 * B - C) / A
        return (x_val, 130.0) 
    except Exception as e:
        print(f"[LineCalc] y=130에서 포인트 계산 중 오류 발생: {e}")
        return None
    

# YOLO 클래스 ID 상수 정의
class YOLOClasses:
    PERSON = 0
    TRUCK = 1
    HOOK = 2 

# tmap.txt에서 가져온 CustomMapLabel 클래스 (유지)
class CustomMapLabel(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.rects = []
        self.way_11 = 299
        self.way_12 = 365
        self.way_21 = 629
        self.way_22 = 695
        
        # 사람, 트럭, 훅 좌표 저장소 분리
        self.people_points_by_cam = {}
        self.truck_points_by_cam = {}  
        self.hook_points_by_cam = {}
        
        # # [추가] 각 카메라별 안전 상태를 저장할 딕셔너리 (기본값 'normal')
        self.cam_states = {}

        # [추가] 자체 계산한 경고 상태의 만료 시간을 저장 (Key: cam_name, Value: timestamp)
        self.cam_warning_until = {}
        
        # 상태 변수 추가
        self.area2_state = 'normal'  # 'normal', 'warning', 'danger'
        
        self.area_labels = ["Layout Map", "Status"]

        # [추가] 거리 표시용 선 데이터 저장소 (예: [((x1,y1), (x2,y2), "3.5m"), ...])
        self.distance_lines = []
        
        
    # [신규] 맵 상의 객체 간 거리 계산 및 상태 업데이트 함수
    def _check_collision(self, cam_name):
        people = self.people_points_by_cam.get(cam_name, [])
        hooks = self.hook_points_by_cam.get(cam_name, [])
        
        is_collision = False
        WARNING_DIST = 20.0 # 픽셀 단위 기준 거리
        WARNING_ANGLE_THRESHOLD = 25.0  # [신규] 각도 임계값
        
        # 1. 거리 계산
        for h_item in hooks:
            # 훅 데이터 파싱 & 각도 체크
            # 데이터 구조: ((lx, ly), ((cx, cy, w, h), angle))
            angle_val = 0.0
            hx, hy = 0, 0
            
            try:
                # 좌표 파싱
                if isinstance(h_item, (list, tuple)) and len(h_item) >= 1:
                    coords = h_item[0] if isinstance(h_item[0], (list, tuple)) else h_item
                    hx, hy = coords[0], coords[1]
                else: continue
                
                # 각도 파싱
                if len(h_item) == 2 and isinstance(h_item[1], (list, tuple)) and len(h_item[1]) == 2:
                    angle_val = h_item[1][1] # angle
                    
                # ▼▼▼ [수정] 각도가 25도 이하면 충돌 검사 제외 ▼▼▼
                if abs(angle_val) <= WARNING_ANGLE_THRESHOLD:
                    continue
                    
            except Exception:
                continue
            
            # 사람과의 거리 측정
            for p_item in people:
                # 사람 좌표 파싱
                px, py = 0, 0
                if isinstance(p_item, (list, tuple)) and len(p_item) >= 2:
                    px, py = p_item[0], p_item[1]
                else: continue
                
                # 거리 측정
                dist = math.sqrt((hx - px)**2 + (hy - py)**2)
                if dist <= WARNING_DIST:
                    is_collision = True
                    break
                
            if is_collision: break
            
        # 2. 상태 업데이트 (2초 홀딩 로직)
        current_time = time.time()
        if is_collision:
            # 충돌 감지 시: 현재 시간 + 2초까지 경고 유지
            self.cam_warning_until[cam_name] = current_time + 2.0
        # (충돌이 없어도 만료 시간이 안 지났으면 paintEvent에서 처리함)
            
        
    # [추가] 카메라별 상태 업데이트 함수
    def update_cam_state(self, cam_name, state):
        self.cam_states[cam_name] = state
        # (여기서는 update()를 호출하지 않아도, set_people_positions 등에서 호출되므로 생략 가능하나 안전하게 둠)
        self.update()
        
    # [추가] 거리 데이터 업데이트 함수
    def set_distance_lines(self, lines_data):
        self.distance_lines = lines_data
        self.update()

    # [추가] 트럭 위치 업데이트 함수
    def set_truck_positions_for_cam(self, cam_name: str, points: list):
        self.truck_points_by_cam[cam_name] = points
        self.update()

    # [추가] 훅 위치 업데이트 함수
    def set_hook_positions_for_cam(self, cam_name: str, points: list):
        self.hook_points_by_cam[cam_name] = points
        self._check_collision(cam_name) # <--- 검사 실행
        self.update()
        
    
    def paintEvent(self, event):
        super().paintEvent(event)
        with QPainter(self) as painter:
            pen = QPen(QColor(0, 0, 0))
            pen.setWidth(2)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            font = QFont("Arial", 10)
            painter.setFont(font)
            label_w = self.width()
            label_h = self.height()
            base_w = 1920
            base_h = 400
            
            for i, rect in enumerate(self.rects):
                x, y, w, h = rect
                scaled_x = int(label_w * (x / base_w))
                scaled_y = int(label_h * (y / base_h))
                scaled_w = int(label_w * (w / base_w))
                scaled_h = int(label_h * (h / base_h))
                
                # Area 2 상태에 따른 배경색
                if i == 1: 
                    if self.area2_state == 'warning':
                        painter.setBrush(QColor(255, 140, 0))  # 주황색
                    elif self.area2_state == 'danger':
                        painter.setBrush(QColor(255, 0, 0))  # 빨간색
                    else:
                        painter.setBrush(QColor(0, 255, 0))  # 초록색
                
                painter.drawRect(scaled_x, scaled_y, scaled_w, scaled_h)
                painter.setBrush(Qt.NoBrush) # 브러쉬 초기화

                # 폰트 저장
                original_font = painter.font()
                original_pen = painter.pen()

                if i == 0: # Layout 영역
                    text_to_draw = "Layout Map"
                    text_font = QFont("Arial", 15); text_font.setBold(True)
                    painter.setFont(text_font); painter.setPen(QColor(0, 0, 0))
                    
                    rect_layout_title = QRect(scaled_x, scaled_y - 30, 120, 30)
                    painter.drawText(rect_layout_title, Qt.AlignLeft | Qt.AlignVCenter, text_to_draw)
                    
                    # [범례 그리기]
                    legend_font = QFont("Arial", 13); legend_font.setBold(True)
                    painter.setFont(legend_font)
                    
                    area_width = 50 
                    base_text_x = scaled_x - area_width 
                    start_y = scaled_y + 10
                    row_gap = 35  
                    radius = 8
                    shape_size = int(radius * 2 * 0.88)
                    shape_center_x = base_text_x + 35
                    
                    # Row 1: Person
                    cur_y = start_y
                    painter.setPen(QColor(0, 0, 0))
                    painter.drawText(QRect(base_text_x, cur_y, 20, 20), Qt.AlignCenter, "P")
                    painter.setBrush(QBrush(QColor(0, 255, 0), Qt.SolidPattern))
                    painter.setPen(QPen(Qt.black, 2))
                    painter.drawEllipse(QPoint(shape_center_x, cur_y + 10), radius, radius)

                    # Row 2: Truck
                    cur_y += row_gap
                    painter.setFont(legend_font); painter.setPen(QColor(0, 0, 0))
                    painter.drawText(QRect(base_text_x, cur_y, 20, 20), Qt.AlignCenter, "T")
                    painter.setBrush(QBrush(QColor(0, 0, 255), Qt.SolidPattern))
                    painter.setPen(QPen(Qt.black, 2))
                    painter.drawRect(shape_center_x - (shape_size // 2), cur_y + 10 - (shape_size // 2), shape_size, shape_size)

                    # Row 3: Hook
                    cur_y += row_gap
                    painter.setFont(legend_font); painter.setPen(QColor(0, 0, 0))
                    painter.drawText(QRect(base_text_x, cur_y, 20, 20), Qt.AlignCenter, "H")
                    painter.setBrush(QBrush(QColor(255, 255, 0), Qt.SolidPattern))
                    painter.setPen(QPen(Qt.black, 2))
                    half_s = shape_size // 2
                    tri_cy = cur_y + 10
                    p1 = QPoint(shape_center_x, tri_cy - half_s)        
                    p2 = QPoint(shape_center_x - half_s, tri_cy + half_s) 
                    p3 = QPoint(shape_center_x + half_s, tri_cy + half_s) 
                    painter.drawPolygon(QPolygon([p1, p2, p3]))

                    # 복원
                    painter.setFont(original_font); painter.setPen(original_pen); painter.setBrush(Qt.NoBrush)
                    
                elif i == 1:  # Status 영역
                    status_font = QFont("Arial", 15); status_font.setBold(True)
                    painter.setFont(status_font)
                    
                    text_x = scaled_x
                    text_y = scaled_y - 30
                    text_height = 30 

                    # "Status : "
                    label_str = "Status : "
                    painter.setPen(QColor(0, 0, 0))
                    fm = painter.fontMetrics()
                    label_width = fm.horizontalAdvance(label_str)
                    painter.drawText(QRect(text_x, text_y, label_width + 10, text_height), Qt.AlignLeft | Qt.AlignVCenter, label_str)
                    
                    # Status Value
                    status_val = ""
                    if self.area2_state == 'normal':
                        status_val = "Normal"; painter.setPen(QColor(0, 180, 0))
                    elif self.area2_state == 'warning':
                        status_val = "Warning"; painter.setPen(QColor(255, 140, 0))
                    elif self.area2_state == 'danger':
                        status_val = "Danger"; painter.setPen(QColor(255, 0, 0))
                    
                    painter.drawText(QRect(text_x + label_width, text_y, 150, text_height), Qt.AlignLeft | Qt.AlignVCenter, status_val)
                    
                    painter.setFont(original_font); painter.setPen(original_pen)

            if self.rects:
                rect = self.rects[0]
                scaled_x = int(label_w * (rect[0] / base_w))
                scaled_y = int(label_h * (rect[1] / base_h))
                scaled_h = int(label_h * (rect[3] / base_h))
                
                green_pen = QPen(QColor(0, 128, 0)); green_pen.setWidth(2)
                painter.setPen(green_pen)
                
                # 라인 그리기
                for way_x in [self.way_11, self.way_12, self.way_21, self.way_22]:
                    scaled_way = int(label_w * (way_x / base_w))
                    painter.drawLine(scaled_way, scaled_y, scaled_way, scaled_y + scaled_h)
                
                # ========================================================
                # [Layer 1 : 맨 아래] 트럭(Truck) 그리기
                # ========================================================
                painter.setPen(QPen(Qt.black, 1))
                painter.setBrush(QBrush(QColor(0, 0, 255), Qt.SolidPattern)) # 파란색
                
                for cam_name, points in self.truck_points_by_cam.items():
                    for point_data in points:
                        try:
                            # 트럭 데이터 파싱 (lx, ly, w, h)
                            if len(point_data) == 4:
                                lx, ly, map_w, map_h = point_data
                            else:
                                lx, ly = point_data; map_w, map_h = 20, 20

                            sx = int(label_w * (lx / base_w))
                            sy = int(label_h * (ly / base_h))
                            
                            # ▼▼▼ [수정] Map 밖 아이콘(20,20)인 경우 강제 정사각형 처리 ▼▼▼
                            # 1920x400 비율 때문에 map_w=map_h여도 화면엔 직사각형으로 나옵니다.
                            # 이를 방지하기 위해 20x20 사이즈가 들어오면 화면상 픽셀(16px)로 고정합니다.
                            
                            if map_w == 20 and map_h == 20:
                                sw = 16  # 화면상 픽셀 크기 (정사각형 고정)
                                sh = 16
                            else:
                                sw = int(label_w * (map_w / base_w))
                                sh = int(label_h * (map_h / base_h))
                            
                            # 중심점 기준 사각형 그리기
                            painter.drawRect(sx - sw//2, sy - sh, sw, sh)
                        except: pass

                # ========================================================
                # [Layer 2 : 중간] 훅(Hook) 그리기
                # ========================================================
                painter.setPen(QPen(Qt.black, 1)) 
                hook_size = 25 
                angle_font = QFont("Arial", 10); angle_font.setBold(True)
                
                current_time = time.time()

                for cam_name, points in self.hook_points_by_cam.items():
                    
                    # 충돌 경고 시간(Warning Timer) 체크
                    until_time = self.cam_warning_until.get(cam_name, 0.0)
                    is_warning_active = current_time < until_time
                    
                    for point_data in points:
                        try:
                            angle_val = None
                            lx, ly = 0, 0
                            
                            # 훅 데이터 파싱 (복잡한 구조 대응)
                            if isinstance(point_data, (list, tuple)):
                                # Case A: ((lx, ly), ((cx, cy, w, h), angle))
                                if len(point_data) == 2 and isinstance(point_data[1], (list, tuple)) and len(point_data[1]) == 2:
                                    (lx, ly) = point_data[0] 
                                    angle_val = point_data[1][1] 
                                # Case B: (lx, ly, angle)
                                elif len(point_data) == 3:
                                    (lx, ly), _, angle_val = point_data
                                # Case C: (lx, ly) or ((lx, ly), ...)
                                elif len(point_data) == 2:
                                    if isinstance(point_data[0], (list, tuple)):
                                        (lx, ly) = point_data[0]
                                    else:
                                        (lx, ly) = point_data
                                else:
                                    continue
                            else:
                                continue

                            # ▼▼▼ [수정] 훅 색상 결정 로직 수정 ▼▼▼
                            # 안전한 훅(25도 이하)도 영향을 받아 주황색이 되는 문제 발생.
                            # 해결: 각도가 25도 이하라면 무조건 노란색(Normal)으로 고정.

                            hook_color = QColor(255, 255, 0) # 기본값: 노란색

                            if angle_val is not None:
                                if abs(angle_val) >= 30.0:
                                    # 각도가 30도 이상이면 무조건 빨간색 (위험)
                                    hook_color = QColor(255, 140, 0)
                                else:
                                    # 각도가 30도 이하면 무조건 노란색 (안전)
                                    # (설령 카메라가 Warning 상태여도 이 훅은 안전함)
                                    hook_color = QColor(255, 255, 0)
                            else:
                                # 각도 데이터가 없는 경우에만 카메라 상태 따름
                                if is_warning_active:
                                    hook_color = QColor(255, 140, 0) # Orange
                                else:
                                    hook_color = QColor(255, 255, 0)

                            painter.setBrush(QBrush(hook_color, Qt.SolidPattern))

                            sx = int(label_w * (lx / base_w))
                            sy = int(label_h * (ly / base_h))
                            
                            # 삼각형(Hook) 그리기
                            half = hook_size // 2
                            p1 = QPoint(sx, sy - half)       
                            p2 = QPoint(sx - half, sy + half) 
                            p3 = QPoint(sx + half, sy + half) 
                            
                            # 여기서 setBrush는 위에서 결정된 hook_color를 사용함
                            painter.setPen(QPen(Qt.black, 1))
                            painter.drawPolygon(QPolygon([p1, p2, p3]))
                            
                            # 각도 텍스트 표시
                            if angle_val is not None:
                                painter.setFont(angle_font)
                                
                                if abs(angle_val) >= 30.0:  
                                    painter.setPen(QColor(255, 0, 0)) # 텍스트도 빨강
                                else:
                                    painter.setPen(QColor(0, 70, 0)) # 텍스트는 진한 녹색
                                
                                painter.drawText(sx + half, sy, f"{int(angle_val)}°")
                                painter.setFont(font) 

                        except Exception as e:
                            pass

                # ========================================================
                # [Layer 3 : 맨 위] 사람(Person) 그리기 -> ★ 가장 마지막에 그짐 ★
                # ========================================================
                painter.setPen(QPen(Qt.black, 2))
                radius = 5 

                cam_colors = {
                    'lblcam1': QColor(255, 165, 0), 'lblcam2': QColor(0, 255, 0),   
                    'lblcam3': QColor(0, 255, 0), 'lblcam4': QColor(0, 255, 0),   
                    'lblcam5': QColor(255, 165, 0), 'default': QColor(128, 128, 128) 
                }
                
                for cam_name, cam_point_list in self.people_points_by_cam.items():
                    color = cam_colors.get(cam_name, cam_colors['default'])
                    painter.setBrush(QBrush(color, Qt.SolidPattern))
                    
                    for point in cam_point_list:
                        try:
                            # 사람 데이터 파싱 (lx, ly, w, h) -> (lx, ly)만 사용
                            # 데이터가 4개여도 앞의 2개만 가져와서 점을 찍음
                            if isinstance(point, (list, tuple)) and len(point) >= 2:
                                layout_x, layout_y = point[0], point[1]
                            else:
                                continue

                            scaled_x = int(label_w * (layout_x / base_w))
                            scaled_y = int(label_h * (layout_y / base_h))
                            
                            top_left_x = scaled_x - radius
                            top_left_y = scaled_y - radius
                            
                            # 원(Person) 그리기
                            painter.drawEllipse(top_left_x, top_left_y, radius * 2, radius * 2)
                        except: pass
                
                painter.setBrush(Qt.NoBrush)
                
                # 계산된 거리 선과 텍스트 그리기 (맨 위에 그리기)
                if self.distance_lines:
                    pen = QPen(QColor(255, 0, 255)) # 마젠타색 (눈에 띄게)
                    pen.setStyle(Qt.DashLine)       # 점선
                    pen.setWidth(2)
                    painter.setPen(pen)
                    painter.setFont(QFont("Arial", 12, QFont.Bold))
                    
                    for line_info in self.distance_lines:
                        # 데이터 구조: ((x1, y1), (x2, y2), text_str)
                        (lx1, ly1), (lx2, ly2), dist_text = line_info
                        
                        p1 = QPoint(int(lx1), int(ly1))
                        p2 = QPoint(int(lx2), int(ly2))
                        
                        # 1. 점선 그리기
                        painter.drawLine(p1, p2)
                        
                        # 2. 텍스트 그리기 (선의 중간 지점)
                        mid_x = (lx1 + lx2) // 2
                        mid_y = (ly1 + ly2) // 2
                        
                        # 텍스트 배경 (가독성 위해)
                        text_rect = QRect(mid_x - 30, mid_y - 15, 60, 30)
                        painter.setBrush(QColor(255, 255, 255, 200)) # 반투명 흰색
                        painter.setPen(Qt.NoPen)
                        painter.drawRect(text_rect)
                        
                        # 텍스트
                        painter.setPen(QColor(0, 0, 0))
                        painter.drawText(text_rect, Qt.AlignCenter, dist_text)
                        
                        # 다시 펜 복구 (다음 루프를 위해)
                        painter.setPen(pen)
                    
                                
    # 상태 변경 메서드 추가 
    def set_area2_state(self, state):
        """Area 2의 상태를 변경하고 위젯을 다시 그리도록 요청합니다."""
        if state in ['normal', 'warning', 'danger']:
            self.area2_state = state
            self.update() # 이 함수가 호출되면 paintEvent가 다시 실행됩니다.
            
    # ▼▼▼ [추가] 사람 위치 업데이트 메서드 ▼▼▼
    def set_people_positions_for_cam(self, cam_name: str, layout_points_list: list):
        """
        특정 카메라의 사람 위치(layout 좌표) 리스트를 업데이트하고
        위젯을 다시 그리도록 요청합니다.
        (cam_name은 'lblcam2' 형식)
        """
        self.people_points_by_cam[cam_name] = layout_points_list
        self._check_collision(cam_name) # <--- 검사 실행
        self.update()


# ▼▼▼ [신규] 영상 취득 전용 클래스 (Producer) ▼▼▼
class FrameGrabber:
    """
    RTSP 스트림에서 영상을 별도 스레드로 계속 읽어옵니다.
    항상 '가장 최신 프레임' 하나만 메모리에 유지합니다.
    """
    def __init__(self, source, name):
        self.source = source
        self.name = name
        self.cap = None
        self.lock = threading.Lock()
        self.running = False
        self.latest_frame = None
        self.status = False # 연결 상태
        self.thread = None

    def start(self):
        if self.running: return
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self._release()

    def _release(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.cap = None

    def _connect(self):
        # 재연결 시도
        if self.cap: self._release()
        
        # TCP 강제 설정 (안정성)
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
        self.cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
        
        if self.cap.isOpened():
            # 버퍼 사이즈 최소화
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 1 or 0
            print(f"[{self.name}] Grabber 연결 성공")
            return True
        return False

    def _capture_loop(self):
        """무한 루프로 프레임을 읽어 latest_frame을 갱신합니다."""
        retry_interval = 5 # 재연결 대기 시간
        
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                if not self._connect():
                    time.sleep(retry_interval)
                    continue

            ret, frame = self.cap.read()
            if not ret:
                print(f"[{self.name}] 프레임 읽기 실패. 재연결 시도...")
                self._release()
                time.sleep(1) # 너무 빠른 재시도 방지
                continue

            # 중요: 최신 프레임 덮어쓰기 (Lock 사용)
            with self.lock:
                self.latest_frame = frame
                self.status = True
            
            # 읽기 스레드는 아주 약간만 쉬어줍니다 (CPU 점유율 방지)
            time.sleep(0.005) 

    def get_latest_frame(self):
        """VideoWorker가 호출하는 함수. 현재 가장 최신 프레임을 복사해서 줍니다."""
        with self.lock:
            if self.latest_frame is not None:
                return True, self.latest_frame.copy()
            else:
                return False, None
            

class VideoWorker(QObject):
    frame_ready = Signal(QImage)
    finished = Signal()
    frame_shared = Signal(object)
    objects_detected = Signal(str, dict)

    def __init__(self, source, name, roi_coords=None, general_model=None, hook_model=None, parent=None):
        super().__init__(parent)
        self.source = source
        self.name = name
        
        # [수정] 직접 cv2.VideoCapture를 가지지 않고 FrameGrabber를 사용
        self.grabber = FrameGrabber(source, name)
        
        self._frame_lock = threading.Lock()
        self._raw_frame = None
        self._processed_frame = None
        
        self.general_model = general_model
        
        # ▼▼▼ [추가 1] FPS 계산을 위한 변수
        self.prev_time = time.time()
        self.current_fps = 0.0
        
        # ROI 관련 초기화
        self.original_roi_polygon = None
        self.extended_roi_polygon = None
        self.active_roi_polygon = None
        self.line_p1_p2 = None
        
        self.is_running = True
        self.yolo_person = False
        self.yolo_fork_person = False
        self.yolo_crane_person = False
        self.yolo_roi_display = False
        
        self.frame_count = 0
        self.skip_frames = 1  # 프레임 스킵 설정 (기본값 1)
        
        if roi_coords:
            self._set_validated_roi(roi_coords)
        
        self._setup_device()

    # [추가 2] 외부(widget_gy.py)에서 접근할 프로퍼티 추가
    @property
    def processed_frame(self):
        """스레드 안전한 processed_frame 접근"""
        with self._frame_lock:
            return self._processed_frame.copy() if self._processed_frame is not None else None
        

    def set_yolo_state(self, person, fork_person, crane_person, falldown, roi_display):
        self.yolo_person = person
        self.yolo_fork_person = fork_person
        self.yolo_crane_person = crane_person
        self.yolo_roi_display = roi_display
        
        # ▼▼▼ [추가] YOLO 상태가 변경되었으므로, 사용할 ROI를 다시 선택합니다.
        self._update_active_roi()

    @property
    def raw_frame(self):
        """스레드 안전한 raw_frame 접근"""
        with self._frame_lock:
            return self._raw_frame.copy() if self._raw_frame is not None else None

    def _set_validated_roi(self, roi_coords: List[Tuple[int, int]]) -> bool:
        """
        [수정됨] ROI 좌표를 검증하고, 원본 ROI와 확장 ROI를 모두 계산한 뒤,
        활성화된 ROI를 업데이트합니다.
        """
        try:
            if not isinstance(roi_coords, list):
                raise ValueError("ROI 좌표는 리스트여야 합니다")
            
            if len(roi_coords) != 4:
                raise ValueError(f"ROI는 정확히 4개의 점이 필요합니다 (현재: {len(roi_coords)}개)")
            
            # 1. 원본 ROI 설정
            self.original_roi_polygon = np.array(roi_coords, dtype=np.int32)
            print(f"[{self.name}] 원본 ROI 설정: {roi_coords}")
            
            # ▼▼▼ [추가] CAM2, 3, 4에 대해 P1-P2 직선 방정식 계산 ▼▼▼
            # ROI가 유효하게 설정되는 시점입니다.
            if self.name in ['lblcam2', 'lblcam3', 'lblcam4']:
                P1, P2, P3, P4 = roi_coords
                self.line_p1_p2 = _get_line_equation(P1, P2)
                print(f"[{self.name}] P1-P2 직선 방정식 설정됨: {self.line_p1_p2}")
            else:
                self.line_p1_p2 = None
            
            # 2. 확장 ROI 계산 (CAM2, CAM4인 경우에만 시도)
            if self.name in ['lblcam2', 'lblcam4']:
                P1, P2, P3, P4 = roi_coords
                
                line_p2_p3_eq = _get_line_equation(P2, P3)
                line_p1_p4_eq = _get_line_equation(P1, P4)
                
                pt_130_p2p3 = _calculate_point_at_y130(*line_p2_p3_eq)
                pt_130_p1p4 = _calculate_point_at_y130(*line_p1_p4_eq)

                if pt_130_p2p3 and pt_130_p1p4:
                    extended_points = [pt_130_p1p4, pt_130_p2p3, P3, P4]
                    # QPainter는 int로 변환해야 하지만, cv2.pointPolygonTest는 float도 괜찮습니다.
                    # np.int32로 변환하여 통일합니다.
                    self.extended_roi_polygon = np.array(extended_points, dtype=np.int32)
                    print(f"[{self.name}] 확장 ROI 계산 성공.")
                else:
                    self.extended_roi_polygon = None
                    print(f"[{self.name}] 확장 ROI 계산 실패 (수평선 또는 오류).")
            else:
                self.extended_roi_polygon = None # CAM 2, 4가 아니면 확장 ROI 없음

            # 3. 사용할 ROI 업데이트
            self._update_active_roi() # <-- 중요: 활성 ROI 업데이트
            return True
            
        except (ValueError, TypeError) as e:
            print(f"[{self.name}] ROI 설정 오류: {e}")
            self.original_roi_polygon = None
            self.line_p1_p2 = None  # 오류 시 직선도 초기화
            self.extended_roi_polygon = None
            self.active_roi_polygon = None
            return False

    def _setup_device(self):
        # ONNX Runtime이 내부적으로 처리하므로 비워둡니다.
        pass

    def set_roi(self, roi_coords_str: str):
        """실시간 ROI 업데이트 (검증 포함)"""
        if not roi_coords_str or not roi_coords_str.strip():
            # ▼▼▼ [수정] 모든 ROI 초기화 ▼▼▼
            self.original_roi_polygon = None
            self.extended_roi_polygon = None
            self.active_roi_polygon = None
            print(f"[{self.name}] 모든 ROI가 초기화되었습니다")
            return

        try:
            coords_list = ast.literal_eval(roi_coords_str)
            self._set_validated_roi(coords_list)
            
        except (ValueError, SyntaxError) as e:
            print(f"[{self.name}] ROI 파싱 오류: {e}")
            self.original_roi_polygon = None
            self.extended_roi_polygon = None
            self.active_roi_polygon = None
            
    # ▼▼▼ [신규] 활성 ROI를 동적으로 선택하는 함수 ▼▼▼
    def _update_active_roi(self):
        """
        현재 yolo 상태와 카메라 이름을 기반으로
        탐지/표시에 사용할 self.active_roi_polygon을 설정합니다.
        """
        # 1. 트럭 탐지 옵션이 켜져있고,
        # 2. CAM2 또는 CAM4이며,
        # 3. 계산된 확장 ROI가 존재할 때
        if (self.yolo_fork_person and 
            self.name in ['lblcam2', 'lblcam4'] and 
            self.extended_roi_polygon is not None):
            
            # -> 확장 ROI를 활성 ROI로 사용
            self.active_roi_polygon = self.extended_roi_polygon
            print(f"[{self.name}] Active ROI set to: EXTENDED")
        else:
            # -> 그 외 모든 경우엔 원본 ROI를 사용
            self.active_roi_polygon = self.original_roi_polygon
            if self.name in ['lblcam2', 'lblcam4']:
                 print(f"[{self.name}] Active ROI set to: ORIGINAL (ForkPerson OFF or Ext.ROI missing)")
            else:
                 print(f"[{self.name}] Active ROI set to: ORIGINAL")    
                 

    def _get_roi_position_status(self, bbox: Tuple[int, int, int, int], target_polygon: np.ndarray) -> str:
        """
        [수정됨] 객체 *바닥면 중심점*의 ROI 내/외부 및 상대 위치를 반환합니다.
        (기존: BBox 중심점 -> 변경: BBox 밑면 중심)
        [개선됨] 'outside' 판단 시, 단순 BoundingRect 대신 Centroid(무게중심)를
                 이용하여 좌/우 판단 정확도 향상
        """
        # 1. 활성화된 ROI가 없으면 "none" (무조건 통과)
        if target_polygon is None:
            return "none"

        # 2. 객체 바닥면 중심점 계산
        x1, y1, x2, y2 = bbox
        comparison_x = (x1 + x2) / 2
        comparison_y = float(y2)  # y2 (바닥면 Y좌표)

        # 3. (정확한) 폴리곤 내부인지 검사
        distance = cv2.pointPolygonTest(
            target_polygon,
            (float(comparison_x), comparison_y),
            False
        )
        
        if distance >= 0:
            return "inside"

        # 4. [핵심 수정] 'outside' 상태 판단 (Centroid 기반)
        try:
            # (A) BoundingRect는 '상/하' 판단을 위해서만 사용
            roi_x, roi_y, roi_w, roi_h = cv2.boundingRect(target_polygon)
            roi_bottom = roi_y + roi_h
            
            # 1순위: 상/하 판단 (신뢰도 높음)
            if comparison_y < roi_y:
                return "outside_above"
            if comparison_y > roi_bottom:
                return "outside_below"

            # (B) 상/하가 아니라면, Centroid(무게중심)로 '좌/우' 판단
            M = cv2.moments(target_polygon)
            if M["m00"] == 0: # 0으로 나누기 방지
                return "outside_complex" # (OUT)
                 
            # 폴리곤의 무게중심 X좌표 계산
            centroid_x = int(M["m10"] / M["m00"])
            
            # 2순위: 좌/우 판단
            if comparison_x < centroid_x:
                return "outside_left"
            else:
                # (comparison_x >= centroid_x)
                return "outside_right"

        except Exception as e:
            print(f"[{self.name}] ROI 바운딩/무게중심 계산 오류: {e}")
            return "outside_complex" # (OUT)    
    
    
    def _set_validated_roi(self, roi_coords: List[Tuple[int, int]]) -> bool:
        # (기존 코드와 동일)
        try:
            if not isinstance(roi_coords, list): raise ValueError("ROI 좌표는 리스트여야 합니다")
            if len(roi_coords) != 4: raise ValueError(f"ROI는 정확히 4개의 점이 필요합니다")
            
            self.original_roi_polygon = np.array(roi_coords, dtype=np.int32)
            if self.name in ['lblcam2', 'lblcam3', 'lblcam4']:
                P1, P2, P3, P4 = roi_coords
                self.line_p1_p2 = _get_line_equation(P1, P2)
            else: self.line_p1_p2 = None
            
            if self.name in ['lblcam2', 'lblcam4']:
                P1, P2, P3, P4 = roi_coords
                line_p2_p3_eq = _get_line_equation(P2, P3)
                line_p1_p4_eq = _get_line_equation(P1, P4)
                pt_130_p2p3 = _calculate_point_at_y130(*line_p2_p3_eq)
                pt_130_p1p4 = _calculate_point_at_y130(*line_p1_p4_eq)
                if pt_130_p2p3 and pt_130_p1p4:
                    extended_points = [pt_130_p1p4, pt_130_p2p3, P3, P4]
                    self.extended_roi_polygon = np.array(extended_points, dtype=np.int32)
                else: self.extended_roi_polygon = None
            else: self.extended_roi_polygon = None
            self._update_active_roi()
            return True
        except Exception as e:
            print(f"[{self.name}] ROI 설정 오류: {e}")
            return False

    def _setup_device(self):
        try:
            if torch.cuda.is_available(): self.device = "cuda:0"
            else: self.device = "cpu"
        except Exception: self.device = "cpu"

    def set_roi(self, roi_coords_str: str):
        if not roi_coords_str or not roi_coords_str.strip():
            self.original_roi_polygon = None
            self.extended_roi_polygon = None
            self.active_roi_polygon = None
            return
        try:
            coords_list = ast.literal_eval(roi_coords_str)
            self._set_validated_roi(coords_list)
        except Exception: pass

    def _update_active_roi(self):
        if (self.yolo_fork_person and self.name in ['lblcam2', 'lblcam4'] and self.extended_roi_polygon is not None):
            self.active_roi_polygon = self.extended_roi_polygon
        else:
            self.active_roi_polygon = self.original_roi_polygon
            
    
    # ▼▼▼ [핵심 수정] run 메서드 변경 ▼▼▼
    def run(self):
        """메인 실행 루프"""
        # 1. 영상 취득 스레드 시작
        self.grabber.start()
        
        # Grabber가 초기 연결될 때까지 잠시 대기
        wait_count = 0
        while not self.grabber.status and wait_count < 20:
            time.sleep(0.5)
            wait_count += 1
        if not self.grabber.status:
            print(f"[{self.name}] 초기 연결 실패 (Grabber)")
            # 실패해도 일단 루프는 돌게 하거나(재시도), 종료할 수 있습니다.
            # 여기서는 계속 시도하도록 둡니다.

        try:
            while self.is_running:
                # 2. Grabber에서 최신 프레임 가져오기 (비동기)
                ret, frame = self.grabber.get_latest_frame()
                
                if not ret:
                    # 프레임이 아직 없으면 아주 잠깐 대기
                    QThread.msleep(10) 
                    continue
                
                # ▼▼▼ [추가] 해상도 리사이즈 (1280x960 -> 1280x720) ▼▼▼
                # cv2.INTER_AREA는 축소 시 화질 보존에 유리합니다.
                if frame is not None:
                    frame = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)
                
                # ▼▼▼ [추가 2] FPS 계산 (모든 카메라가 계산은 하되, 그리기는 아래에서 처리)
                current_time = time.time()
                time_diff = current_time - self.prev_time
                if time_diff > 0:
                    fps = 1.0 / time_diff
                    # 값이 너무 튀지 않게 부드럽게(Exponential Moving Average) 적용
                    self.current_fps = (self.current_fps * 0.9) + (fps * 0.1)
                self.prev_time = current_time

                # 3. 프레임 처리 (기존 로직)
                # 원본 저장
                with self._frame_lock:
                    self._raw_frame = frame.copy()

                # YOLO 및 그리기
                frame = self._apply_yolo_detection(frame)
                
                # ROI 그리기
                if self.yolo_roi_display and self.original_roi_polygon is not None:
                    try:
                        pts = self.original_roi_polygon.reshape((-1, 1, 2))
                        cv2.polylines(frame, [pts], True, (211, 211, 211), 3)
                    except Exception: pass
                
                # 라벨 그리기
                try:
                    label_text = self.name.replace('lbl', '').upper()
                    cv2.putText(frame, label_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                1, (255, 255, 0), 2, cv2.LINE_AA)
                    
                    # ▼▼▼ [수정 3] CAM1인 경우에만 옆에 FPS 표시
                    if self.name == 'lblcam1':
                        fps_text = f"FPS: {self.current_fps:.1f}"
                        # 이름 옆(x: 130)에 노란색으로 표시
                        cv2.putText(frame, fps_text, (130, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, (0, 255, 255), 2, cv2.LINE_AA)
                        
                except Exception: pass

                # 처리된 프레임 저장
                with self._frame_lock:
                    self._processed_frame = frame.copy()
                
                # UI 전송
                self.frame_shared.emit(frame.copy())
                self._emit_qimage(frame)
                
                # 4. 루프 속도 조절
                # YOLO 처리가 빠르면 프레임이 너무 빨리 돌 수 있으므로 최소한의 텀을 둡니다.
                QThread.msleep(10)

        finally:
            self._cleanup()
            

    def _apply_yolo_detection(self, frame: np.ndarray) -> np.ndarray:
        # (기존 코드 유지)
        if self.yolo_person or self.yolo_fork_person or self.yolo_crane_person:
            frame = self._detect_general_objects(frame)
        return frame


    # [camera_gy.py] VideoWorker 클래스 내부 메서드로 추가/교체

    def _solve_duplicate_hooks(self, detections, iomin_thresh=0.5):
        """
        [강력한 중복 제거]
        1. 면적이 큰 순서대로 정렬합니다.
        2. 작은 박스가 큰 박스와 50% 이상(iomin_thresh) 겹치면 무조건 제거합니다.
           (Intersection over Minimum Area 방식)
        """
        if len(detections) < 2:
            return detections

        # 1. 면적 기준 내림차순 정렬 (큰 놈이 먼저 오게)
        # box: [x1, y1, x2, y2, conf, cls_id]
        sorted_dets = sorted(detections, key=lambda x: (x[2] - x[0]) * (x[3] - x[1]), reverse=True)
        
        keep = []
        
        while len(sorted_dets) > 0:
            # 현재 가장 큰 박스를 꺼냄 (살릴 후보)
            large_box = sorted_dets.pop(0) 
            keep.append(large_box)
            
            lx1, ly1, lx2, ly2 = large_box[:4]
            large_area = (lx2 - lx1) * (ly2 - ly1)
            
            # 남은 박스들(더 작은 박스들)과 비교하여 중복 제거
            remains = []
            for small_box in sorted_dets:
                sx1, sy1, sx2, sy2 = small_box[:4]
                small_area = (sx2 - sx1) * (sy2 - sy1)
                
                # 교집합(Intersection) 계산
                ix1 = max(lx1, sx1)
                iy1 = max(ly1, sy1)
                ix2 = min(lx2, sx2)
                iy2 = min(ly2, sy2)
                
                inter_w = max(0, ix2 - ix1)
                inter_h = max(0, iy2 - iy1)
                inter_area = inter_w * inter_h
                
                # [핵심 로직] IoMin (Intersection over Minimum Area)
                # 작은 박스 면적 대비 겹치는 비율 계산
                if small_area > 0:
                    overlap_ratio = inter_area / small_area
                else:
                    overlap_ratio = 0
                
                # 겹치는 비율이 50%(0.5)를 넘으면 "큰 박스의 일부"로 보고 제거
                # 또한, 중심점이 매우 가까우면(가로 50px 이내) 모양이 달라도 중복으로 간주
                large_cx = (lx1 + lx2) / 2
                small_cx = (sx1 + sx2) / 2
                center_dist_x = abs(large_cx - small_cx)

                if overlap_ratio > iomin_thresh or center_dist_x < 50:
                    continue # 제거 (remains에 담지 않음)
                else:
                    remains.append(small_box) # 살림
            
            # 살아남은 박스들로 리스트 갱신
            sorted_dets = remains
            
        return keep
    
    
    def _check_vertical_chain(self, frame, bbox):
        """
        Sobel 필터를 이용해 '수직 성분(체인)'이 '수평 성분(배경 철판)'보다
        우세한지 검사합니다.
        """
        x1, y1, x2, y2 = bbox
        w = x2 - x1
        h = y2 - y1
        
        if w < 10 or h < 10: return False

        # ---------------------------------------------------------
        # 1. ROI 설정: "정중앙 좁은 영역(Center Strip)"만 봅니다.
        # ---------------------------------------------------------
        # 체인은 항상 후크 중심에서 아래로 떨어집니다.
        # 좌우의 배경(철판)을 배제하기 위해 너비의 가운데 30%만 잘라냅니다.
        center_x_start = x1 + int(w * 0.35)
        center_x_end = x1 + int(w * 0.65)
        
        # 높이는 후크 몸통 바로 아래부터 바닥까지 (상단 20% 제외)
        roi_y_start = y1 + int(h * 0.2)
        
        # 이미지 범위 체크
        img_h, img_w = frame.shape[:2]
        center_x_start = max(0, center_x_start)
        center_x_end = min(img_w, center_x_end)
        roi_y_start = max(0, roi_y_start)
        y2 = min(img_h, y2)
        
        roi = frame[roi_y_start:y2, center_x_start:center_x_end]
        if roi.size == 0: return False

        # ---------------------------------------------------------
        # 2. Sobel 필터로 수직 vs 수평 성분 분리
        # ---------------------------------------------------------
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Sobel X: 수직선 검출 (x방향 변화량 -> 세로 엣지)
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        # Sobel Y: 수평선 검출 (y방향 변화량 -> 가로 엣지)
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        
        # 절댓값 취하기 (강도 계산)
        abs_sobel_x = cv2.convertScaleAbs(sobel_x)
        abs_sobel_y = cv2.convertScaleAbs(sobel_y)
        
        # ---------------------------------------------------------
        # 3. 점수 계산 (Vertical Score vs Horizontal Score)
        # ---------------------------------------------------------
        # 전체 엣지 강도의 합
        total_vertical = np.sum(abs_sobel_x)
        total_horizontal = np.sum(abs_sobel_y)
        
        # 분모가 0일 경우 방지
        if total_horizontal == 0: total_horizontal = 1
        
        # [비율 판별]
        # 수직 성분이 수평 성분보다 1.5배 이상 강하면 체인으로 간주
        # (배경 철판이 많으면 horizontal 값이 높아져서 ratio가 낮게 나옵니다 -> 필터링됨)
        ratio = total_vertical / total_horizontal
        
        # 디버깅용 (나중에 주석 처리)
        # print(f"Vert: {total_vertical}, Horiz: {total_horizontal}, Ratio: {ratio:.2f}")
        
        # 임계값 튜닝 포인트: 
        # 1.2 ~ 1.5 정도가 적당합니다. (세로선이 가로선보다 20%~50% 더 많아야 함)
        return ratio > 1.3
    

    def _detect_general_objects(self, frame: np.ndarray) -> np.ndarray:
        try:
            # ▼▼▼ [추가된 방어 코드] 모델이 없으면 탐지 건너뛰기 ▼▼▼
            if self.general_model is None:
                # 로그가 너무 많이 뜨지 않게 100프레임마다 한 번씩만 출력하거나, 최초 1회만 출력 추천
                if self.frame_count % 100 == 0:
                    print(f"[{self.name}] 경고: 모델이 로드되지 않아 탐지를 수행할 수 없습니다.")
                return frame
        
            self.frame_count += 1
            if self.frame_count % self.skip_frames == 0:
                
                # ▼▼▼ ONNX 모델 추론 실행 ▼▼▼
                # 반환값 형식: [[x1, y1, x2, y2, conf, class_id], ...]
                results = self.general_model.predict(frame, conf=0.5, iou=0.5)
                
                # ▼▼▼ [수정 시작] Hook에 대해서만 중첩(포함) 필터링 적용 ▼▼▼
                final_results = []
                hooks = []
                others = []

                # 1. Hook과 나머지 분리
                for det in results:
                    if int(det[5]) == YOLOClasses.HOOK:
                        hooks.append(det)
                    else:
                        others.append(det)
                
                # ▼▼▼ [수정] 강력한 중복 제거 함수 호출 (임계값 0.5) ▼▼▼
                filtered_hooks = self._solve_duplicate_hooks(hooks, iomin_thresh=0.5)
                
                # 3. 다시 합치기
                final_results = others + filtered_hooks
                
                det_person = []
                det_truck = []
                det_hook = []
                
                # ▼▼▼ [변경] 결과 파싱 로직 수정 ▼▼▼
                for det in results:
                    x1, y1, x2, y2, conf, cls_id = det
                    x1, y1, x2, y2, cls_id = int(x1), int(y1), int(x2), int(y2), int(cls_id)
                    
                    w = x2 - x1
                    h = y2 - y1
                    
                    # --- 아래부터는 기존 로직과 동일 (변수명만 cls -> cls_id로 주의) ---
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2 
                    lower_y = float(y2)
                    
                    # 2. 유효 위치 판별 로직 (ROI/Line 체크)
                    is_valid_loc = False
                    
                    if self.line_p1_p2 is not None:
                        A, B, C = self.line_p1_p2
                        if B != 0:
                            line_y_at_x = (-C - (A * center_x)) / B
                            if cls_id == YOLOClasses.HOOK: # HOOK 체크
                                THRESHOLD_Y = line_y_at_x - 30.0
                            else:
                                THRESHOLD_Y = line_y_at_x 
                                
                            if lower_y > THRESHOLD_Y:
                                is_valid_loc = True
                                
                    else:
                        if self.original_roi_polygon is not None:
                            dist = cv2.pointPolygonTest(self.original_roi_polygon, (center_x, lower_y), False)
                            if dist >= 0:
                                is_valid_loc = True
                    
                    if self.name in ['lblcam1', 'lblcam5']:
                        if cls_id in [YOLOClasses.TRUCK, YOLOClasses.PERSON, YOLOClasses.HOOK]:
                            is_valid_loc = True
                        
                    # A. 사람 (Person)
                    if self.yolo_person and cls_id == YOLOClasses.PERSON:
                        color = (0, 255, 0)
                        label_text = f"P {conf:.2f}"
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame, label_text, (x1, y1 - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        if is_valid_loc: det_person.append((center_x, lower_y, w, h))

                    # B. 트럭 (Truck)
                    elif self.yolo_fork_person and cls_id == YOLOClasses.TRUCK:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        cv2.putText(frame, f"T {conf:.2f}", (x1, y1 - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        if is_valid_loc or (self.name in ['lblcam2', 'lblcam4']):
                            det_truck.append((center_x, lower_y, w, h))

                    # C. 훅 (Hook)
                    elif self.yolo_crane_person and cls_id == YOLOClasses.HOOK:
                        yellow = (0, 255, 255)
                        color_to_draw = yellow 
        
                        if is_valid_loc: 
                            # 각도 계산
                            if h > 0: aspect_angle = np.degrees(2.0 * np.arctan((w / 2.0) / h))
                            else: aspect_angle = 0.0
                            
                            # ▼▼▼ [수정] 엣지 밀도를 이용한 "가짜 회전" 필터링 ▼▼▼
                            is_real_load = False
                            
                            # 1. 각도가 30도 이상이어야 'Warning' 후보가 됨
                            if aspect_angle >= 30.0:
                                # [수정] Sobel 필터로 세로선(체인) 유무 확인
                                # 배경(철판)이 많아도 가로선이므로 ratio가 낮게 나와서 False가 됩니다.
                                is_real_load = self._check_vertical_chain(frame, (x1, y1, x2, y2))
                                
                                if is_real_load: 
                                    # 진짜 체인이 감지됨 -> Warning 유지
                                    pass 
                                else:
                                    # 각도는 크지만 세로선(체인)이 부족함 -> 가짜(회전한 빈 후크)
                                    aspect_angle = 29.0 # 경고 해제
                            
                            # 결과 리스트에 추가
                            # det_hook 구조: ((좌표), 각도, 진짜짐여부) - 필요하다면 구조 확장
                            # 여기서는 편의상 angle 값을 조작하거나 플래그를 활용할 수 있습니다.
                            
                            if is_real_load:
                                # 진짜 짐이 있을 때만 각도 값을 유지 (Warning 발생)
                                final_angle = aspect_angle
                                # 시각적으로도 붉은색 표시 (선택사항)
                                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2) # Red Box
                                cv2.putText(frame, f"LOAD {aspect_angle:.0f}", (x1, y1 - 10), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                            else:
                                # 짐이 없으면 각도를 0으로 보내버리거나, 
                                # Warning 조건(30도) 미만인 29도로 낮춰서 Warning을 회피
                                final_angle = 29.0 
                                
                                # 빈 후크 시각화 (Yellow)
                                cv2.rectangle(frame, (x1, y1), (x2, y2), yellow, 2)
                                cv2.putText(frame, f"{conf:.2f}", (x1, y1 - 10), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, yellow, 2)

                            det_hook.append(((center_x, lower_y, w, h), final_angle))

                detection_data = {'person': det_person, 'truck': det_truck, 'hook': det_hook}
                with self._frame_lock: self._processed_frame = frame.copy()
                self.objects_detected.emit(self.name, detection_data)

        except Exception as e:
            print(f"[{self.name}] 탐지 로직 오류: {e}")
        
        return frame
    

    def _emit_qimage(self, frame: np.ndarray):
        # (기존 코드 유지)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame_rgb.shape
        bytes_per_line = ch * w
        q_img = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
        self.frame_ready.emit(q_img)

    def _cleanup(self):
        """리소스 정리"""
        # Grabber 정지
        if self.grabber:
            self.grabber.stop()
        self.finished.emit()
        print(f"[{self.name}] VideoWorker 정리 완료")

    def stop(self):
        self.is_running = False
        # Grabber도 함께 정지하도록 유도
        if self.grabber:
            self.grabber.stop()


# 카메라와 LiDAR 관리 클래스
class CameraController(QObject):
    
    # ▼▼▼ [추가] 워커로부터 받은 '사람 탐지' 시그널을 릴레이 ▼▼▼
    objects_detected_in_worker = Signal(str, dict)
    
    def __init__(self, main_window):
        super().__init__(main_window)
        self.main_window = main_window
        self.threads = []
        self.workers = []
        self.labels = {}
        self._is_setup = False
        self.enlarge_dialogs = {}
        self.LIDAR_SOURCE_ID = "LIDAR"
        
        # ▼▼▼ [수정 3] 싱글톤: 모델을 CameraController 생성 시 한 번만 로드합니다.
        self.general_model = None
        self.hook_model = None
        self.load_yolo_models() # 모델 로딩 함수 호출
        
        # ▼▼▼ [수정] DB에서 ROI와 RTSP 주소를 불러와 저장할 변수 초기화 ▼▼▼
        self.camera_rois = {}
        self.camera_sources = {} # RTSP 주소 저장용 딕셔너리 추가
        self._load_camera_data_from_db() # 기존 _load_roi_data_from_db 호출 변경

        # ▼▼▼ [수정] 스트림 안정성을 위해 UDP 대신 TCP를 강제로 사용하도록 설정 ▼▼▼
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
        # os.environ["OPENCV_FFMPEG_INTERRUPT_CALLBACK_TIMEOUT"] = "60000"
        
    
    def load_yolo_models(self):
        print("[CameraController] YOLO(ONNX) 모델 로딩 시작...")
        try:
            # 1. 절대 경로 설정 (안전성 확보)
            base_dir = os.path.dirname(os.path.abspath(__file__))
            general_model_path = os.path.join(base_dir, "weights", "gy_best.onnx")  # gy_best
            
            if os.path.exists(general_model_path):
                # 2. 모델 로드 (우선 안정적인 CPU 모드로 설정)
                # 추후 GPU 설정이 완벽해지면 use_gpu=True로 변경 가능
                self.general_model = YOLO_ONNX(general_model_path, use_gpu=True)
                print(f"[CameraController] ONNX Model 로드 완료: {general_model_path}")
            else:
                print(f"[CameraController] 경고: 모델 파일 없음: {general_model_path}")
                self.general_model = None
                
        except Exception as e:
            print(f"[CameraController] 모델 로드 오류: {e}")
            self.general_model = None
        
        
    # VideoWorker를 생성하는 함수
    def create_video_worker(self, source, label_name):
        # ... (로직 생략 - 기존 로직 유지) ...
        # worker_name = label_name

        # ▼▼▼ [수정 4] worker 생성 시 로드된 모델 객체를 인자로 전달
        # model_path 대신 self.general_model과 self.hook_model을 전달
        worker = VideoWorker(
            source=source, 
            name=label_name, 
            general_model=self.general_model, # 모델 객체 전달
            hook_model=self.hook_model,       # 모델 객체 전달
            main_window=self.main_window
        )
                
     # 이 함수는 GUI를 통해 설정된 ROI 좌표를 라벨에 바로 적용하는 헬퍼 함수입니다.
    def set_label_roi_coords(self, label_name, roi_coords_str):
        if label_name in self.labels and 'label_widget' in self.labels[label_name]:
            self.labels[label_name]['label_widget'].set_roi_coords(roi_coords_str)

    # 이 함수는 GUI를 통해 설정된 ROI 표시 상태를 라벨에 바로 적용하는 헬퍼 함수입니다.
    def set_label_roi_display(self, label_name, display_state):
        if label_name in self.labels and 'label_widget' in self.labels[label_name]:
            self.labels[label_name]['label_widget'].set_roi_display_state(display_state)                

    # ▼▼▼ [수정] DB에서 RTSP 주소와 ROI 데이터를 모두 로드하는 메서드로 변경 ▼▼▼
    def _load_camera_data_from_db(self):
        """setting_gy.db에서 카메라 주소(ai_ipadd)와 ROI 좌표를 불러옵니다."""
        try:
            # (중요) DB 파일 경로가 camera_gy.py와 동일한 위치에 있다고 가정
            conn = sqlite3.connect('setting_gy.db') 
            cursor = conn.cursor()
            
            # Setting 테이블에서 cam_name, ai_ipadd, cam_roi_coords를 조회 
            cursor.execute("SELECT cam_name, ai_ipadd, cam_roi_coords FROM Setting")
            rows = cursor.fetchall()
            
            for row in rows:
                cam_name, ip_address, roi_coords_str = row

                if not cam_name:
                    continue

                # 1. 카메라 소스(RTSP 주소) 저장
                if ip_address:
                    self.camera_sources[cam_name] = ip_address
                else:
                    print(f"[DB_Loader] {cam_name}에 IP 주소가 없습니다.")

                # 2. ROI 좌표 저장
                if roi_coords_str: # 좌표 문자열이 비어있지 않은 경우
                    try:
                        # ast.literal_eval을 사용해 문자열을 
                        # '[(x,y), ...]' 형태의 리스트로 안전하게 변환
                        roi_coords_list = ast.literal_eval(roi_coords_str)
                        # DB의 키(CAM1, CAM2 등)를 그대로 사용
                        self.camera_rois[cam_name] = roi_coords_list
                    except Exception as parse_error:
                        print(f"[DB_Loader] {cam_name} ROI 파싱 오류: {parse_error}")
            
            print(f"[DB_Loader] DB에서 {len(self.camera_sources)}개의 카메라 소스와 {len(self.camera_rois)}개의 ROI 로드 완료.")
            conn.close()
            
        except Exception as e:
            print(f"[DB_Loader] setting_gy.db 로드 실패: {e}")

    def setup_cameras(self):
        if self._is_setup:
            print("Cameras are already set up.")
            return

        labels_map = {
            "lblcam2": self.main_window.ui.lblcam2,
            "lblcam3": self.main_window.ui.lblcam3,
            "lblcam4": self.main_window.ui.lblcam4,
            "lblcam1": self.main_window.ui.lblcam1,
            "lblcam5": self.main_window.ui.lblcam5,
            "lbllidar": self.main_window.ui.lbllidar,
        }

        # ▼▼▼ [수정] PREDEFINED_PATHS 대신 DB에서 로드한 self.camera_sources 사용 ▼▼▼
        for cam_name, source in self.camera_sources.items():
            # DB 키 'CAM1'을 라벨 맵 키 'lblcam1'로 변환
            label_name = f"lbl{cam_name.lower()}"
            
            if label_name in labels_map:
                label = labels_map[label_name]
                # create_and_start_thread 호출 시 라벨 이름(lblcam1)을 전달
                self.create_and_start_thread(label_name, source, label)
            else:
                print(f"DB의 {cam_name}에 해당하는 UI 라벨({label_name})을 찾을 수 없습니다.")
        # ▲▲▲ [수정] 완료 ▲▲▲

        self._is_setup = True

    def create_and_start_thread(self, name, source, label):
        # ▼▼▼ [수정 10] Worker 생성 시 DB에서 로드한 ROI 좌표 전달 ▼▼▼
        
        # 1. 내부 이름(lblcam1)을 DB 키(CAM1)로 변환
        #    (UI 탭 이름과 DB 키가 일치한다고 가정)
        db_cam_name = name.replace('lbl', '').upper() 
        
        # 2. self.camera_rois에서 해당 카메라의 ROI 좌표를 가져옴 (없으면 None)
        #    (self.camera_rois는 _load_camera_data_from_db에서 미리 로드됨)
        roi_coords = self.camera_rois.get(db_cam_name)
        
        # 3. VideoWorker 생성 시 'name'과 'roi_coords' 전달
        worker = VideoWorker(
            source, 
            name, 
            roi_coords=roi_coords,
            # ▼▼▼ [수정 3] CameraController가 로드한 모델 객체를 인자로 전달합니다.
            general_model=self.general_model,
            hook_model=self.hook_model
        )
        
        thread = QThread()
        worker.moveToThread(thread)

        label.setScaledContents(True)
        label.setCursor(Qt.PointingHandCursor)

        label.installEventFilter(self)

        thread.started.connect(worker.run)
        worker.frame_ready.connect(lambda img: self.update_label_image(label, img))
        worker.finished.connect(thread.quit)
        
        # ▼▼▼ [추가] 워커의 시그널을 컨트롤러의 릴레이 시그널로 연결 ▼▼▼
        worker.objects_detected.connect(self.objects_detected_in_worker)

        # 스레드 정리를 위한 추가 연결
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)

        thread.start()

        self.threads.append(thread)
        self.workers.append(worker)
        self.labels[name] = {"label": label, "source": source, "thread": thread, "worker": worker}

    def update_label_image(self, label, q_img):
        label.setPixmap(QPixmap.fromImage(q_img))
        # self.last_frames[label] = q_img

    def eventFilter(self, watched, event):
        if event.type() == QEvent.MouseButtonPress:
            if event.button() == Qt.LeftButton:
                for name, info in self.labels.items():
                    if info["label"] == watched:
                        self.show_enlarged_image(info["source"])
                        return True
        return super().eventFilter(watched, event)

    def find_worker_by_source(self, source):
        """소스에 해당하는 기존 VideoWorker 찾기"""
        for name, info in self.labels.items():
            if info["source"] == source:
                return info["worker"]
        return None

    def show_enlarged_image(self, source):
        if source in self.enlarge_dialogs and self.enlarge_dialogs[source].isVisible():
            self.enlarge_dialogs[source].raise_()
            self.enlarge_dialogs[source].activateWindow()
            return

        # 2. 새로운 확대 창을 열기 전, 현재 열려있는 '다른' 확대 창이 있다면 모두 닫습니다.
        #    딕셔너리의 복사본을 순회하여 반복 중 아이템이 삭제되어도 오류가 발생하지 않도록 합니다.
        for dialog in list(self.enlarge_dialogs.values()):
            dialog.close()

        # 3. 기존 worker를 찾아서 새 확대 창을 생성하는 부분은 그대로 유지됩니다.
        existing_worker = self.find_worker_by_source(source)
        print(f"[CameraController] Found worker for source {source}: {existing_worker is not None}")

        # 기존 worker를 전달하여 프레임 공유
        enlarge_dialog = EnlargeDialog(
            parent=self.main_window,
            image_source=source,
            shared_worker=existing_worker,  # 기존 worker 전달
        )

        main_window_center = self.main_window.frameGeometry().center()
        dialog_geometry = enlarge_dialog.frameGeometry()
        dialog_geometry.moveCenter(main_window_center)
        enlarge_dialog.move(dialog_geometry.topLeft())

        enlarge_dialog.finished.connect(lambda: self.enlarge_dialogs.pop(source, None))

        enlarge_dialog.show()
        self.enlarge_dialogs[source] = enlarge_dialog
        print(f"[CameraController] EnlargeDialog created and shown for {source}")


    def release_all(self):
        """(수정) 모든 카메라 리소스를 안전하게 해제합니다."""
        print("[CameraController] Releasing all resources...")
        
        # 1. 팝업창 닫기
        for dialog in list(self.enlarge_dialogs.values()):
            # dialog.stop_video_stream() # 이 메서드가 없다면 제거하거나 close()만 호출
            dialog.close()
        self.enlarge_dialogs.clear()

        # 2. [단계별 종료 1] 모든 워커에게 "멈춰!" 신호 보내기 (Stop Flag)
        # 스레드를 바로 끄지 않고, 내부 루프를 탈출할 시간을 줍니다.
        for worker in self.workers:
            if worker:
                worker.stop()

        # 워커들이 루프를 빠져나올 시간을 조금 줍니다 (AI 연산 중일 수 있음)
        # QThread.msleep(100) # 필요시 사용 (보통은 불필요)

        # 3. [단계별 종료 2] 스레드 종료 대기
        thread_list_copy = list(self.threads)
        print(f"[CameraController] {len(thread_list_copy)}개의 스레드 종료 대기 중...")

        for i, thread in enumerate(thread_list_copy):
            try:
                if thread.isRunning():
                    # 스레드 루프 종료 요청
                    thread.quit()
                    
                    # [수정] 대기 시간을 3000 -> 5000ms (5초)로 늘림
                    # TCP 타임아웃이나 AI 연산이 길어질 경우를 대비
                    if not thread.wait(5000):  
                        print(f"[CameraController] 스레드 {i} 응답 없음. 강제 종료(terminate)합니다.")
                        thread.terminate()
                        thread.wait(1000)
                    else:
                        print(f"[CameraController] 스레드 {i} 정상 종료됨.")
            except RuntimeError as e:
                print(f"[CameraController] 스레드 종료 중 오류: {e}")
        
        # 4. 리스트 비우기
        self.threads.clear()
        self.workers.clear()
        self.labels.clear() 
        self.camera_sources.clear() 
        self.camera_rois.clear()    
        
        print("[CameraController] 모든 리소스가 안전하게 해제되었습니다.")