# widget_gy.py
# 메인 윈도우 및 설정 다이얼로그 관리 모듈
# 11/5 : 프로그램 수정보완
# 11/6 : X_dist, Y_dist, x_ref DB 저장 및 로드 추가
# 11/7 : ROI 추가시 마우스좌표 표시 추가, Lidar 및 Camera 연결 ON/OFF 기능 추가
# 11/12: ROI 직선 방정식 계산 함수 추가, extended ROI 그리기 기능 구현
# 11/13: 설정창 십자선 그리기 기능 추가, tmap에 사람 위치 표시 기능 추가
# 11/27: cam2-cam3, cam3-cam4 중복부분 객체 인식 처리,
# 11/28: truck/hook Map에 반영
# 11/29: Status 초기값을 Normal로 고정, 조건별 상태 변경 함수(update_safety_status) 추가
# 12/1 : 상태 판단 로직 개선, Warning 깜빡임 효과 추가
# 12/5 : cam1, cam5 map 반영 추가, 중복 처리 개선
# 12/8 : cam1, cam5 연속 프레임 필터링 추가, setting창, enlarge 창 줌/팬 기능 추가
#        ROI 설정 시에만 Zoom 허용, 우클릭 Pan, 좌표 변환 로직 추가
# 12/11 : ROI 위쪽에 있는 truck 도 map에 표시하도록 수정, danger 상태 로직 개선, substream 해상도 변경
# 12/15 : map 상에 트럭 표시 안정화 로직 추가. hook이 roi 위 일정범위 안에 있으면 표시하도록 수정
# 12/23 : cam2,3,4 화면 양끝단에 걸친 객체의 경우, 중복문제 해결 로직 개선
# 12/26 : danger 상태 진입 로직 개선, test_force_danger 플래그 추가(D키: 테스트 모드)

import os
# os.environ["OPENCV_VIDEOIO_DEBUG"] = "1" # OpenCV 비디오 I/O 디버그 활성화
import cv2
import sys
import sqlite3
import ast  # 저장된 좌표 문자열 [(x, y), ...]를 안전하게 리스트로 변환
import numpy as np
import time
from datetime import datetime
from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox, QDialog, QLayout
from PySide6.QtCore import Qt, QSize, QTimer, QThread, QEvent, QPoint, QRect
from PySide6.QtGui import QPixmap, QPainter, QPen, QColor, QImage
from mainwindowgyui import Ui_mainWindow
from alarm_dlgui import Ui_alarmDialog
from setting_dlgui import Ui_Setting  
from camera_gy import CameraController, CustomMapLabel
from alarm_gy import AlarmDialogManager
from lidar_worker import LidarWorker
from enlarge_image import EnlargeDialog


#직선 방정식 계산을 위한 헬퍼 함수
def _get_line_equation(p1: tuple[int, int], p2: tuple[int, int]) -> tuple[int, int, int]:
    x1, y1 = p1
    x2, y2 = p2
    A = y2 - y1
    B = x1 - x2
    C = (x2 * y1) - (x1 * y2)
    return A, B, C

def _calculate_roi_line_equations(roi_coords_str: str):
    if not roi_coords_str or not roi_coords_str.strip():
        return None, None
    try:
        roi_points = ast.literal_eval(roi_coords_str)
        if not isinstance(roi_points, list) or len(roi_points) != 4:
            return None, None
        P1 = roi_points[0]
        P2 = roi_points[1]
        P3 = roi_points[2]
        P4 = roi_points[3]
        line_p2_p3_eq = _get_line_equation(P2, P3)
        line_p1_p4_eq = _get_line_equation(P1, P4)
        return line_p2_p3_eq, line_p1_p4_eq
    except (ValueError, SyntaxError, TypeError) as e:
        return None, None
    
def _calculate_point_at_y130(A, B, C):
    if A is None or B is None or C is None: return None
    if A == 0 or A == 0.0: return None
    try:
        x_val = (-130.0 * B - C) / A
        return (x_val, 130.0) 
    except Exception as e:
        return None
    

class widget(QMainWindow):
    def __init__(self, app):
        self.app = app
        super().__init__()
        self.ui = Ui_mainWindow()
        self.ui.setupUi(self)

        self.setWindowTitle("GYsteel - AI Control System")
        # self.resize(1280, 720)
        
        # ▼▼▼ 설정창 줌/팬 상태 관리 변수 초기화 ▼▼▼
        self.setting_view_states = {}
        for i in range(1, 6):
            self.setting_view_states[f'CAM{i}'] = {'zoom': 1.0, 'pan': QPoint(0, 0)}
            
        self.is_setting_panning = False
        self.last_setting_mouse_pos = QPoint()
        
        self.layout_points_cache = {}
        self.hook_layout_points_cache = {}
        self.truck_layout_points_cache = {}
        
        self.truck_persistence_counter = {}
        
        self.hook_ema_cache = {} 
        self.hook_display_cache = {}
        
        self.camera_alert_states = {} 
        self.alert_cooldowns = {} 
        
        self.is_blink_on = False      
        
        # ▼▼▼ [테스트용] 강제 위험 상황 트리거 변수 추가 ▼▼▼
        self.is_test_mode_active = False
        self.test_force_danger = False

        self.statusBar().showMessage("프로그램이 시작되었습니다.")
        
        self.alarm_manager = AlarmDialogManager(self, Ui_alarmDialog)
        
        if hasattr(self.ui, "actionQuit"):
            self.ui.actionQuit.triggered.connect(self.종료)
        if hasattr(self.ui, "actionAbout"):
            self.ui.actionAbout.triggered.connect(self.about)
        if hasattr(self.ui, "actionAlarmInfo"):
            self.ui.actionAlarmInfo.triggered.connect(self.alarm_manager.show_alarm_dialog)
            
        if hasattr(self.ui, "actionCamera"):
            self.ui.actionCamera.setText("Camera OFF") 
            self.ui.actionCamera.triggered.connect(self.toggle_camera_stream)
        
        if hasattr(self.ui, "actionSetting"):
            self.ui.actionSetting.triggered.connect(self.show_setting_dialog)
            
        if hasattr(self.ui, "actionLidar"):
            self.ui.actionLidar.setText("Lidar OFF") 
            self.ui.actionLidar.triggered.connect(self.toggle_lidar_stream)
        
        self.lidar_thread = None
        self.lidar_worker = None
        self.ui.lbllidar.setScaledContents(True) 
        self.ui.lbllidar.installEventFilter(self)
        self.enlarge_dialogs = {}           

        self.camera_controller = CameraController(self)
        self.camera_controller.objects_detected_in_worker.connect(self.handle_objects_detection)
        
        self.setup_tmap_label()
        
        self.app.aboutToQuit.connect(self.cleanup_on_quit)
        
        self.tmap_label.set_area2_state('normal')
        
        self.roi_draw_state = {
            'is_drawing': False,   
            'target_cam': None,    
            'points': [],          
            'target_label': None,  
            'target_textbox': None 
        }
        
        self.cam_points_at_y130 = {
            'CAM1': {'point_p2p3': None, 'point_p1p4': None},
            'CAM2': {'point_p2p3': None, 'point_p1p4': None},
            'CAM3': {'point_p2p3': None, 'point_p1p4': None},
            'CAM4': {'point_p2p3': None, 'point_p1p4': None},
            'CAM5': {'point_p2p3': None, 'point_p1p4': None}
        }
        
        self.shared_truck_height = None
        self.shared_truck_height_cam5 = None
        
        # [수정] CAM1, CAM5 트럭 길이 안정화를 위한 변수들
        self.cam1_height_ema = None 
        self.cam5_height_ema = None
        
        self.warning_timer = QTimer(self)
        self.warning_timer.setInterval(1000)  
        self.warning_timer.timeout.connect(self._animate_warning_labels)
        
        self.camera_alert_states = {} 
        self.is_blink_on = False      
        
        self.db_cam_settings = {}
        self.current_setting_pixmap = None
        
        self.apply_db_settings_to_workers()    
        self.load_all_cameras()
        
        self.ui.lblcam2.setStyleSheet("border: 5px solid transparent;")
        self.ui.lblcam3.setStyleSheet("border: 5px solid transparent;")
        self.ui.lblcam4.setStyleSheet("border: 5px solid transparent;")

    # ▼▼▼ [신규] 화면 좌표(Label) -> 실제 이미지 좌표(Raw) 변환 헬퍼 ▼▼▼
    def _get_image_coords_from_widget(self, widget_pos, cam_name, label_size):
        state = self.setting_view_states.get(cam_name)
        if not state: return widget_pos.x(), widget_pos.y()

        zoom = state['zoom']
        pan = state['pan']
        
        img_w, img_h = 1920, 1080 # Default fallback
        info = self.camera_controller.labels.get(f"lbl{cam_name.lower()}")
        if info and info['worker'] and info['worker'].raw_frame is not None:
             img_h, img_w, _ = info['worker'].raw_frame.shape

        view_w = img_w / zoom
        view_h = img_h / zoom
        
        center_x = (img_w / 2) - pan.x()
        center_y = (img_h / 2) - pan.y()
        
        crop_x = center_x - (view_w / 2)
        crop_y = center_y - (view_h / 2)
        
        crop_x = max(0, min(crop_x, img_w - view_w))
        crop_y = max(0, min(crop_y, img_h - view_h))
        
        scale_x = view_w / label_size.width()
        scale_y = view_h / label_size.height()
        
        real_x = crop_x + (widget_pos.x() * scale_x)
        real_y = crop_y + (widget_pos.y() * scale_y)
        
        return int(real_x), int(real_y)

    # ▼▼▼ [신규] 실제 이미지 좌표(Raw) -> 화면 좌표(Label) 변환 헬퍼 ▼▼▼
    def _get_widget_coords_from_image(self, img_x, img_y, cam_name, label_size):
        state = self.setting_view_states.get(cam_name)
        zoom = state['zoom']
        pan = state['pan']
        
        img_w, img_h = 1920, 1080 
        info = self.camera_controller.labels.get(f"lbl{cam_name.lower()}")
        if info and info['worker'] and info['worker'].raw_frame is not None:
             img_h, img_w, _ = info['worker'].raw_frame.shape

        view_w = img_w / zoom
        view_h = img_h / zoom
        
        center_x = (img_w / 2) - pan.x()
        center_y = (img_h / 2) - pan.y()
        
        crop_x = max(0, min(center_x - (view_w / 2), img_w - view_w))
        crop_y = max(0, min(center_y - (view_h / 2), img_h - view_h))
        
        scale_x = label_size.width() / view_w
        scale_y = label_size.height() / view_h
        
        widget_x = (img_x - crop_x) * scale_x
        widget_y = (img_y - crop_y) * scale_y
        
        return QPoint(int(widget_x), int(widget_y))

    # ▼▼▼ 설정창 이미지 줌/팬 처리 헬퍼 함수 ▼▼▼
    def _get_zoomed_pixmap_for_setting(self, q_img, zoom_factor, pan_offset, target_size):
        if q_img is None: return QPixmap()

        if zoom_factor == 1.0:
            return QPixmap.fromImage(q_img).scaled(target_size, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)

        img_w = q_img.width()
        img_h = q_img.height()

        view_w = int(img_w / zoom_factor)
        view_h = int(img_h / zoom_factor)

        center_x = (img_w // 2) - pan_offset.x()
        center_y = (img_h // 2) - pan_offset.y()

        crop_x = center_x - (view_w // 2)
        crop_y = center_y - (view_h // 2)

        crop_x = max(0, min(crop_x, img_w - view_w))
        crop_y = max(0, min(crop_y, img_h - view_h))

        rect = QRect(int(crop_x), int(crop_y), view_w, view_h)
        cropped_img = q_img.copy(rect)

        return QPixmap.fromImage(cropped_img).scaled(target_size, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
    
    
    # ▼▼▼ 현재 활성화된 탭의 이미지를 갱신 (저장된 ROI 표시 기능 추가) ▼▼▼
    def update_current_setting_image(self):
        if not hasattr(self, 'ui_setting'): return
        
        index = self.ui_setting.tabWidget.currentIndex()
        cam_names = ['CAM1', 'CAM2', 'CAM3', 'CAM4', 'CAM5']
        if not (0 <= index < len(cam_names)): return
        
        cam_name = cam_names[index]
        state = self.setting_view_states[cam_name]
        
        setting_label = getattr(self.ui_setting, f"lbltabimg_{cam_name.lower()}")
        
        info = self.camera_controller.labels.get(f"lbl{cam_name.lower()}")
        q_image = None
        
        if info and info['worker'] and info['worker'].raw_frame is not None:
            try:
                frame = info['worker'].raw_frame
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame_rgb.shape
                bytes_per_line = ch * w
                q_image = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            except Exception: pass
            
        if setting_label and q_image:
            # 1. 라벨 크기 확보 (초기화 이슈 방지)
            target_size = setting_label.size()
            if target_size.width() < 100 or target_size.height() < 100:
                target_size = QSize(1250, 700)

            # 2. 줌/팬 적용된 기본 이미지 생성
            final_pixmap = self._get_zoomed_pixmap_for_setting(
                q_image, state['zoom'], state['pan'], target_size
            )
            
            # ▼▼▼ [추가된 부분] 저장된 ROI가 있다면 이미지 위에 그리기 ▼▼▼
            try:
                # 현재 탭의 체크박스와 텍스트박스 가져오기
                chk_roi = getattr(self.ui_setting, f"chkroi_{cam_name.lower()}")
                txt_roi = getattr(self.ui_setting, f"txtroi_{cam_name.lower()}")
                
                # 체크박스가 켜져 있고(Checked), 좌표 텍스트가 있다면 그리기 수행
                if chk_roi.isChecked() and txt_roi.toPlainText().strip():
                    roi_str = txt_roi.toPlainText()
                    roi_coords = ast.literal_eval(roi_str)
                    
                    if isinstance(roi_coords, list) and len(roi_coords) == 4:
                        painter = QPainter(final_pixmap)
                        # 저장된 ROI는 Cyan(하늘색)으로 그려서 구별 (두께 2)
                        pen = QPen(QColor(0, 255, 255), 2) 
                        painter.setPen(pen)
                        
                        screen_pts = []
                        for (rx, ry) in roi_coords:
                            # 헬퍼 함수를 이용해 [원본좌표 -> 현재 줌/팬 상태의 화면좌표]로 변환
                            # 주의: target_size를 넘겨주어야 현재 생성된 pixmap 기준의 정확한 위치가 나옴
                            pt = self._get_widget_coords_from_image(rx, ry, cam_name, target_size)
                            screen_pts.append(pt)
                        
                        # 다각형 그리기
                        if len(screen_pts) > 0:
                            painter.drawPolygon(screen_pts)
                        
                        painter.end()
            except Exception:
                pass # 좌표 파싱 실패 시 무시
            # ▲▲▲ [추가 끝] ▲▲▲

            # 3. 라벨 설정 및 이미지 표시
            setting_label.setScaledContents(True)
            self.current_setting_pixmap = final_pixmap.copy() 
            setting_label.setPixmap(final_pixmap)
            
            # 4. (새로 그리는 중이라면) 가이드라인 덧그리기
            if self.roi_draw_state['is_drawing'] and self.roi_draw_state['target_cam'] == cam_name:
                self.draw_roi_feedback()
                
    
    # 헬퍼: 십자선 및 ROI 그리기 업데이트
    def update_crosshair_on_setting(self, label_widget, mouse_pos, cam_name):
        if self.current_setting_pixmap and not self.current_setting_pixmap.isNull():
            temp_pixmap = self.current_setting_pixmap.copy()
            painter = QPainter(temp_pixmap)
            
            # 1. 이미 찍힌 ROI 점들 그리기 (화면 좌표로 변환 필요)
            if self.roi_draw_state['is_drawing'] and self.roi_draw_state['points']:
                pen_roi = QPen(QColor(0, 255, 0)) 
                pen_roi.setWidth(2)
                painter.setPen(pen_roi)
                
                screen_points = []
                for pt_tuple in self.roi_draw_state['points']:
                    screen_pt = self._get_widget_coords_from_image(pt_tuple[0], pt_tuple[1], cam_name, label_widget.size())
                    screen_points.append(screen_pt)
                    painter.drawEllipse(screen_pt, 4, 4) 

                if len(screen_points) > 1:
                    painter.drawPolyline(screen_points)
                    
            # 2. 십자선 그리기
            pen_cross = QPen(QColor(255, 0, 0)) 
            pen_cross.setWidth(1)
            painter.setPen(pen_cross)
            painter.drawLine(0, mouse_pos.y(), temp_pixmap.width(), mouse_pos.y()) 
            painter.drawLine(mouse_pos.x(), 0, mouse_pos.x(), temp_pixmap.height()) 
            
            painter.end()
            label_widget.setPixmap(temp_pixmap)

    # -------------------------------------------------------------
    # [수정] 이벤트 필터: 줌(그리기 모드일 때만), 우클릭 팬, 좌클릭 좌표변환
    # -------------------------------------------------------------
    def eventFilter(self, watched, event):
        if hasattr(self, 'ui_setting') and hasattr(self, 'current_setting_dialog'): 
            setting_labels = [
                self.ui_setting.lbltabimg_cam1, self.ui_setting.lbltabimg_cam2,
                self.ui_setting.lbltabimg_cam3, self.ui_setting.lbltabimg_cam4,
                self.ui_setting.lbltabimg_cam5
            ]

            if watched in setting_labels:
                index = setting_labels.index(watched)
                cam_name = f"CAM{index + 1}"
                state = self.setting_view_states[cam_name]

                # 1. Wheel Event
                if event.type() == QEvent.Type.Wheel:
                    if self.roi_draw_state['is_drawing']:
                        delta = event.angleDelta().y()
                        zoom_factor = 1.1 if delta > 0 else 0.9
                        new_zoom = state['zoom'] * zoom_factor
                        new_zoom = max(1.0, min(new_zoom, 10.0))

                        mouse_pos = event.position().toPoint()
                        real_x, real_y = self._get_image_coords_from_widget(mouse_pos, cam_name, watched.size())
                        
                        state['zoom'] = new_zoom
                        
                        img_w, img_h = 1920, 1080 
                        info = self.camera_controller.labels.get(f"lbl{cam_name.lower()}")
                        if info and info['worker'] and info['worker'].raw_frame is not None:
                            img_h, img_w, _ = info['worker'].raw_frame.shape

                        new_view_w = img_w / new_zoom
                        new_view_h = img_h / new_zoom
                        
                        ratio_x = mouse_pos.x() / watched.width()
                        ratio_y = mouse_pos.y() / watched.height()
                        
                        new_crop_x = real_x - (new_view_w * ratio_x)
                        new_crop_y = real_y - (new_view_h * ratio_y)
                        
                        new_center_x = new_crop_x + (new_view_w / 2)
                        new_center_y = new_crop_y + (new_view_h / 2)
                        
                        state['pan'] = QPoint(int((img_w / 2) - new_center_x), int((img_h / 2) - new_center_y))

                        # [수정] 휠 줌 시 윈도우 제목에 줌 배율 및 좌표 업데이트
                        title = f"{self.original_dialog_title} - 좌표: ({real_x}, {real_y}) [Zoom: x{state['zoom']:.1f}]"
                        self.current_setting_dialog.setWindowTitle(title)
                        
                        self.update_current_setting_image()
                        self.update_crosshair_on_setting(watched, mouse_pos, cam_name)
                        return True

                # 2. Mouse Press
                if event.type() == QEvent.Type.MouseButtonPress:
                    if event.button() == Qt.MouseButton.LeftButton:
                        if self.roi_draw_state['is_drawing'] and watched == self.roi_draw_state['target_label']:
                            real_x, real_y = self._get_image_coords_from_widget(event.position().toPoint(), cam_name, watched.size())
                            self.handle_roi_click(QPoint(real_x, real_y)) 
                            return True
                    elif event.button() == Qt.MouseButton.RightButton:
                        if state['zoom'] > 1.0:
                            self.is_setting_panning = True
                            self.last_setting_mouse_pos = event.position().toPoint()
                            watched.setCursor(Qt.ClosedHandCursor)
                            return True

                # 3. Mouse Move
                if event.type() == QEvent.Type.MouseMove:
                    pos = event.position().toPoint()
                    
                    if self.is_setting_panning:
                        delta = pos - self.last_setting_mouse_pos
                        self.last_setting_mouse_pos = pos
                        img_w = 1920 
                        info = self.camera_controller.labels.get(f"lbl{cam_name.lower()}")
                        if info and info['worker'] and info['worker'].raw_frame is not None:
                             img_w = info['worker'].raw_frame.shape[1]
                        view_w = img_w / state['zoom']
                        pixel_ratio = view_w / watched.width()
                        state['pan'] += QPoint(int(delta.x() * pixel_ratio), int(delta.y() * pixel_ratio))
                        self.update_current_setting_image()
                        return True
                    
                    real_x, real_y = self._get_image_coords_from_widget(pos, cam_name, watched.size())
                    
                    # [핵심 수정] Setting 창 제목 표시줄(Header)에 좌표 표시
                    title_text = f"{self.original_dialog_title} - 좌표: ({real_x}, {real_y})"
                    if state['zoom'] > 1.0:
                        title_text += f" [Zoom: x{state['zoom']:.1f}]"
                    self.current_setting_dialog.setWindowTitle(title_text)
                    
                    # 십자선 업데이트 (텍스트는 전달하지 않음)
                    self.update_crosshair_on_setting(watched, pos, cam_name)
                    return True 

                # ... (이하 동일) ...
                if event.type() == QEvent.Type.MouseButtonRelease:
                    if event.button() == Qt.MouseButton.RightButton:
                        if self.is_setting_panning:
                            self.is_setting_panning = False
                            watched.setCursor(Qt.ArrowCursor)
                            return True

                # 마우스가 이미지 밖으로 나가면 제목 원복
                if event.type() == QEvent.Type.Leave:
                     if self.current_setting_pixmap and not self.current_setting_pixmap.isNull():
                         watched.setPixmap(self.current_setting_pixmap)
                     self.current_setting_dialog.setWindowTitle(self.original_dialog_title)
                     return True

        if watched == self.ui.lbllidar:
            if event.type() == QEvent.Type.MouseButtonPress:
                if event.button() == Qt.MouseButton.LeftButton:
                    self.show_enlarged_lidar() 
                    return True

        return super().eventFilter(watched, event)
    

    def start_roi_drawing(self, cam_name, label_widget, text_widget, checkbox_widget):
        # [수정] ROI 설정 시작 시 줌/팬 초기화 (필수)
        self.setting_view_states[cam_name]['zoom'] = 1.0
        self.setting_view_states[cam_name]['pan'] = QPoint(0, 0)
        self.update_current_setting_image()
        
        if text_widget:
            text_widget.clear() 
        if checkbox_widget:
            checkbox_widget.setChecked(False)
            checkbox_widget.setEnabled(False) 
        
        self.roi_draw_state['points'] = []
        if hasattr(self, 'ui_setting'): 
            self.on_tab_changed(self.ui_setting.tabWidget.currentIndex())

        self.roi_draw_state['is_drawing'] = True
        self.roi_draw_state['target_cam'] = cam_name
        self.roi_draw_state['target_label'] = label_widget
        self.roi_draw_state['target_textbox'] = text_widget
        self.roi_draw_state['target_checkbox'] = checkbox_widget
        
        self.statusBar().showMessage(f"{cam_name}: ROI 그리기를 시작합니다. 4개의 점을 클릭하세요. (휠: Zoom, 우클릭: Pan)")

    def handle_roi_click(self, pos: QPoint):
        state = self.roi_draw_state
        state['points'].append((pos.x(), pos.y()))
        
        # 점을 찍을 때마다 피드백(선 그리기) 갱신
        self.draw_roi_feedback()

        # 4번째 점이 찍히면 ROI 설정 완료
        if len(state['points']) == 4:
            # 1. 좌표 텍스트박스 업데이트
            state['target_textbox'].setText(str(state['points'])) 
            
            # 2. 체크박스 활성화 및 체크
            if state['target_checkbox']:
                state['target_checkbox'].setEnabled(True)
                state['target_checkbox'].setChecked(True)
            
            # 3. [핵심 수정] 줌/팬 상태를 원본으로 초기화
            cam_name = state['target_cam']
            self.setting_view_states[cam_name]['zoom'] = 1.0
            self.setting_view_states[cam_name]['pan'] = QPoint(0, 0)
            
            # 4. 설정창 제목 초기화 (Zoom x... 텍스트 제거)
            if hasattr(self, 'current_setting_dialog') and hasattr(self, 'original_dialog_title'):
                self.current_setting_dialog.setWindowTitle(self.original_dialog_title)

            # 5. 그리기 모드 종료
            self.roi_draw_state['is_drawing'] = False
            
            # 6. 화면 갱신 (초기화된 사이즈로 ROI 포함하여 다시 그리기)
            self.update_current_setting_image()
            
            self.statusBar().showMessage(f"{state['target_cam']}: ROI 설정 완료. 화면이 원래 크기로 복귀되었습니다.")
        else:
            self.statusBar().showMessage(f"{len(state['points'])}/4 점 선택됨. 다음 점을 클릭하세요.")

    def draw_roi_feedback(self):
        state = self.roi_draw_state
        if not state['target_label']: return

        # 현재 줌/팬이 적용된 Pixmap 캐시를 가져옴
        if self.current_setting_pixmap and not self.current_setting_pixmap.isNull():
            temp_pixmap = self.current_setting_pixmap.copy()
        else: return 

        painter = QPainter(temp_pixmap)
        pen = QPen(QColor(0, 255, 0))
        pen.setWidth(2)
        painter.setPen(pen)
        
        cam_name = state['target_cam']
        label_size = state['target_label'].size()

        # [수정] 저장된 원본 좌표를 현재 화면 좌표로 변환하여 그리기
        screen_points = []
        for (x, y) in state['points']:
            pt = self._get_widget_coords_from_image(x, y, cam_name, label_size)
            screen_points.append(pt)
            painter.drawEllipse(pt, 3, 3) 

        if len(screen_points) > 1:
            painter.drawPolyline(screen_points)

        if len(screen_points) == 4:
            painter.drawLine(screen_points[3], screen_points[0])

        painter.end() 
        state['target_label'].setPixmap(temp_pixmap)
        
    def load_all_cameras(self):
        self.statusBar().showMessage("모든 카메라를 시작합니다...")
        self.camera_controller.setup_cameras()
        self.apply_db_settings_to_workers()
        self.ui.actionCamera.setText("Camera ON")
        
    def stop_all_cameras(self):
        self.statusBar().showMessage("모든 카메라 연결을 중지합니다.")
        self.camera_controller.release_all()
        self.camera_controller._is_setup = False
        self.ui.actionCamera.setText("Camera OFF")
        
        try:
            self.ui.lblcam1.clear()
            self.ui.lblcam2.clear()
            self.ui.lblcam3.clear()
            self.ui.lblcam4.clear()
            self.ui.lblcam5.clear()
            
            self.ui.lblcam1.setText("cam1")
            self.ui.lblcam2.setText("cam2")
            self.ui.lblcam3.setText("cam3")
            self.ui.lblcam4.setText("cam4")
            self.ui.lblcam5.setText("cam5")
        except AttributeError: pass
        
    def toggle_camera_stream(self):        
        if self.camera_controller._is_setup:
            self.stop_all_cameras()
        else:
            self.load_all_cameras()
    
    def setup_tmap_label(self):
        self.ui.lbltmap.hide()
        self.tmap_label = CustomMapLabel(self.ui.gridLayoutWidget)
        self.tmap_label.setObjectName("tmap_label")
        self.tmap_label.setAlignment(Qt.AlignCenter)
        self.tmap_label.setMinimumSize(QSize(600, 400))
        layout = self.ui.gridLayout
        old_item = layout.itemAtPosition(2, 1)
        if old_item:
            layout.removeItem(old_item)
            layout.addWidget(self.tmap_label, 2, 0, 1, 3) 
        self.tmap_label.rects = [(100, 30, 860, 200), (1000, 30, 200, 200)]
        self.tmap_label.repaint()

    
    def show_setting_dialog(self):
        setting_dialog = QDialog(self)
        self.ui_setting = Ui_Setting()
        self.ui_setting.setupUi(setting_dialog)
        
        # 1. 레이아웃 크기 제약 해제 & 이미지 꽉 채우기 (필수)
        self.ui_setting.verticalLayout_6.setSizeConstraint(QLayout.SetDefaultConstraint)
        self.ui_setting.lbltabimg_cam1.setScaledContents(True)

        self.current_setting_dialog = setting_dialog
        try:
            self.original_dialog_title = setting_dialog.windowTitle() 
            if not self.original_dialog_title: self.original_dialog_title = "Setting"
        except Exception:
            self.original_dialog_title = "Setting" 

        if not self.camera_controller._is_setup:
            self.camera_controller.setup_cameras()
            
        self.ui_setting.tabWidget.currentChanged.connect(self.on_tab_changed)
        
        self.ui_setting.chkroi_cam1.stateChanged.connect(lambda: self.on_tab_changed(0))
        self.ui_setting.chkroi_cam2.stateChanged.connect(lambda: self.on_tab_changed(1))
        self.ui_setting.chkroi_cam3.stateChanged.connect(lambda: self.on_tab_changed(2))
        self.ui_setting.chkroi_cam4.stateChanged.connect(lambda: self.on_tab_changed(3))
        self.ui_setting.chkroi_cam5.stateChanged.connect(lambda: self.on_tab_changed(4))

        self.ui_setting.pbnroiset_cam1.clicked.connect(
            lambda: self.start_roi_drawing('CAM1', self.ui_setting.lbltabimg_cam1, self.ui_setting.txtroi_cam1, self.ui_setting.chkroi_cam1))
        self.ui_setting.pbnroiset_cam2.clicked.connect(
            lambda: self.start_roi_drawing('CAM2', self.ui_setting.lbltabimg_cam2, self.ui_setting.txtroi_cam2, self.ui_setting.chkroi_cam2))
        self.ui_setting.pbnroiset_cam3.clicked.connect(
            lambda: self.start_roi_drawing('CAM3', self.ui_setting.lbltabimg_cam3, self.ui_setting.txtroi_cam3, self.ui_setting.chkroi_cam3))
        self.ui_setting.pbnroiset_cam4.clicked.connect(
            lambda: self.start_roi_drawing('CAM4', self.ui_setting.lbltabimg_cam4, self.ui_setting.txtroi_cam4, self.ui_setting.chkroi_cam4))
        self.ui_setting.pbnroiset_cam5.clicked.connect(
            lambda: self.start_roi_drawing('CAM5', self.ui_setting.lbltabimg_cam5, self.ui_setting.txtroi_cam5, self.ui_setting.chkroi_cam5))

        self.ui_setting.pbnsave_cam1.clicked.connect(lambda: self.save_settings_to_db(self.ui_setting, 'CAM1'))
        self.ui_setting.pbnsave_cam2.clicked.connect(lambda: self.save_settings_to_db(self.ui_setting, 'CAM2'))
        self.ui_setting.pbnsave_cam3.clicked.connect(lambda: self.save_settings_to_db(self.ui_setting, 'CAM3'))
        self.ui_setting.pbnsave_cam4.clicked.connect(lambda: self.save_settings_to_db(self.ui_setting, 'CAM4'))
        self.ui_setting.pbnsave_cam5.clicked.connect(lambda: self.save_settings_to_db(self.ui_setting, 'CAM5'))

        self.ui_setting.lbltabimg_cam1.installEventFilter(self)
        self.ui_setting.lbltabimg_cam2.installEventFilter(self)
        self.ui_setting.lbltabimg_cam3.installEventFilter(self)
        self.ui_setting.lbltabimg_cam4.installEventFilter(self)
        self.ui_setting.lbltabimg_cam5.installEventFilter(self)
        
        self.ui_setting.lbltabimg_cam1.setMouseTracking(True)
        self.ui_setting.lbltabimg_cam2.setMouseTracking(True)
        self.ui_setting.lbltabimg_cam3.setMouseTracking(True)
        self.ui_setting.lbltabimg_cam4.setMouseTracking(True)
        self.ui_setting.lbltabimg_cam5.setMouseTracking(True)

        self.load_settings_from_db(self.ui_setting)
        self.update_all_roi_checkbox_visibility()
        self.ui_setting.tabWidget.setCurrentIndex(0)
        self.on_tab_changed(0) 

        setting_dialog.exec()

        if hasattr(self, 'current_setting_dialog'): del self.current_setting_dialog
        if hasattr(self, 'original_dialog_title'): del self.original_dialog_title
    

    def on_tab_changed(self, index):
        if self.roi_draw_state['is_drawing']:
            self.roi_draw_state['is_drawing'] = False
            self.roi_draw_state['points'] = []
            self.statusBar().showMessage("ROI 그리기가 취소되었습니다.")
            
        self.update_current_setting_image()
        
        cam_names = ['CAM1', 'CAM2', 'CAM3', 'CAM4', 'CAM5']
        if 0 <= index < len(cam_names):
            cam_name = cam_names[index]
        setting_label = getattr(self.ui_setting, f"lbltabimg_{cam_name.lower()}")
        
        # 줌이 1.0일 때만 저장된 ROI를 표시 (좌표 왜곡 방지 위해 단순화)
        # 하지만 update_current_setting_image에서 이미 처리하므로 여기선 텍스트 로드 위주
        pass 
    
    def set_label_roi_coords(self, label_name, roi_coords_str):
        if label_name in self.camera_controller.labels and 'label_widget' in self.camera_controller.labels[label_name]:
            self.camera_controller.labels[label_name]['label_widget'].set_roi_coords(roi_coords_str)

    def set_label_roi_display(self, label_name, display_state):
        if label_name in self.camera_controller.labels and 'label_widget' in self.camera_controller.labels[label_name]:
            self.camera_controller.labels[label_name]['label_widget'].set_roi_display_state(display_state)
            
    def set_label_status(self, label_name, alarm_active, status_text):
        if label_name in self.camera_controller.labels and 'label_widget' in self.camera_controller.labels[label_name]:
            self.camera_controller.labels[label_name]['label_widget'].set_status(alarm_active, status_text)
            
    def calculate_iou(self, boxA, boxB):
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        interArea = max(0, xB - xA) * max(0, yB - yA)
        if interArea == 0: return 0
        boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
        boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
        return interArea / float(boxAArea + boxBArea - interArea)
        
    def get_dist(self, p1, p2):
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) ** 0.5
            
    
    def update_safety_status(self):
        # ▼▼▼ [추가] 테스트 모드일 때는 실시간 판단 로직을 수행하지 않고 나감 ▼▼▼
        # if self.is_test_mode_active:
        #     return
        
        target_cams = ['lblcam2', 'lblcam3', 'lblcam4']
        current_frame_states = {}
        current_time = time.time() 
        
        # [설정] 이미지 좌표계(1280x720) 기준 임계값
        WARNING_DIST = 80.0  
        WARNING_IOU = 0.05
        WARNING_ANGLE_THRESHOLD = 30.0  # hook 각도 임계값
        WARNING_MAP_DIST = 10.0

        # [헬퍼] 점이 트럭 박스 안에 있는지 확인 (이미지 좌표계 사용)
        # rect_tuple: (cx, bottom_y, w, h)
        def is_point_in_img_rect(px, py, rect_tuple):
            rcx, rcy, rw, rh = rect_tuple
            x_left = rcx - (rw / 2)
            x_right = rcx + (rw / 2)
            y_top = rcy - rh     # 트럭의 윗면 (Y값이 작음)
            y_bottom = rcy       # 트럭의 바닥면 (Y값이 큼)
            
            return (x_left <= px <= x_right) and (y_top <= py <= y_bottom)

        for cam in target_cams:
            cam_state = 'normal'
            warning_detail_msg = None
            
            # 1. 데이터 가져오기
            people_data = self.layout_points_cache.get(cam, [])
            hook_data = self.hook_layout_points_cache.get(cam, [])
            truck_data = self.truck_layout_points_cache.get(cam, [])
            
            # 2. 사람 vs 훅 검사
            for p_item in people_data:
                if len(p_item) < 2: continue
                
                # ▼▼▼ [추가 1] 사람의 Map 좌표 추출 (layout_x, layout_y) ▼▼▼
                # p_item[0]은 (lx, ly) 혹은 (lx, ly, w, h) 형태임
                p_map_coords = p_item[0]
                p_map_x, p_map_y = p_map_coords[0], p_map_coords[1]
                
                # Raw 이미지 좌표 가져오기
                raw_p = p_item[1] 
                if not (isinstance(raw_p, (list, tuple)) and len(raw_p) == 4):
                    continue

                pcx, pcy_bottom, pw, ph = raw_p
                
                # [2단계용] 사람 이미지상 '중심점' 계산 (거리 계산용)
                p_img_center_x = pcx
                p_img_center_y = pcy_bottom - (ph / 2)
                
                # 사람 이미지상 BBox
                p_box_img = (pcx - pw/2, pcy_bottom - ph, pcx + pw/2, pcy_bottom)

                for h_item in hook_data:
                    if len(h_item) < 2: continue
                    
                    # ▼▼▼ [추가 2] 훅의 Map 좌표 추출 ▼▼▼
                    h_map_coords = h_item[0]
                    h_map_x, h_map_y = h_map_coords[0], h_map_coords[1]
                    
                    # 훅 데이터 파싱
                    raw_h_wrapper = h_item[1]
                    angle_val = 0.0
                    raw_h_coords = None
                    
                    if isinstance(raw_h_wrapper, (list, tuple)):
                        if len(raw_h_wrapper) == 2 and isinstance(raw_h_wrapper[0], (list, tuple)):
                            raw_h_coords = raw_h_wrapper[0]
                            angle_val = raw_h_wrapper[1]
                        elif len(raw_h_wrapper) == 4 and isinstance(raw_h_wrapper[0], (int, float)):
                             raw_h_coords = raw_h_wrapper
                             angle_val = 0.0

                    if raw_h_coords is None or len(raw_h_coords) != 4:
                        continue
                        
                    # [1단계] 각도 필터링 (28도 이하면 무시)
                    if abs(angle_val) <= WARNING_ANGLE_THRESHOLD: 
                        continue

                    hcx, hcy_bottom, hw, hh = raw_h_coords
                    
                    # [2단계용] 훅 이미지상 '중심점' 계산 (거리 계산용)
                    h_img_center_x = hcx
                    h_img_center_y = hcy_bottom - (hh / 2)
                    
                    # 훅 이미지상 BBox
                    h_box_img = (hcx - hw/2, hcy_bottom - hh, hcx + hw/2, hcy_bottom)

                    # ---------------------------------------------------------
                    # [2단계: Warning] 거리 및 IOU 계산 (중심점 기준 유지)
                    # ---------------------------------------------------------
                    # A. 이미지 기준 판단 (기존)
                    dist_img = self.get_dist((p_img_center_x, p_img_center_y), (h_img_center_x, h_img_center_y))
                    iou = self.calculate_iou(p_box_img, h_box_img)
                    cond_img_warning = (dist_img < WARNING_DIST) and (iou > WARNING_IOU)
                    
                    # B. Map 거리 기준 판단 (신규)
                    dist_map = ((p_map_x - h_map_x)**2 + (p_map_y - h_map_y)**2) ** 0.5
                    cond_map_warning = (dist_map <= WARNING_MAP_DIST)
                    
                    # ▼▼▼ [수정] 두 조건 중 하나라도 만족하면 Warning ▼▼▼
                    if cond_img_warning and cond_map_warning:
                        # warning_detail_msg = f"[{cam}] Warning! ImgDist:{dist_img:.1f}, IOU:{iou:.4f}, MapDist:{dist_map:.1f}"
                        warning_detail_msg = f"[{cam}] Warning! ImgDist:{dist_img:.1f}, IOU:{iou:.4f}, MapDist:{dist_map:.1f}, Angle:{angle_val:.1f}"
                        detected_grade = 'warning'
                        
                        # -----------------------------------------------------
                        # [3단계: Danger] 트럭 내부 여부 (★수정: 바닥점 기준★)
                        # -----------------------------------------------------
                        for t_item in truck_data:
                            if len(t_item) < 2: continue
                            
                            raw_t = t_item[1] # (cx, cy_bottom, w, h)
                            if len(raw_t) != 4: continue
                            
                            # [변경] 중심점 대신 '바닥 중심점(Bottom Center)'을 사용해 판단
                            # pcx, pcy_bottom : 사람의 발 위치
                            # hcx, hcy_bottom : 후크의 최하단 위치
                            p_in = is_point_in_img_rect(pcx, pcy_bottom, raw_t)
                            h_in = is_point_in_img_rect(hcx, hcy_bottom, raw_t)
                            
                            if p_in and h_in:
                                detected_grade = 'danger'
                                break 
                        
                        if cam_state != 'danger':
                            cam_state = detected_grade

            # ▼▼▼ [수정 2] 테스트용 강제 위험 상황 처리 ▼▼▼
            if self.test_force_danger and cam == 'CAM2':
                cam_state = 'danger'
                detected_grade = 'danger'
                            
            # 3. 알람 저장 및 쿨다운 관리 (기존 유지)
            if not hasattr(self, 'alert_cooldowns'): self.alert_cooldowns = {}
            if cam not in self.alert_cooldowns:
                self.alert_cooldowns[cam] = {'state': 'normal', 'until': 0.0, 'last_save': 0.0}
            
            last_alert = self.alert_cooldowns[cam]
            SAVE_INTERVAL = 2.0 
            
            if cam_state != 'normal':
                priority = {'danger': 2, 'warning': 1, 'normal': 0}
                if priority[cam_state] >= priority[last_alert['state']]:
                    time_diff = current_time - last_alert['last_save']
                    
                    if (cam_state != last_alert['state']) or (time_diff >= SAVE_INTERVAL):
                        # ▼▼▼ [수정 3] 실제로 저장/알람이 발생하는 시점에만 프린트!
                        if warning_detail_msg:
                            print(warning_detail_msg)
                            
                        cam_name_upper = cam.replace('lbl', '').upper()
                        self.save_alarm_snapshot(cam_name_upper, cam_state)
                        
                        self.alert_cooldowns[cam] = {
                            'state': cam_state,
                            'until': current_time + 2.0,
                            'last_save': current_time
                        }
                    else:
                        self.alert_cooldowns[cam]['until'] = current_time + 2.0
                        if priority[cam_state] > priority[last_alert['state']]:
                             self.alert_cooldowns[cam]['state'] = cam_state
            else:
                if current_time < last_alert['until']:
                    cam_state = last_alert['state']
                else:
                    self.alert_cooldowns[cam] = {'state': 'normal', 'until': 0.0, 'last_save': 0.0} 
                    
            self.tmap_label.update_cam_state(cam, cam_state)
            if cam_state != 'normal': current_frame_states[cam] = cam_state   
        
        # 4. 전체 시스템 상태 업데이트
        resolved_cams = set(self.camera_alert_states.keys()) - set(current_frame_states.keys())
        for cam_name in resolved_cams: self._reset_camera_style(cam_name)
        self.camera_alert_states = current_frame_states
        
        if self.camera_alert_states:
            if not self.warning_timer.isActive(): self.warning_timer.start()
        else:
            if self.warning_timer.isActive():
                self.warning_timer.stop()
                self._reset_all_camera_styles()
        
        final_system_state = 'normal'
        if 'danger' in self.camera_alert_states.values(): final_system_state = 'danger'
        elif 'warning' in self.camera_alert_states.values(): final_system_state = 'warning'
        
        if self.tmap_label.area2_state != final_system_state:
            self.tmap_label.set_area2_state(final_system_state)
    
                
    def _animate_warning_labels(self):
        self.is_blink_on = not self.is_blink_on
        for cam_name, state in self.camera_alert_states.items():
            try:
                label_widget = getattr(self.ui, cam_name)
                if self.is_blink_on:
                    if state == 'danger': color = "red"
                    else: color = "rgb(255, 140, 0)" 
                    label_widget.setStyleSheet(f"border: 5px solid {color};")
                else:
                    label_widget.setStyleSheet("border: 5px solid transparent;")
            except AttributeError: pass

    def _reset_camera_style(self, cam_name):
        try:
            label_widget = getattr(self.ui, cam_name)
            label_widget.setStyleSheet("border: 5px solid transparent;")
        except AttributeError: pass
            
    def _reset_all_camera_styles(self):
        target_cams = ['lblcam2', 'lblcam3', 'lblcam4']
        for cam in target_cams: self._reset_camera_style(cam)

    def save_settings_to_db(self, ui_setting, cam_name):
        try:
            cam_name_lower = cam_name.lower() 
            ip_address = getattr(ui_setting, f"txtipadd_{cam_name_lower}").toPlainText()
            is_person = getattr(ui_setting, f"chkperson_{cam_name_lower}").isChecked()
            fork_person = getattr(ui_setting, f"chkforkperson_{cam_name_lower}").isChecked()
            crane_person = getattr(ui_setting, f"chkcraneperson_{cam_name_lower}").isChecked()
            roi_display = getattr(ui_setting, f"chkroi_{cam_name_lower}").isChecked()
            roi_coords = getattr(ui_setting, f"txtroi_{cam_name_lower}").toPlainText()
            xdist = getattr(ui_setting, f"txt_xdist_{cam_name_lower}").toPlainText()
            ydist = getattr(ui_setting, f"txt_ydist_{cam_name_lower}").toPlainText()
            xref = getattr(ui_setting, f"txt_xref_{cam_name_lower}").toPlainText()
        except AttributeError as e: return

        line_p2p3_A, line_p2p3_B, line_p2p3_C = None, None, None
        line_p1p4_A, line_p1p4_B, line_p1p4_C = None, None, None

        if cam_name in ['CAM1', 'CAM2', 'CAM3', 'CAM4', 'CAM5']:
            line_p2_p3, line_p1_p4 = _calculate_roi_line_equations(roi_coords)
            if line_p2_p3 and line_p1_p4:
                line_p2p3_A, line_p2p3_B, line_p2p3_C = line_p2_p3
                line_p1p4_A, line_p1p4_B, line_p1p4_C = line_p1_p4
                point_23 = _calculate_point_at_y130(line_p2p3_A, line_p2p3_B, line_p2p3_C)
                point_14 = _calculate_point_at_y130(line_p1p4_A, line_p1p4_B, line_p1p4_C)
                self.cam_points_at_y130[cam_name]['point_p2p3'] = point_23
                self.cam_points_at_y130[cam_name]['point_p1p4'] = point_14
            else:
                self.cam_points_at_y130[cam_name]['point_p2p3'] = None
                self.cam_points_at_y130[cam_name]['point_p1p4'] = None

        conn = None
        try:
            conn = sqlite3.connect("setting_gy.db")
            cursor = conn.cursor()
            cursor.execute("""
                UPDATE Setting
                SET ai_ipadd = ?, ai_person = ?, ai_forkperson = ?, ai_craneperson = ?, 
                    ai_roi_display = ?, cam_roi_coords = ?,
                    X_dist = ?, Y_dist = ?, x_ref = ?,
                    line_p2p3_A = ?, line_p2p3_B = ?, line_p2p3_C = ?,
                    line_p1p4_A = ?, line_p1p4_B = ?, line_p1p4_C = ?
                WHERE cam_name = ?
            """, (ip_address, is_person, fork_person, crane_person, roi_display, roi_coords,
                  xdist, ydist, xref,
                  line_p2p3_A, line_p2p3_B, line_p2p3_C,
                  line_p1p4_A, line_p1p4_B, line_p1p4_C,
                  cam_name)) 
            conn.commit()
            
            try:
                self.db_cam_settings[cam_name] = {
                    'X_dist': float(xdist) if xdist else None,
                    'Y_dist': float(ydist) if ydist else None,
                    'x_ref': float(xref) if xref else None,
                    'cam_roi_coords_str': roi_coords,
                    'line_p2p3_A': line_p2p3_A, 'line_p2p3_B': line_p2p3_B, 'line_p2p3_C': line_p2p3_C,
                    'line_p1p4_A': line_p1p4_A, 'line_p1p4_B': line_p1p4_B, 'line_p1p4_C': line_p1p4_C
                }
            except Exception: pass
            
            label_name = f"lbl{cam_name_lower}" 
            info = self.camera_controller.labels.get(label_name)
            if info and info['worker']:
                info['worker'].set_yolo_state(is_person, fork_person, crane_person, False, roi_display)
                info['worker'].set_roi(roi_coords)
            
            QMessageBox.information(None, "저장 완료", f"{cam_name}의 설정이 저장되었습니다.")
        except sqlite3.Error as e:
            QMessageBox.critical(None, "데이터베이스 오류", f"데이터 저장 중 오류 발생: {e}")
        finally:
            if conn: conn.close()                
                
    def load_settings_from_db(self, ui_setting):
        conn = None
        try:
            conn = sqlite3.connect("setting_gy.db")
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM Setting")
            settings = cursor.fetchall() 
            # (Note: Assuming column order matches. Simplified for brevity as per existing logic)
            # Re-implementing specific select for safety
            cursor.execute("""
                SELECT cam_name, ai_ipadd, ai_person, ai_forkperson, ai_craneperson, 
                    ai_roi_display, cam_roi_coords, X_dist, Y_dist, x_ref,
                    line_p2p3_A, line_p2p3_B, line_p2p3_C,
                    line_p1p4_A, line_p1p4_B, line_p1p4_C
                FROM Setting
            """)
            settings = cursor.fetchall()
            settings_dict = {row[0]: row for row in settings}

            for i in range(1, 6):
                cam_name = f"CAM{i}"
                if cam_name in settings_dict:
                    data = settings_dict[cam_name]
                    cam_name_lower = cam_name.lower() 
                    try:
                        getattr(ui_setting, f"txtipadd_{cam_name_lower}").setText(data[1])
                        getattr(ui_setting, f"chkperson_{cam_name_lower}").setChecked(data[2] == 1)
                        getattr(ui_setting, f"chkforkperson_{cam_name_lower}").setChecked(data[3] == 1)
                        getattr(ui_setting, f"chkcraneperson_{cam_name_lower}").setChecked(data[4] == 1)
                        getattr(ui_setting, f"chkroi_{cam_name_lower}").setChecked(data[5] == 1)
                        roi_value = str(data[6]) if data[6] is not None else ""
                        getattr(ui_setting, f"txtroi_{cam_name_lower}").setText(roi_value)
                        
                        getattr(ui_setting, f"txt_xdist_{cam_name_lower}").setText(str(data[7]) if data[6] else "")
                        getattr(ui_setting, f"txt_ydist_{cam_name_lower}").setText(str(data[8]) if data[7] else "")
                        getattr(ui_setting, f"txt_xref_{cam_name_lower}").setText(str(data[9]) if data[8] else "")
                        
                        x_dist_val = float(data[7]) if data[7] else None
                        y_dist_val = float(data[8]) if data[8] else None
                        x_ref_val = float(data[9]) if data[9] else None
                        
                        self.db_cam_settings[cam_name] = {
                            'X_dist': x_dist_val, 'Y_dist': y_dist_val, 'x_ref': x_ref_val,
                            'cam_roi_coords_str': data[6], 
                            'line_p2p3_A': data[10], 'line_p2p3_B': data[11], 'line_p2p3_C': data[12],
                            'line_p1p4_A': data[13], 'line_p1p4_B': data[14], 'line_p1p4_C': data[15]
                        }
                        
                        if cam_name in ['CAM2', 'CAM4'] and data[9] is not None:
                            A1, B1, C1 = data[9], data[10], data[11] # Indices mismatch in logic? 
                            # Logic correction: Indices 10,11,12 are p2p3. 
                            # The original code logic was slightly confusing in mapping, using corrected indices from SELECT
                            A1, B1, C1 = data[10], data[11], data[12]
                            A2, B2, C2 = data[13], data[14], data[15]
                            point_23 = _calculate_point_at_y130(A1, B1, C1)
                            point_14 = _calculate_point_at_y130(A2, B2, C2)
                            self.cam_points_at_y130[cam_name]['point_p2p3'] = point_23
                            self.cam_points_at_y130[cam_name]['point_p1p4'] = point_14
                    except Exception: pass
        except sqlite3.Error as e:
            QMessageBox.critical(self, "Database Error", f"데이터베이스 오류: {e}")
        finally:
            if conn: conn.close()            

    def update_all_roi_checkbox_visibility(self):
        if not hasattr(self, 'ui_setting'): return
        for i in range(1, 6):
            cam_name = f"CAM{i}"
            try:
                txt_roi_widget = getattr(self.ui_setting, f"txtroi_{cam_name.lower()}")
                chk_roi_widget = getattr(self.ui_setting, f"chkroi_{cam_name.lower()}")
                if txt_roi_widget.toPlainText().strip():
                    chk_roi_widget.setEnabled(True)
                else:
                    chk_roi_widget.setEnabled(False)
            except AttributeError: pass
                
    def apply_db_settings_to_workers(self):
        conn = None
        try:
            conn = sqlite3.connect("setting_gy.db")
            cursor = conn.cursor()
            cursor.execute("""
                SELECT cam_name, ai_ipadd, ai_person, ai_forkperson, ai_craneperson, 
                    ai_roi_display, cam_roi_coords, X_dist, Y_dist, x_ref,
                    line_p2p3_A, line_p2p3_B, line_p2p3_C,
                    line_p1p4_A, line_p1p4_B, line_p1p4_C
                FROM Setting
            """)
            settings_list = cursor.fetchall()
            for row_data in settings_list:
                cam_name = row_data[0]
                if not cam_name: continue
                x_dist_val = float(row_data[7]) if row_data[7] else None
                y_dist_val = float(row_data[8]) if row_data[8] else None
                x_ref_val = float(row_data[9]) if row_data[9] else None

                self.db_cam_settings[cam_name] = {
                    'X_dist': x_dist_val, 'Y_dist': y_dist_val, 'x_ref': x_ref_val,
                    'cam_roi_coords_str': row_data[6],
                    'line_p2p3_A': row_data[10], 'line_p2p3_B': row_data[11], 'line_p2p3_C': row_data[12],
                    'line_p1p4_A': row_data[13], 'line_p1p4_B': row_data[14], 'line_p1p4_C': row_data[15],
                    '_person': bool(row_data[2]), '_fork_person': bool(row_data[3]), 
                    '_crane_person': bool(row_data[4]), '_roi_display': bool(row_data[5]), '_roi_coords': row_data[6]
                }

            for cam_label_name, info in self.camera_controller.labels.items():
                cam_name = f"CAM{cam_label_name[-1]}"
                if cam_name in self.db_cam_settings: 
                    settings = self.db_cam_settings[cam_name] 
                    worker = info['worker']
                    if worker:
                        worker.set_yolo_state(settings['_person'], settings['_fork_person'], settings['_crane_person'], False, settings['_roi_display'])
                        worker.set_roi(settings['_roi_coords'])
        except sqlite3.Error: pass
        finally:
            if conn: conn.close()

    def start_lidar_stream(self):
        if self.lidar_thread is not None and self.lidar_thread.isRunning(): return
        self.statusBar().showMessage("LiDAR 스트림을 시작합니다...")
        self.lidar_thread = QThread()
        self.lidar_worker = LidarWorker()
        self.lidar_worker.moveToThread(self.lidar_thread)
        self.lidar_thread.started.connect(self.lidar_worker.run)
        self.lidar_worker.frame_ready.connect(self.update_lidar_image)
        self.lidar_worker.finished.connect(self.lidar_thread.quit)
        self.lidar_worker.finished.connect(self.lidar_worker.deleteLater)
        self.lidar_thread.finished.connect(self.lidar_thread.deleteLater)
        self.lidar_worker.finished.connect(self.on_lidar_stopped)
        self.lidar_thread.finished.connect(lambda: setattr(self, 'lidar_thread', None))
        self.lidar_thread.finished.connect(lambda: setattr(self, 'lidar_worker', None))
        self.lidar_thread.start()
        self.ui.actionLidar.setText("Lidar ON")
        
    def toggle_lidar_stream(self):
        if self.lidar_thread is not None and self.lidar_thread.isRunning(): self.stop_lidar_stream()
        else: self.start_lidar_stream()

    def stop_lidar_stream(self):
        if self.lidar_worker: self.lidar_worker.stop() 
        if self.lidar_thread and self.lidar_thread.isRunning():
            self.lidar_thread.quit()
            if not self.lidar_thread.wait(3000): self.lidar_thread.terminate()
        
    def on_lidar_stopped(self):
        self.ui.actionLidar.setText("Lidar OFF")
        self.ui.lbllidar.clear() 

    def update_lidar_image(self, q_img):
        if q_img: self.ui.lbllidar.setPixmap(QPixmap.fromImage(q_img))

    def cleanup_on_quit(self):
        for dialog in list(self.enlarge_dialogs.values()): dialog.close()
        self.camera_controller.release_all()
        self.stop_lidar_stream()
        
    def _convert_cam_to_layout(self, cam_name, coords_list, params, obj_type=None):
        if not coords_list: return []
        layout_points = []
        roi_str = params.get('cam_roi_coords_str')
        try:
            roi_coords = ast.literal_eval(roi_str) if roi_str else None
        except: roi_coords = None
        if not roi_coords: return []

        try:
            src_pts = np.array(roi_coords, dtype=np.float32)
            if cam_name == 'CAM1':
                map_x, map_y, map_w, map_h = 100.0, 30.0, 199.0, 200.0
                dst_pts = np.array([[map_x+map_w, map_y], [map_x+map_w, map_y+map_h], [map_x, map_y+map_h], [map_x, map_y]], dtype=np.float32)
            elif cam_name == 'CAM5':
                map_x, map_y, map_w, map_h = 695.0, 30.0, 265.0, 200.0
                dst_pts = np.array([[map_x, map_y+map_h], [map_x, map_y], [map_x+map_w, map_y], [map_x+map_w, map_y+map_h]], dtype=np.float32)
            else:
                X_dist = params.get('X_dist')
                Y_dist = params.get('Y_dist')
                x_ref = params.get('x_ref')
                if not (X_dist and Y_dist and x_ref): return []
                map_x = 100 + (X_dist * 10)
                map_y = 30
                map_w = x_ref * 10
                map_h = Y_dist * 10
                dst_pts = np.array([[map_x, map_y], [map_x+map_w, map_y], [map_x+map_w, map_y+map_h], [map_x, map_y+map_h]], dtype=np.float32)
            
            if dst_pts is None: return []
            M = cv2.getPerspectiveTransform(src_pts, dst_pts)
            valid_min_x, valid_max_x = map_x, map_x + map_w
            valid_min_y, valid_max_y = map_y, map_y + map_h             
            
            for item in coords_list:
                if obj_type == 'hook' or (obj_type is None and len(item) == 2 and isinstance(item[0], (list, tuple)) and len(item[0]) == 4):
                    (cx, cy, w, h) = item[0]
                    # 1. 원본 좌표(카메라 영상 기준)를 배열 형태로 준비
                    pt_arr = np.array([[float(cx), float(cy)]], dtype=np.float32).reshape(-1, 1, 2)

                    # 2. ★ 핵심 지점: 변환 매트릭스(M)를 적용하여 맵 좌표로 투영
                    transformed_pt = cv2.perspectiveTransform(pt_arr, M)

                    # 3. 최종 좌표 추출
                    lx = transformed_pt[0][0][0]
                    ly = transformed_pt[0][0][1]
                    
                    # lx 수정값
                    # if cam_name == 'CAM1':
                    #     lx = int(19.91 * lx / 8.8)+
                    
                    if cam_name in ['CAM1', 'CAM5']:
                        lx = np.clip(lx, valid_min_x, valid_max_x)
                        ly = np.clip(ly, valid_min_y, valid_max_y)
                    elif cam_name == 'CAM2': lx = np.clip(lx, 190, 465)
                    elif cam_name == 'CAM4': lx = np.clip(lx, 500, 800)
                    if cam_name not in ['CAM1', 'CAM5']: ly = np.clip(ly, 30, 230)
                    layout_points.append(((int(lx), int(ly)), item))

                elif obj_type == 'truck' or (obj_type is None and len(item) == 4):
                    cx, cy, w, h = item
                    pt_arr = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
                    dst_pt = cv2.perspectiveTransform(pt_arr, M)
                    lx, ly = dst_pt[0][0][0], dst_pt[0][0][1]
                    pt_right_arr = np.array([[[float(cx + w/2), float(cy)]]], dtype=np.float32)
                    rx = cv2.perspectiveTransform(pt_right_arr, M)[0][0][0]
                    pt_top_arr = np.array([[[float(cx), float(cy - h)]]], dtype=np.float32)
                    ty = cv2.perspectiveTransform(pt_top_arr, M)[0][0][1]
                    map_w_val = abs(rx - lx) * 2
                    map_h_val = abs(ly - ty)
                    if cam_name in ['CAM1', 'CAM5']:
                        lx = np.clip(lx, valid_min_x, valid_max_x)
                        ly = np.clip(ly, valid_min_y, valid_max_y)
                    elif cam_name == 'CAM2': lx = np.clip(lx, 190, 465)
                    elif cam_name == 'CAM4': lx = np.clip(lx, 500, 800)
                    if cam_name not in ['CAM1', 'CAM5']: ly = np.clip(ly, 30, 230)
                    layout_points.append(((int(lx), int(ly), int(map_w_val), int(map_h_val)), item))

                elif obj_type == 'person' or obj_type is None:
                    if len(item) == 4: 
                        cx, cy, w, h = item  # _, _ 대신 w, h 사용
                    elif len(item) == 2: 
                        cx, cy = item
                        w = 0; h = 0 # 없을 경우 0으로 초기화
                    else: 
                        continue
                    
                    # [1] 바닥점(Feet) 변환 -> Map 상의 위치(lx, ly)
                    pt_arr = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
                    dst_pt = cv2.perspectiveTransform(pt_arr, M)
                    lx, ly = dst_pt[0][0][0], dst_pt[0][0][1]
                    
                    # [2] 정수리점(Top) 변환 -> Map 상의 높이(map_h) 계산
                    # (트럭/후크와 동일한 로직 적용)
                    if h > 0:
                        pt_top_arr = np.array([[[float(cx), float(cy - h)]]], dtype=np.float32)
                        dst_top = cv2.perspectiveTransform(pt_top_arr, M)
                        ty = dst_top[0][0][1]
                        map_h_val = abs(ly - ty)
                        map_w_val = map_h_val * 0.5  # 사람은 너비가 좁으므로 비율 임의 설정 (또는 계산)
                    else:
                        map_h_val = 20 # 기본값
                        map_w_val = 20

                    # [클리핑 로직]
                    if cam_name in ['CAM1', 'CAM5']:
                        lx = np.clip(lx, valid_min_x, valid_max_x)
                        ly = np.clip(ly, valid_min_y, valid_max_y)
                    elif cam_name == 'CAM2': lx = np.clip(lx, 190, 465)
                    elif cam_name == 'CAM3': lx = np.clip(lx, 370, 625)
                    elif cam_name == 'CAM4': lx = np.clip(lx, 500, 800)
                    if cam_name not in ['CAM1', 'CAM5']: ly = np.clip(ly, 30, 230)
                    
                    # [수정 결과] (lx, ly) 뿐만 아니라 크기 정보도 함께 반환
                    # 기존: layout_points.append(((int(lx), int(ly)), item))
                    layout_points.append(((int(lx), int(ly), int(map_w_val), int(map_h_val)), item))
                    
        except Exception: pass
        return layout_points

    def handle_objects_detection(self, cam_label_name: str, data_dict: dict):
        cam_name = cam_label_name.replace('lbl', '').upper()
        params = self.db_cam_settings.get(cam_name)
        if not params: return
        
        def get_line_eq(p_start, p_end):
            x1, y1 = p_start; x2, y2 = p_end
            return (y2 - y1), (x1 - x2), (x2 * y1 - x1 * y2)
        def is_right_of_line(cx, cy, A, B, C): return True if A==0 else cx > (-B * cy - C) / A
        def is_left_of_line(cx, cy, A, B, C): return True if A==0 else cx < (-B * cy - C) / A
        def is_below_line(cx, cy, A, B, C): return True if B==0 else cy > (-A * cx - C) / B
        def get_pixel_dist(p1, p2): return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) ** 0.5

        original_raw_trucks = data_dict.get('truck', [])
        
        # [카운터 로직은 유지: 모든 카메라에 대해 횟수는 셈]
        if cam_name not in self.truck_persistence_counter: self.truck_persistence_counter[cam_name] = 0
        if len(original_raw_trucks) > 0: self.truck_persistence_counter[cam_name] += 1
        else: self.truck_persistence_counter[cam_name] = 0
        
        # ▼▼▼ [수정된 부분] CAM1, CAM5에만 15회 인식 제한을 적용 ▼▼▼
        if cam_name in ['CAM1', 'CAM5']:
            DETECTION_THRESHOLD = 15
            if self.truck_persistence_counter[cam_name] < DETECTION_THRESHOLD: 
                original_raw_trucks = []
        
        has_truck_detection = len(original_raw_trucks) > 0
        raw_trucks = list(original_raw_trucks) 
        if cam_name == 'CAM3': raw_trucks = []
        
        # [수정 2] 트럭이 감지되지 않으면 EMA 초기화 (새로운 트럭 진입 대비)
        if cam_name == 'CAM1' and len(raw_trucks) == 0:
            self.cam1_height_ema = None
            self.shared_truck_height = None  # 트럭이 없으면 공유 값도 초기화

        if cam_name == 'CAM1':
            roi_str = params.get('cam_roi_coords_str')
            x_ref = params.get('x_ref')  
            if raw_trucks and roi_str and x_ref:
                try:
                    roi_coords = ast.literal_eval(roi_str)
                    if isinstance(roi_coords, list) and len(roi_coords) == 4:
                        p1, p2, p3, p4 = roi_coords
                        roi_w_px = get_pixel_dist(p1, p2)
                        best_truck = max(raw_trucks, key=lambda x: x[2])
                        truck_w_px = best_truck[2]
                        if roi_w_px > 0:
                            ratio = truck_w_px / roi_w_px
                            truck_len_m = ratio * float(x_ref) 
                            calculated_map_h = truck_len_m * 10.0 
                            
                            if 1.0 < calculated_map_h < 600.0:
                                # ▼▼▼ [수정 2 핵심] 급격한 길이 변화(가림) 필터링 로직 ▼▼▼
                                
                                # 1. 첫 측정인 경우
                                if self.cam1_height_ema is None:
                                    self.cam1_height_ema = calculated_map_h
                                    self.shared_truck_height = calculated_map_h
                                    
                                else:
                                    # 2. 길이가 커지거나 비슷함 (정상 진입 혹은 인식 개선) -> 반영
                                    if calculated_map_h >= (self.cam1_height_ema * 0.95):
                                        alpha = 0.3 # 반영 비율 (빠르게 반영)
                                        self.cam1_height_ema = (self.cam1_height_ema * (1 - alpha)) + (calculated_map_h * alpha)
                                        self.shared_truck_height = self.cam1_height_ema
                                        
                                    # 3. 길이가 급격히 작아짐 (평균의 80% 미만) -> 가림 현상(Occlusion)으로 판단
                                    elif calculated_map_h < (self.cam1_height_ema * 0.8):
                                        # CAM1 데이터 신뢰 불가 -> 공유 값을 None으로 설정
                                        # 이렇게 하면 CAM2 로직에서 self.shared_truck_height가 None이 되어
                                        # CAM2 자신의 map_h 데이터를 사용하게 됨
                                        self.shared_truck_height = None
                                        # 주의: EMA는 갱신하지 않음 (가려진 잘못된 값이므로)
                                        
                                    # 4. 완만한 감소 (80% ~ 95% 사이) -> 정상적인 오차나 미세 조정으로 반영
                                    else:
                                        alpha = 0.1 # 감소는 천천히 반영 (떨림 방지)
                                        self.cam1_height_ema = (self.cam1_height_ema * (1 - alpha)) + (calculated_map_h * alpha)
                                        self.shared_truck_height = self.cam1_height_ema
                except: pass
                
        # ▼▼▼ [수정] CAM5 리셋 로직 추가 (트럭이 사라지면 EMA 초기화) ▼▼▼
        if cam_name == 'CAM5' and len(raw_trucks) == 0:
            self.cam5_height_ema = None
            self.shared_truck_height_cam5 = None  # 트럭이 없으면 공유 값도 초기화                
                            
        if cam_name == 'CAM5':
            roi_str = params.get('cam_roi_coords_str')
            x_ref = params.get('x_ref') 
            if raw_trucks and roi_str and x_ref:
                try:
                     roi_coords = ast.literal_eval(roi_str)
                     if isinstance(roi_coords, list) and len(roi_coords) == 4:
                        p1, p2, p3, p4 = roi_coords
                        roi_w_px = get_pixel_dist(p1, p2)
                        best_truck = max(raw_trucks, key=lambda x: x[2])
                        truck_w_px = best_truck[2]
                        if roi_w_px > 0:
                            ratio = truck_w_px / roi_w_px
                            truck_len_m = ratio * float(x_ref)
                            calculated_map_h = truck_len_m * 10.0
                            
                            # ▼▼▼ 신규: EMA 및 가림 필터 적용 ▼▼▼
                            if 1.0 < calculated_map_h < 600.0:
                                # 1. 첫 측정
                                if self.cam5_height_ema is None:
                                    self.cam5_height_ema = calculated_map_h
                                    self.shared_truck_height_cam5 = calculated_map_h
                                
                                else:
                                    # 2. 정상 범위 (커지거나 비슷함) -> 빠른 반영
                                    if calculated_map_h >= (self.cam5_height_ema * 0.95):
                                        alpha = 0.3 
                                        self.cam5_height_ema = (self.cam5_height_ema * (1 - alpha)) + (calculated_map_h * alpha)
                                        self.shared_truck_height_cam5 = self.cam5_height_ema
                                    
                                    # 3. 급격한 감소 (가림 현상, 80% 미만) -> 공유 데이터 무효화
                                    elif calculated_map_h < (self.cam5_height_ema * 0.8):
                                        self.shared_truck_height_cam5 = None 
                                        # CAM4는 shared 값이 None이면 자신의 map_h를 사용함
                                    
                                    # 4. 완만한 감소 -> 느린 반영
                                    else:
                                        alpha = 0.1
                                        self.cam5_height_ema = (self.cam5_height_ema * (1 - alpha)) + (calculated_map_h * alpha)
                                        self.shared_truck_height_cam5 = self.cam5_height_ema
                except: pass

        # 트럭 좌표 변환 및 "Map 밖 표시" 로직 적용
        if cam_name in ['CAM1', 'CAM5']: raw_trucks = []
        
        trucks_converted = self._convert_cam_to_layout(cam_name, raw_trucks, params, obj_type='truck')
        final_truck_pts = []
        truck_cache_data = []
        
        # [추가] CAM2, CAM4의 경우 P1-P2(상단) 라인 방정식을 구함 (라인 위쪽 판단용)
        line_A, line_B, line_C = None, None, None
        if cam_name in ['CAM2', 'CAM4']:
            roi_str = params.get('cam_roi_coords_str')
            if roi_str:
                try:
                    p_list = ast.literal_eval(roi_str)
                    if len(p_list) == 4:
                         # P1-P2 라인 계산 (ROI의 상단 선)
                         line_A, line_B, line_C = _get_line_equation(p_list[0], p_list[1])
                except: pass
        
        for item in trucks_converted:
            (lx, ly, map_w, map_h), original = item
            
            # 1. "Map 밖(라인 위)" 인지 판단
            is_outside_above = False
            if line_A is not None: # CAM2, CAM4이고 라인 방정식이 유효할 때
                raw_cx, raw_lower_y, _, _ = original
                # 직선 방정식 기준 위쪽인지 확인 (y < line_y)
                if line_B != 0:
                    line_y_at_x = (-line_C - line_A * raw_cx) / line_B
                    # 이미지 좌표계는 아래로 갈수록 Y가 커지므로, 작으면 '위'에 있는 것
                    if raw_lower_y < line_y_at_x:
                        is_outside_above = True

            # 2. 좌표 및 크기 결정
            if is_outside_above:
                # [변경] 라인 위에 있으면: Map 상단(30)보다 위(10)에, 작은 정사각형(20x20)으로 표시
                final_x = lx
                final_y = 20  # Map 시작(30) 바로 위
                final_w_val = 20
                final_h_val = 20
            else:
                # [기존] 정상적인 Map 내부 표시
                final_x = lx
                
                # ▼▼▼ [중요] 변수 추출을 먼저 수행해야 아래 조건문에서 사용 가능합니다. ▼▼▼
                raw_cx, raw_lower_y, raw_w, raw_h = original
                
                # ▼▼▼ [수정] CAM2, CAM4 트럭이 이미지 하단(입구)에 걸쳤을 때 Map 하단 고정 ▼▼▼
                raw_cx, raw_lower_y, raw_w, raw_h = original
                
                IMG_HEIGHT = 720   # 실제 영상 높이에 맞게 수정(resized 영상 기준)
                BOTTOM_MARGIN = 30 # 여유 범위
                
                # 조건: CAM2 또는 CAM4 이면서, bbox 밑면이 이미지 하단 3픽셀 이내(>=1077)인 경우
                if cam_name in ['CAM2', 'CAM4'] and (raw_lower_y >= IMG_HEIGHT - BOTTOM_MARGIN):
                    final_y = 230  # Map의 y축 하단 한계값 (클리핑 최대값)
                else:
                    final_y = ly   # 변환된 좌표 그대로 사용
                
                final_w_val = map_w
                
                # ---------------------------------------------------------
                # 높이(Height) 결정 로직
                # ---------------------------------------------------------
                raw_top_y = raw_lower_y - raw_h 
                
                is_crossing_top = False
                if line_A is not None and line_B != 0:
                     line_y_at_x = (-line_C - line_A * raw_cx) / line_B
                     if raw_top_y < line_y_at_x:
                         is_crossing_top = True
                         
                # # ▼▼▼ [신규 로직] 트럭 상단이 ROI 라인에 걸쳤는지(진입 중) 판단 ▼▼▼
                # raw_cx, raw_lower_y, raw_w, raw_h = original
                # raw_top_y = raw_lower_y - raw_h # 원본 BBox의 상단 Y좌표
                
                # is_crossing_top = False
                # if line_A is not None and line_B != 0:
                #      # 현재 X위치에서의 ROI 라인 Y값 계산
                #      line_y_at_x = (-line_C - line_A * raw_cx) / line_B
                #      # 트럭의 머리(Top)가 라인보다 위에 있으면(값이 작으면) 걸쳐있는 상태
                #      if raw_top_y < line_y_at_x:
                #          is_crossing_top = True
                
                # [Case A] 트럭이 ROI 경계에 걸쳐서 진입 중인 경우
                # 사용자의 요청: 바닥(final_y)은 그대로 두고, 높이를 Map 상단(30)까지 연결
                if is_crossing_top:
                    # 높이 = (보정된 바닥 위치 final_y) - (Map 상단 30)
                    calculated_h = final_y - 30
                    final_h_val = max(calculated_h, 20) # 최소 크기 안전장치

                # [Case B] 완전히 들어온 경우 (기존 로직: 공유 높이 적용)
                else:
                    final_h_val = map_h # 기본값
                    
                    # CAM2 처리
                    if cam_name == 'CAM2':
                        # ▼▼▼ [수정] 트럭이 2대 이상이면 CAM1 공유 값 무시하고 본인 감지 값 사용 ▼▼▼
                        if len(trucks_converted) >= 2:
                            final_h_val = map_h
                            
                        # ▼▼▼ 트럭이 1대(또는 0대)일 때는 기존 로직 유지 (CAM1 공유 값 우선) ▼▼▼
                        elif self.shared_truck_height is not None: 
                            full_len = self.shared_truck_height
                            # 75% 이상 크기면 전체 길이 적용 (Ghost Truck 방지)
                            if map_h >= full_len * 0.75:
                                final_h_val = full_len
                            else:
                                final_h_val = map_h
                        else:
                            final_h_val = max(map_h, 80)

                    # CAM4 처리
                    elif cam_name == 'CAM4':
                        # ▼▼▼ [수정] 트럭이 2대 이상이면 CAM5 공유 값 무시하고 본인 감지 값 사용 ▼▼▼
                        if len(trucks_converted) >= 2:
                            final_h_val = map_h

                        # ▼▼▼ 트럭이 1대(또는 0대)일 때는 기존 로직 유지 (CAM5 공유 값 우선) ▼▼▼
                        elif self.shared_truck_height_cam5 is not None: 
                            full_len = self.shared_truck_height_cam5
                            # 75% 이상 크기면 전체 길이 적용 (Ghost Truck 방지)
                            if map_h >= full_len * 0.75:
                                final_h_val = full_len
                            else:
                                final_h_val = map_h
                        else:
                            final_h_val = max(map_h, 80)

            final_truck_pts.append((final_x, final_y, final_w_val, final_h_val))
            truck_cache_data.append(((final_x, final_y, final_w_val, final_h_val), original))

        if cam_name == 'CAM1':
            cam2_cache = self.truck_layout_points_cache.get('lblcam2', [])
            cam2_has_truck = len(cam2_cache) > 0
            is_valid_trigger = False
            roi_str = params.get('cam_roi_coords_str')
            if has_truck_detection and roi_str:
                try:
                    roi_pts = ast.literal_eval(roi_str)
                    if len(roi_pts) == 4:
                        P1, P2, P3, P4 = roi_pts
                        A_right, B_right, C_right = get_line_eq(P2, P3)
                        A_left, B_left, C_left = get_line_eq(P1, P4)
                        for t_item in original_raw_trucks:
                            cx, lower_y, w, h = t_item
                            center_y = lower_y - (h / 2.0)
                            if is_left_of_line(cx, center_y, A_right, B_right, C_right) and \
                               is_right_of_line(cx, center_y, A_left, B_left, C_left) and \
                               (center_y > 240):
                                is_valid_trigger = True
                                break 
                except Exception: pass
                
        if cam_name == 'CAM5':
            cam4_cache = self.truck_layout_points_cache.get('lblcam4', [])
            cam4_has_truck = len(cam4_cache) > 0
            is_valid_trigger = False
            roi_str = params.get('cam_roi_coords_str')
            if has_truck_detection and roi_str:
                try:
                    roi_pts = ast.literal_eval(roi_str)
                    if len(roi_pts) == 4:
                        P1, P2, P3, P4 = roi_pts
                        A_right, B_right, C_right = get_line_eq(P2, P3)
                        A_left, B_left, C_left = get_line_eq(P1, P4)
                        for t_item in original_raw_trucks:
                            cx, lower_y, w, h = t_item
                            center_y = lower_y - (h / 2.0)
                            if is_left_of_line(cx, center_y, A_right, B_right, C_right) and \
                               is_right_of_line(cx, center_y, A_left, B_left, C_left) and \
                               (center_y > 240):
                                is_valid_trigger = True
                                break 
                except Exception: pass

        self.tmap_label.set_truck_positions_for_cam(cam_label_name, final_truck_pts)
        self.truck_layout_points_cache[cam_label_name] = truck_cache_data
        
        if cam_name == 'CAM2' and len(final_truck_pts) > 0: self.tmap_label.set_truck_positions_for_cam('lblcam1', [])
        if cam_name == 'CAM4' and len(final_truck_pts) > 0: self.tmap_label.set_truck_positions_for_cam('lblcam5', [])
        
        raw_hooks = data_dict.get('hook', [])
        # ▼▼▼ [추가] CAM2,3,4: 화면 양 끝단에 닿은 후크 제거 (중복 방지) ▼▼▼
        if cam_name in ['CAM2', 'CAM3', 'CAM4']:
            filtered_hooks = []
            IMG_WIDTH = 1280
            EDGE_MARGIN = 5 # 가장자리 여유 픽셀 (5px 이내면 닿은 것으로 간주)
            
            for item in raw_hooks:
                # 데이터 구조 파싱: ((cx, cy, w, h), angle) 또는 (cx, cy, w, h)
                cx, w = None, None
                
                # 1. ((cx, cy, w, h), angle) 형태
                if len(item) == 2 and isinstance(item[0], (list, tuple)) and len(item[0]) == 4:
                    cx, _, w, _ = item[0]
                # 2. (cx, cy, w, h) 형태
                elif len(item) == 4 and isinstance(item[0], (int, float)):
                    cx, _, w, _ = item
                
                if cx is not None and w is not None:
                    x1 = cx - (w / 2) # 좌측 끝
                    x2 = cx + (w / 2) # 우측 끝
                    
                    # 왼쪽 끝(0)이나 오른쪽 끝(1280)에 닿으면 제외
                    if x1 <= EDGE_MARGIN or x2 >= (IMG_WIDTH - EDGE_MARGIN):
                        continue # 필터링 (Skip)

                filtered_hooks.append(item)
            raw_hooks = filtered_hooks
            
        current_time = time.time()
        
        if not hasattr(self, 'hook_display_cache'): self.hook_display_cache = {}
        if cam_name not in self.hook_display_cache: self.hook_display_cache[cam_name] = {'last_time': 0.0, 'angles': []}
        cache = self.hook_display_cache[cam_name]
        
        if not cache['angles'] and len(raw_hooks) > 0: cache['last_time'] = current_time - 1.1
        if current_time - cache['last_time'] >= 1.0:
            if len(raw_hooks) > 0:
                new_snapshots = []
                for item in raw_hooks:
                    if len(item) == 2 and isinstance(item[1], (int, float)): new_snapshots.append(item[1]) 
                    else: new_snapshots.append(0.0)
                cache['angles'] = new_snapshots
                cache['last_time'] = current_time 
            
        final_hooks = []
        for i, item in enumerate(raw_hooks):
            coords = item[0]; display_angle = 0.0
            if i < len(cache['angles']): display_angle = cache['angles'][i]
            elif len(cache['angles']) > 0: display_angle = cache['angles'][-1]
            else: display_angle = item[1]
            final_hooks.append((coords, display_angle))
        raw_hooks = final_hooks 

        if cam_name == 'CAM1':
            roi_str = params.get('cam_roi_coords_str')
            if roi_str:
                try:
                    roi_pts = ast.literal_eval(roi_str)
                    if len(roi_pts) == 4:
                        P1, P2, P3, P4 = roi_pts
                        A1, B1, C1 = get_line_eq(P1, P4)
                        A2, B2, C2 = get_line_eq(P1, P2)
                        filtered_hooks = []
                        for h_item in raw_hooks:
                            coords = h_item[0]
                            cx, cy = coords[0], coords[1]
                            if is_right_of_line(cx, cy, A1, B1, C1) and is_below_line(cx, cy, A2, B2, C2):
                                filtered_hooks.append(h_item)
                        raw_hooks = filtered_hooks
                except: pass

        if cam_name == 'CAM5':
            roi_str = params.get('cam_roi_coords_str')
            if roi_str:
                try:
                    roi_pts = ast.literal_eval(roi_str)
                    if len(roi_pts) == 4:
                        P1, P2, P3, P4 = roi_pts
                        A_top, B_top, C_top = get_line_eq(P1, P2)
                        A_right, B_right, C_right = get_line_eq(P2, P3)
                        filtered_hooks = []
                        for h_item in raw_hooks:
                            coords = h_item[0]
                            cx, cy = coords[0], coords[1]
                            if is_below_line(cx, cy, A_top, B_top, C_top) and is_left_of_line(cx, cy, A_right, B_right, C_right):
                                filtered_hooks.append(h_item)
                        raw_hooks = filtered_hooks
                except: pass

        hooks_converted = self._convert_cam_to_layout(cam_name, raw_hooks, params, obj_type='hook')
        self.hook_layout_points_cache[cam_label_name] = hooks_converted
        if cam_name == 'CAM1':
            #  cam_hook_pts = [h[0] for h in hooks_converted]
            # [수정 후] 각도 정보가 포함된 전체 데이터(hooks_converted)를 그대로 전달
             self.tmap_label.set_hook_positions_for_cam(cam_label_name, hooks_converted)
            
        raw_people = data_dict.get('person', [])
        # ▼▼▼ [추가] CAM2,3,4: 화면 양 끝단에 닿은 사람 제거 (중복 방지) ▼▼▼
        if cam_name in ['CAM2', 'CAM3', 'CAM4']:
            filtered_people = []
            IMG_WIDTH = 1280
            EDGE_MARGIN = 20 
            
            for p in raw_people:
                cx, w = None, None
                
                # (cx, cy, w, h) 형태
                if len(p) == 4:
                    cx, _, w, _ = p
                # (cx, cy) 형태는 너비 정보가 없어 필터링 불가 -> 유지
                elif len(p) == 2:
                    filtered_people.append(p)
                    continue
                
                if cx is not None and w is not None:
                    x1 = cx - (w / 2)
                    x2 = cx + (w / 2)
                    
                    # 왼쪽 끝이나 오른쪽 끝에 닿으면 제외
                    if x1 <= EDGE_MARGIN or x2 >= (IMG_WIDTH - EDGE_MARGIN):
                        continue 

                filtered_people.append(p)
            raw_people = filtered_people
        
        if cam_name == 'CAM1':
            roi_str = params.get('cam_roi_coords_str')
            if roi_str:
                try:
                    roi_pts = ast.literal_eval(roi_str)
                    if len(roi_pts) == 4:
                        P1, P2, P3, P4 = roi_pts
                        A1, B1, C1 = get_line_eq(P1, P4)
                        A2, B2, C2 = get_line_eq(P1, P2)
                        filtered_people = []
                        for p in raw_people:
                            if len(p) == 4: cx, cy, w, h = p
                            elif len(p) == 2: cx, cy = p; w=h=0
                            else: continue
                            if is_right_of_line(cx, cy, A1, B1, C1) and is_below_line(cx, cy, A2, B2, C2):
                                filtered_people.append(p)
                        raw_people = filtered_people
                except: pass

        if cam_name == 'CAM5':
            roi_str = params.get('cam_roi_coords_str')
            if roi_str:
                try:
                    roi_pts = ast.literal_eval(roi_str)
                    if len(roi_pts) == 4:
                        P1, P2, P3, P4 = roi_pts
                        A_top, B_top, C_top = get_line_eq(P1, P2)
                        A_right, B_right, C_right = get_line_eq(P2, P3)
                        filtered_people = []
                        for p in raw_people:
                            if len(p) == 4: cx, cy, w, h = p
                            elif len(p) == 2: cx, cy = p; w=h=0
                            else: continue
                            if is_below_line(cx, cy, A_top, B_top, C_top) and is_left_of_line(cx, cy, A_right, B_right, C_right):
                                filtered_people.append(p)
                        raw_people = filtered_people
                except: pass

        people_converted = self._convert_cam_to_layout(cam_name, raw_people, params, obj_type='person')
        
        if cam_name in ['CAM2', 'CAM4'] and truck_cache_data:
            try:
                t_map, t_raw = truck_cache_data[0]
                t_mlx, t_mly, t_mw, t_mh = t_map          
                t_rcx, t_rcy, t_rw, t_rh = t_raw          
                t_x1 = t_rcx - t_rw / 2; t_x2 = t_rcx + t_rw / 2
                t_y1 = t_rcy - t_rh; t_y2 = t_rcy         
                
                # ▼▼▼ [수정] 트럭 위 객체 보정 함수 (데이터 구조 대응) ▼▼▼
                def adjust_pos_to_truck(obj_list):
                    for i, item in enumerate(obj_list):
                        # [문제 해결] item[0]이 (x,y)일수도 있고 (x,y,w,h)일수도 있음 -> 유연하게 처리
                        map_data, raw_obj = item
                        
                        map_x, map_y, map_w, map_h = 0, 0, 20, 20 # 기본값
                        
                        # 4개짜리 데이터(w,h 포함)인지 2개짜리인지 확인
                        if len(map_data) == 4:
                            map_x, map_y, map_w, map_h = map_data
                        elif len(map_data) == 2:
                            map_x, map_y = map_data
                        else: continue

                        # Raw 데이터 좌표 파싱
                        ocx, ocy = 0, 0
                        if isinstance(raw_obj, (list, tuple)):
                             first_elem = raw_obj[0] if isinstance(raw_obj[0], (list, tuple)) else raw_obj
                             if len(first_elem) >= 2: ocx, ocy = first_elem[0], first_elem[1]
                             else: continue
                        else: continue
                        
                        # 트럭 범위 안에 있는지 확인
                        margin = 10
                        if (t_x1 - margin <= ocx <= t_x2 + margin) and (t_y1 - margin <= ocy <= t_y2 + margin):
                               # 비율대로 위치 보정
                               ratio_x = (ocx - t_x1) / t_rw
                               ratio_y = (ocy - t_y1) / t_rh 
                               new_map_x = (t_mlx - t_mw / 2) + (ratio_x * t_mw)
                               new_map_y = (t_mly - t_mh) + (ratio_y * t_mh)
                               
                               # [수정] 값을 다시 저장할 때, 원래 데이터 구조(4개 혹은 2개)를 유지해야 함
                               if len(map_data) == 4:
                                   obj_list[i] = ((int(new_map_x), int(new_map_y), map_w, map_h), raw_obj)
                               else:
                                   obj_list[i] = ((int(new_map_x), int(new_map_y)), raw_obj)
                               
                adjust_pos_to_truck(hooks_converted)
                adjust_pos_to_truck(people_converted)
                
            except Exception as e:
                # 에러 내용을 출력하면 디버깅에 도움이 됩니다.
                print(f"Truck adjustment error: {e}")
        
        self.layout_points_cache[cam_label_name] = people_converted
        if cam_name == 'CAM1':
             cam_pts = [p[0] for p in people_converted]
             self.tmap_label.set_people_positions_for_cam(cam_label_name, cam_pts)
             self.update_safety_status()

        if cam_name in ['CAM2', 'CAM3', 'CAM4', 'CAM5']:
            self._update_map_and_safety()
    
    def show_enlarged_lidar(self):
        LIDAR_SOURCE_ID = "LIDAR_STREAM" 
        if self.lidar_worker is None or not self.lidar_worker.is_running:
            QMessageBox.warning(self, "Lidar", "Lidar 스트림이 아직 실행 중이 아닙니다.")
            return
        if LIDAR_SOURCE_ID in self.enlarge_dialogs and self.enlarge_dialogs[LIDAR_SOURCE_ID].isVisible():
            self.enlarge_dialogs[LIDAR_SOURCE_ID].raise_()
            self.enlarge_dialogs[LIDAR_SOURCE_ID].activateWindow()
            return
        for dialog in list(self.camera_controller.enlarge_dialogs.values()): dialog.close()
        for dialog in list(self.enlarge_dialogs.values()): dialog.close()
        enlarge_dialog = EnlargeDialog(parent=self, image_source=LIDAR_SOURCE_ID, shared_worker=self.lidar_worker, lidar_callback_data=self.lidar_worker.callback_data)
        main_window_center = self.frameGeometry().center()
        dialog_geometry = enlarge_dialog.frameGeometry()
        dialog_geometry.moveCenter(main_window_center)
        enlarge_dialog.move(dialog_geometry.topLeft())
        enlarge_dialog.finished.connect(lambda: self.enlarge_dialogs.pop(LIDAR_SOURCE_ID, None))
        enlarge_dialog.show()
        self.enlarge_dialogs[LIDAR_SOURCE_ID] = enlarge_dialog 

    def 종료(self):
        self.app.quit() 

    def about(self):
        QMessageBox.about(self, "", "메인윈도우입니다.")
        
    
    def _update_map_and_safety(self):
        # 1. 데이터 가져오기
        # ▼▼▼ [추가] CAM1 데이터도 가져옵니다 (비교 대상)
        h_pairs_cam1 = self.hook_layout_points_cache.get('lblcam1', [])
        p_pairs_cam1 = self.layout_points_cache.get('lblcam1', [])
        
        h_pairs_cam2 = self.hook_layout_points_cache.get('lblcam2', [])
        h_pairs_cam3 = self.hook_layout_points_cache.get('lblcam3', [])
        h_pairs_cam4 = self.hook_layout_points_cache.get('lblcam4', [])
        h_pairs_cam5 = self.hook_layout_points_cache.get('lblcam5', [])
        
        final_hooks_cam2 = h_pairs_cam2
        final_hooks_cam4 = h_pairs_cam4
        final_hooks_cam3 = []
        final_hooks_cam5 = []

        # 2. 상수 설정
        HOOK_DUP_DIST = 80.0  
        HOOK_DUP_IOU = 0.05
        HOOK_BOX_SIZE = 30
        
        # ▼▼▼ [신규] CAM2 좌측(CAM1 중복) 필터링 상수
        CAM2_LEFT_LIMIT = 120  # 0~100px 구간에 여유를 두어 120px로 설정
        
        # -------------------------------------------------------------
        # [HOOK] CAM2 필터링 (CAM1과 중복 시 CAM1 우선) - 신규 추가됨
        # -------------------------------------------------------------
        filtered_cam2_hooks = []
        for pair2 in final_hooks_cam2:
            # pair2 구조: ((lx, ly), raw_data)
            # raw_data 구조가 ((cx, cy, w, h), angle) 형태일 수 있음
            try:
                raw_data = pair2[1]
                raw_cx = 0
                
                # 데이터 구조 파싱 (tuple 안에 tuple이 있는지 확인)
                if isinstance(raw_data, (list, tuple)):
                    if isinstance(raw_data[0], (list, tuple)): # ((cx,cy,w,h), angle)
                        raw_cx = raw_data[0][0]
                    else: # (cx,cy,w,h)
                        raw_cx = raw_data[0]
                
                # 조건 1: CAM2 화면상 왼쪽 구석(0~120px)에 있는가?
                if raw_cx < CAM2_LEFT_LIMIT:
                    is_dup_cam1 = False
                    lx2, ly2 = pair2[0][0], pair2[0][1] # Map 좌표
                    
                    # 조건 2: CAM1에 근처에 있는 Hook이 있는가? (Map 거리 기반)
                    for pair1 in h_pairs_cam1:
                        lx1, ly1 = pair1[0][0], pair1[0][1]
                        dist = ((lx1 - lx2)**2 + (ly1 - ly2)**2)**0.5
                        
                        # Map상 거리가 가까우면(100px 이내) 같은 객체로 판단 -> CAM2 삭제
                        if dist < HOOK_DUP_DIST:
                            is_dup_cam1 = True
                            break
                    
                    if is_dup_cam1:
                        continue # 중복이므로 리스트에 추가하지 않음 (CAM2 제거)

            except Exception: pass
            
            filtered_cam2_hooks.append(pair2)
        
        # 필터링된 리스트로 교체
        final_hooks_cam2 = filtered_cam2_hooks
        
        # [설정] CAM3 중복 처리 기준 (이미지 오른쪽 끝에서의 거리 픽셀)
        IMG_WIDTH = 1280 
        CAM3_HOOK_MARGIN = 150   # 후크용 마진
        CAM3_PERSON_MARGIN = 100 # 사람용 마진

        # -------------------------------
        # [HOOK] CAM3 후크 처리 (수정됨)
        # -------------------------------
        for pair3 in h_pairs_cam3:
            pt3 = pair3[0]; lx3, ly3 = pt3[0], pt3[1]
            box3 = (lx3 - HOOK_BOX_SIZE//2, ly3 - HOOK_BOX_SIZE//2, lx3 + HOOK_BOX_SIZE//2, ly3 + HOOK_BOX_SIZE//2)
            is_dup = False
            
            # (1) CAM2와 중복 체크
            for pair2 in final_hooks_cam2:
                lx2, ly2 = pair2[0][0], pair2[0][1]
                dist = ((lx2 - lx3)**2 + (ly2 - ly3)**2)**0.5
                box2 = (lx2 - HOOK_BOX_SIZE//2, ly2 - HOOK_BOX_SIZE//2, lx2 + HOOK_BOX_SIZE//2, ly2 + HOOK_BOX_SIZE//2)
                if dist < HOOK_DUP_DIST or self.calculate_iou(box2, box3) > HOOK_DUP_IOU:
                    is_dup = True; break
            
            if is_dup: continue

            # (2) CAM4와 중복 체크 (조건부 적용)
            for pair4 in final_hooks_cam4:
                lx4, ly4 = pair4[0][0], pair4[0][1]
                dist = ((lx4 - lx3)**2 + (ly4 - ly3)**2)**0.5
                box4 = (lx4 - HOOK_BOX_SIZE//2, ly4 - HOOK_BOX_SIZE//2, lx4 + HOOK_BOX_SIZE//2, ly4 + HOOK_BOX_SIZE//2)
                
                # 거리가 가깝거나 IOU가 높으면 중복 후보
                if dist < HOOK_DUP_DIST or self.calculate_iou(box4, box3) > HOOK_DUP_IOU:
                    try:
                        # pair3[1]구조: ((cx, cy, w, h), angle) -> cx는 pair3[1][0][0]
                        raw_cx = pair3[1][0][0]
                        # CAM3 오른쪽 끝(경계)에 있는 경우에만 중복 처리(삭제)
                        if raw_cx > (IMG_WIDTH - CAM3_HOOK_MARGIN):
                            is_dup = True
                        else:
                            is_dup = False # 안쪽에 있으면 살림
                    except:
                        is_dup = True # 데이터 에러 시 안전하게 삭제
                    
                    if is_dup: break

            if not is_dup: final_hooks_cam3.append(pair3)

        # -------------------------------
        # [HOOK] CAM5 후크 처리 (기존 유지)
        # -------------------------------
        for pair5 in h_pairs_cam5:
            pt5 = pair5[0]; lx5, ly5 = pt5[0], pt5[1]
            box5 = (lx5 - HOOK_BOX_SIZE//2, ly5 - HOOK_BOX_SIZE//2, lx5 + HOOK_BOX_SIZE//2, ly5 + HOOK_BOX_SIZE//2)
            is_dup = False
            for pair4 in final_hooks_cam4:
                lx4, ly4 = pair4[0][0], pair4[0][1]
                dist = ((lx4 - lx5)**2 + (ly4 - ly5)**2)**0.5
                box4 = (lx4 - HOOK_BOX_SIZE//2, ly4 - HOOK_BOX_SIZE//2, lx4 + HOOK_BOX_SIZE//2, ly4 + HOOK_BOX_SIZE//2)
                if dist < HOOK_DUP_DIST or self.calculate_iou(box4, box5) > HOOK_DUP_IOU:
                    is_dup = True; break
            if not is_dup: final_hooks_cam5.append(pair5)

        # 3. 사람 데이터 준비
        # ▼▼▼ [추가] CAM1 데이터 사용
        p_pairs_cam2 = self.layout_points_cache.get('lblcam2', [])
        p_pairs_cam3 = self.layout_points_cache.get('lblcam3', [])
        p_pairs_cam4 = self.layout_points_cache.get('lblcam4', [])
        p_pairs_cam5 = self.layout_points_cache.get('lblcam5', [])
        
        # 임시 리스트
        final_people_cam2_pairs = list(p_pairs_cam2) # 복사본
        final_people_cam4 = [p[0] for p in p_pairs_cam4]
        final_people_cam3 = []
        final_people_cam5 = []

        PERSON_DUP_DIST = 50.0
        PERSON_DUP_IOU = 0.2
        PERSON_BOX_SIZE = 30

        # -------------------------------------------------------------
        # [PERSON] CAM2 필터링 (CAM1과 중복 시 CAM1 우선) - 신규 추가됨
        # -------------------------------------------------------------
        filtered_cam2_people = []
        for pair2 in final_people_cam2_pairs:
            try:
                raw_data = pair2[1] # (cx, cy, w, h) 또는 (cx, cy)
                raw_cx = 0
                if isinstance(raw_data, (list, tuple)) and len(raw_data) >= 1:
                    raw_cx = raw_data[0]
                
                # 조건 1: CAM2 화면상 왼쪽 구석(0~120px)에 있는가?
                if raw_cx < CAM2_LEFT_LIMIT:
                    is_dup_cam1 = False
                    lx2, ly2 = pair2[0][0], pair2[0][1] # Map 좌표
                    
                    # 조건 2: CAM1에 근처에 있는 사람이 있는가?
                    for pair1 in p_pairs_cam1:
                        lx1, ly1 = pair1[0][0], pair1[0][1]
                        dist = ((lx1 - lx2)**2 + (ly1 - ly2)**2)**0.5
                        
                        if dist < PERSON_DUP_DIST:
                            is_dup_cam1 = True
                            break
                    
                    if is_dup_cam1:
                        continue # 중복이므로 제거

            except Exception: pass
            
            filtered_cam2_people.append(pair2[0]) # 결과는 좌표만 저장 (기존 로직 호환)

        # 필터링된 리스트로 교체 (좌표 리스트 형태)
        final_people_cam2 = filtered_cam2_people

        # -------------------------------
        # [PERSON] CAM3 사람 처리 (수정됨)
        # -------------------------------
        # 주의: 기존 코드에서 h_pairs_cam3를 쓰던 버그 수정됨 -> p_pairs_cam3 사용
        for pair3 in p_pairs_cam3:
            pt3 = pair3[0]; lx3, ly3 = pt3[0], pt3[1]
            box3 = (lx3 - PERSON_BOX_SIZE//2, ly3 - PERSON_BOX_SIZE//2, lx3 + PERSON_BOX_SIZE//2, ly3 + PERSON_BOX_SIZE//2)
            is_dup = False
            
            # (1) CAM2 사람과 비교
            for pt2 in final_people_cam2:
                lx2, ly2 = pt2[0], pt2[1]
                dist = ((lx2 - lx3)**2 + (ly2 - ly3)**2)**0.5
                box2 = (lx2 - PERSON_BOX_SIZE//2, ly2 - PERSON_BOX_SIZE//2, lx2 + PERSON_BOX_SIZE//2, ly2 + PERSON_BOX_SIZE//2)
                if dist < PERSON_DUP_DIST or self.calculate_iou(box2, box3) > PERSON_DUP_IOU:
                    is_dup = True; break
            
            if is_dup: continue

            # (2) CAM4 사람과 비교 (조건부 적용)
            for pt4 in final_people_cam4:
                lx4, ly4 = pt4[0], pt4[1]
                dist = ((lx4 - lx3)**2 + (ly4 - ly3)**2)**0.5
                box4 = (lx4 - PERSON_BOX_SIZE//2, ly4 - PERSON_BOX_SIZE//2, lx4 + PERSON_BOX_SIZE//2, ly4 + PERSON_BOX_SIZE//2)
                
                if dist < PERSON_DUP_DIST or self.calculate_iou(box4, box3) > PERSON_DUP_IOU:
                    try:
                        # pair3[1] = [cx, cy, w, h] or [cx, cy]
                        raw_data = pair3[1]
                        raw_cx = 0
                        if isinstance(raw_data, (list, tuple)) and len(raw_data) >= 1:
                            raw_cx = raw_data[0]
                        
                        # CAM3 오른쪽 끝 경계에 있을 때만 삭제
                        if raw_cx > (IMG_WIDTH - CAM3_PERSON_MARGIN):
                            is_dup = True
                        else:
                            is_dup = False
                    except:
                        is_dup = True

                    if is_dup: break

            if not is_dup: final_people_cam3.append(pt3)

        # -------------------------------
        # [PERSON] CAM5 사람 처리 (기존 유지)
        # -------------------------------
        for pair5 in p_pairs_cam5:
            pt5 = pair5[0]; lx5, ly5 = pt5[0], pt5[1]
            box5 = (lx5 - PERSON_BOX_SIZE//2, ly5 - PERSON_BOX_SIZE//2, lx5 + PERSON_BOX_SIZE//2, ly5 + PERSON_BOX_SIZE//2)
            is_dup = False
            for pt4 in final_people_cam4:
                lx4, ly4 = pt4[0], pt4[1]
                dist = ((lx4 - lx5)**2 + (ly4 - ly5)**2)**0.5
                box4 = (lx4 - PERSON_BOX_SIZE//2, ly4 - PERSON_BOX_SIZE//2, lx4 + PERSON_BOX_SIZE//2, ly4 + PERSON_BOX_SIZE//2)
                if dist < PERSON_DUP_DIST or self.calculate_iou(box4, box5) > PERSON_DUP_IOU:
                    is_dup = True; break
            if not is_dup: final_people_cam5.append(pt5)

        # 4. 결과 Map 반영
        self.tmap_label.set_hook_positions_for_cam('lblcam2', final_hooks_cam2)
        self.tmap_label.set_hook_positions_for_cam('lblcam3', final_hooks_cam3)
        self.tmap_label.set_hook_positions_for_cam('lblcam4', final_hooks_cam4)
        self.tmap_label.set_hook_positions_for_cam('lblcam5', final_hooks_cam5)
        self.tmap_label.set_people_positions_for_cam('lblcam2', final_people_cam2)
        self.tmap_label.set_people_positions_for_cam('lblcam3', final_people_cam3)
        self.tmap_label.set_people_positions_for_cam('lblcam4', final_people_cam4)
        self.tmap_label.set_people_positions_for_cam('lblcam5', final_people_cam5)

        # 5. 안전 거리(Warning/Danger) 체크 로직 (기존 유지)
        all_people = final_people_cam2 + final_people_cam3 + final_people_cam4 + final_people_cam5
        
        all_hooks_with_angle = []
        for h_list in [final_hooks_cam2, final_hooks_cam3, final_hooks_cam4, final_hooks_cam5]:
            for h in h_list:
                # h 구조: (coords, angle) 또는 (coords, (raw, angle)) 등... 데이터 구조에 맞게 처리
                # 여기서는 기존 코드 로직을 따름
                if len(h) > 1 and isinstance(h[1], (list, tuple)) and len(h[1]) > 1: 
                    all_hooks_with_angle.append((h[0], h[1][1]))
                elif len(h) > 1 and isinstance(h[1], (float, int)):
                     all_hooks_with_angle.append((h[0], h[1]))
                else: 
                    all_hooks_with_angle.append((h[0], 0.0))

        # map 상에서의 거리
        WARNING_DIST = 10.0
        WARNING_ANGLE_THRESHOLD = 30.0 # hook 각도 임계값 
        current_state = 'normal'
        
        for p_item in all_people:
            # [수정 1] 변수를 먼저 0으로 초기화하여 UnboundLocalError 원천 차단
            px, py = 0, 0
            
            # [수정 2] 데이터 길이에 따른 안전한 파싱
            if len(p_item) == 4:
                px, py, pw, ph = p_item
            elif len(p_item) == 2:
                px, py = p_item
            else:
                # 길이가 4도 2도 아닌 경우(예: 3)에도 앞의 2개 좌표는 가져오도록 처리
                if len(p_item) >= 2:
                    px = p_item[0]
                    py = p_item[1]
                else:
                    # 데이터가 비정상이면 이 사람은 건너뜀
                    continue

            # 3. p_box 정의
            p_box = (px - PERSON_BOX_SIZE//2, py - PERSON_BOX_SIZE//2, px + PERSON_BOX_SIZE//2, py + PERSON_BOX_SIZE//2)
            
            for h_info in all_hooks_with_angle:
                h_coords, h_angle = h_info
                
                if abs(h_angle) <= WARNING_ANGLE_THRESHOLD: continue
                
                # 훅 좌표 파싱
                if len(h_coords) >= 2:
                    hx, hy = h_coords[0], h_coords[1]
                else: continue

                h_box = (hx - HOOK_BOX_SIZE//2, hy - HOOK_BOX_SIZE//2, hx + HOOK_BOX_SIZE//2, hy + HOOK_BOX_SIZE//2)
                
                dist = ((px - hx)**2 + (py - hy)**2)**0.5
                iou = self.calculate_iou(p_box, h_box)
                
                if dist < WARNING_DIST and iou > 0.05:
                    current_state = 'warning'
                    break
            
            if current_state == 'warning': break
            
        self.detected_safety_state = current_state
        self.update_safety_status()
        
        
    # [추가] 한글 경로 호환 이미지 저장 헬퍼 함수
    def imwrite_korean(self, filename, img):
        try:
            ext = os.path.splitext(filename)[1]
            result, n = cv2.imencode(ext, img)
            if result:
                with open(filename, mode='w+b') as f:
                    n.tofile(f)
                return True
            return False
        except Exception as e:
            print(f"이미지 인코딩/저장 오류: {e}")
            return False
        

    def save_alarm_snapshot(self, cam_name, grade):
        # 1. 절대 경로 설정
        base_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 저장 폴더 이름 결정
        if grade == 'danger':
            folder_name = "danger_images"
        else:
            folder_name = "warning_images"
            
        save_dir = os.path.join(base_dir, folder_name)
        
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        target_label = f"lbl{cam_name.lower()}"
        if target_label in self.camera_controller.labels:
            worker = self.camera_controller.labels[target_label]['worker']
            
            # 프레임 가져오기
            frame = worker.processed_frame
            if frame is None:
                frame = worker.raw_frame
            
            # ▼▼▼ [핵심 수정] 프레임 유효성 정밀 검사 ▼▼▼
            # 1. None 체크
            # 2. NumPy 배열 타입 체크 (isinstance)
            # 3. 데이터가 비어있지 않은지 체크 (size > 0)
            if frame is not None and isinstance(frame, np.ndarray) and frame.size > 0:
                try:
                    now = datetime.now()
                    timestamp = now.strftime("%Y%m%d_%H%M%S")
                    filename = f"{timestamp}_{cam_name}_{grade}.png"
                    full_path = os.path.join(save_dir, filename)
                    
                    # 한글 경로 지원 저장 시도
                    success = self.imwrite_korean(full_path, frame)
                    
                    if success:
                        print(f"[{cam_name}] {grade} 알람 이미지 저장 성공: {full_path}")
                        self.insert_alarm_to_db(now, cam_name, grade, filename)
                    else:
                        print(f"[{cam_name}] 오류: 이미지 저장 실패 (imwrite 반환값 False)")
                except Exception as e:
                    print(f"[{cam_name}] 저장 중 예외 발생: {e}")
            else:
                # 디버깅을 위해 어떤 데이터가 들어왔는지 출력
                print(f"[{cam_name}] 오류: 유효하지 않은 프레임 데이터입니다. (Type: {type(frame)})")
                                

    def insert_alarm_to_db(self, dt, cam_name, grade, filename):
        conn = None
        try:
            # 실행 위치가 달라도 항상 같은 DB 파일을 바라보게 함
            base_dir = os.path.dirname(os.path.abspath(__file__))
            db_path = os.path.join(base_dir, "alarm_gy.db")
            
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            
            db_time = dt.strftime("%Y-%m-%d %H:%M:%S")
            
            # ▼▼▼ [수정 2] 실제 저장이 시도되는지 확인하는 디버깅 로그 추가
            print(f"[DB Insert] 시도: {cam_name} | {grade} | {filename}")
            
            cursor.execute("""
                INSERT INTO Alarm (al_date, al_cam, al_grade, al_file)
                VALUES (?, ?, ?, ?)
            """, (db_time, cam_name, grade, filename))
            conn.commit()
        except sqlite3.Error as e: print(f"알람 DB 저장 오류: {e}")
        finally:
            if conn: conn.close()


    # # [수정] 키보드 이벤트(D키)로 강제 Danger 테스트 모드 진입
    # def keyPressEvent(self, event):
    #     if event.key() == Qt.Key_D:
    #         if self.is_test_mode_active:
    #             print("[TEST] 이미 테스트 진행 중입니다.")
    #             return

    #         print("[TEST] 강제 Danger 테스트 시작! (3초간 유지)")
            
    #         # 1. [Lock] 실시간 감지 로직이 간섭하지 못하게 막음
    #         self.is_test_mode_active = True

    #         # 2. [DB 저장] 즉시 실행
    #         self.save_alarm_snapshot('CAM2', 'danger')

    #         # 3. [화면 강제 표시] 테두리 및 맵 상태 변경
    #         # (1) 테두리 깜빡임 강제 설정
    #         self.camera_alert_states['lblcam2'] = 'danger'
    #         if not self.warning_timer.isActive():
    #             self.warning_timer.start()

    #         # (2) Map 상태 강제 설정 (아이콘 색상 + 텍스트)
    #         self.tmap_label.update_cam_state('lblcam2', 'danger')
    #         self.tmap_label.set_area2_state('danger')

    #         # 4. [복구 예약] 3초 뒤에 정상 상태로 복구
    #         QTimer.singleShot(3000, self._reset_test_state)

    #     super().keyPressEvent(event)


    # # [추가] 테스트 상태 초기화 헬퍼 함수
    # def _reset_test_state(self):
    #     print("[TEST] 테스트 종료 -> 정상 감지 모드로 복귀")
        
    #     # 1. 강제 설정했던 상태들 초기화
    #     self.camera_alert_states.clear() # 모든 경고 상태 제거
    #     self._reset_all_camera_styles()  # 테두리 투명하게
        
    #     if self.warning_timer.isActive():
    #         self.warning_timer.stop()
        
    #     # 2. Map 상태 초기화
    #     self.tmap_label.update_cam_state('lblcam2', 'normal')
    #     self.tmap_label.set_area2_state('normal')

    #     # 3. [Unlock] 락을 풀어 다시 실시간 감지 시작
    #     self.is_test_mode_active = False
        
            
#==============================================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = widget(app)
    window.show()
    sys.exit(app.exec())