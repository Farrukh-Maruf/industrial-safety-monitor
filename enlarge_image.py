# enlarge_image.py
# 9/22 : 동영상저장, 이미지캡처 구현
# 9/29 : Recording 기능 리팩토링 - RecordingManager 클래스 도입
# 9/30 : 이미지 확대(1280x720)로 변경, 녹화 프레임 크기 동기화
# 10/23 : 이미지확대(1111x640)을 (1280x720)로 재변경, 녹화 상태 표시 개선
# 10/23 : LiDAR 영상 확대창에서 녹화 기능 정상 동작하도록 수정
# 12/08 : 마우스 휠 줌(확대/축소) 및 드래그(이동) 기능을 추가

import datetime
import os
import sys
import cv2
import numpy as np
from PySide6.QtCore import QObject, Qt, QThread, QTimer, Signal, Slot, QCoreApplication, QEvent, QPoint, QRect
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QApplication, QDialog, QMessageBox
from ultralytics import YOLO

from enlarge_imageui import Ui_enlargeDialog
from recording_manager import RecordingManager


# 카메라/LiDAR 영상 처리를 위한 워커 객체
# 이 워커는 EnlargeDialog에서 독립적으로 사용됩니다.
class VideoWorker(QObject):
    frame_ready = Signal(QImage)
    finished = Signal()

    def __init__(self, source, parent=None):
        super().__init__(parent)
        self.source = source
        self.cap = None
        # YOLO 모델 제거 - camera_gy.py에서 처리
        self.is_running = True
        self.current_frame = None
        self.frame_size = None
        self.timer = None   

        # 새로운 RecordingManager 인스턴스 생성
        self.recording_manager = RecordingManager(self)
        # RecordingManager의 시그널 연결
        self.recording_manager.recording_started.connect(self._on_recording_started)
        self.recording_manager.recording_stopped.connect(self._on_recording_stopped)
        self.recording_manager.recording_error.connect(self._on_recording_error)

    # RecordingManager 시그널 처리 메서드들
    def _on_recording_started(self, success, message):
        """녹화 시작 결과 처리"""
        print(f"[VideoWorker] Recording started: success={success}, message='{message}'")

    def _on_recording_stopped(self, output_file):
        """녹화 중지 처리"""
        print(f"[VideoWorker] Recording stopped: {output_file}")

    def _on_recording_error(self, error_message):
        """녹화 에러 처리"""
        print(f"[VideoWorker] Recording error: {error_message}")
        # 필요시 에러 처리 로직 추가

    def start_capture(self):
        """캡처 시작 - 워커 스레드에서 호출됨"""
        print(f"[VideoWorker] start_capture() called for source: {self.source}")
        print(f"[VideoWorker] Current thread: {self.thread()}")

        try:
            self.timer = QTimer()
            self.timer.timeout.connect(self.process_frame)

            self.cap = cv2.VideoCapture(self.source)
            if not self.cap.isOpened():
                print(f"[VideoWorker] Error: Can't access the source at {self.source}")
                self.is_running = False
                self.finished.emit()
                return False

            print("[VideoWorker] Camera opened successfully, starting timer")
            self.timer.start(33)  # 약 30fps (33ms 간격)
            return True

        except Exception as e:
            print(f"[VideoWorker] Exception in start_capture(): {e}")
            self.is_running = False
            self.finished.emit()
            return False

    def process_frame(self):
        """타이머에 의해 호출되는 프레임 처리 메서드"""
        if not self.is_running or not self.cap or not self.cap.isOpened():
            return

        try:
            ret, frame = self.cap.read()
            if not ret:
                print("[VideoWorker] Failed to read frame")
                return

            # 프레임 유효성 검사
            if frame is None or frame.size == 0:
                print("[VideoWorker] Invalid frame received")
                return

            if self.frame_size is None:
                h, w = frame.shape[:2]
                self.frame_size = (w, h)
                print(f"[VideoWorker] Detected frame size: {self.frame_size}")

            self.current_frame = frame.copy()

            # RecordingManager를 통한 프레임 쓰기
            if self.is_running and self.recording_manager.is_recording:
                try:
                    self.recording_manager.write_frame(frame)
                except Exception as recording_error:
                    print(f"[VideoWorker] Recording error: {recording_error}")

            # RGB 변환 및 QImage 생성 (안전하게 처리)
            try:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame_rgb.shape
                bytes_per_line = ch * w
                q_img = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                self.frame_ready.emit(q_img)
            except Exception as image_error:
                print(f"[VideoWorker] Image conversion error: {image_error}")

        except Exception as e:
            print(f"[VideoWorker] Exception in process_frame(): {e}")
            # 심각한 오류가 발생해도 워커를 중지하지 않음

    # 단순화된 RecordingManager 기반 메서드들
    @Slot(str, int, tuple)
    def start_recording(self, output_file, fps, size):
        """RecordingManager를 사용한 녹화 시작"""
        print(f"[VideoWorker] start_recording called: {output_file}, fps={fps}, size={size}")

        # 실제 프레임 크기 사용 (size 파라미터보다 실제 프레임 크기 우선)
        actual_size = self.frame_size if self.frame_size else size
        print(f"[VideoWorker] Using frame size: {actual_size}")

        # RecordingManager 설정 업데이트
        self.recording_manager.set_recording_config(fps=fps)
        self.recording_manager.set_frame_size(actual_size)

        # 파일명 추출 (전체 경로에서 파일명만)
        filename = os.path.basename(output_file)

        # RecordingManager를 통한 녹화 시작
        self.recording_manager.start_recording(filename)

    @Slot()
    def stop_recording(self):
        """RecordingManager를 사용한 녹화 중지"""
        print("[VideoWorker] stop_recording called")
        self.recording_manager.stop_recording()

    def stop(self):
        """워커 중지"""
        print("[VideoWorker] stop() called")
        self.is_running = False

        # QTimer 안전하게 정리 (스레드 오류 방지)
        if self.timer:
            try:
                # 현재 스레드에서 타이머 정리
                if self.timer.isActive():
                    self.timer.stop()
                self.timer = None
            except Exception as e:
                print(f"[VideoWorker] Error stopping timer: {e}")

        # RecordingManager 정리
        if hasattr(self, "recording_manager"):
            self.recording_manager.cleanup()

        if self.cap and self.cap.isOpened():
            try:
                self.cap.release()
            except Exception as e:
                print(f"[VideoWorker] Error releasing camera in cleanup: {e}")

        print("[VideoWorker] stop() completed")
        self.finished.emit()


class EnlargeDialog(QDialog):
    start_record_signal = Signal(str, int, tuple)
    stop_record_signal = Signal()

    def __init__(self, parent=None, image_source=None, is_video=True, shared_worker=None, lidar_callback_data=None):
        super().__init__(parent)
        self.ui = Ui_enlargeDialog()
        self.ui.setupUi(self)

        # ▼▼▼ [추가] 줌/팬 기능 변수 초기화 ▼▼▼
        self.zoom_factor = 1.0          # 현재 배율 (1.0 = 100%)
        self.pan_offset = QPoint(0, 0)  # 중심점 이동 오프셋
        self.is_panning = False         # 드래그 중인지 여부
        self.last_mouse_pos = QPoint()  # 마우스 마지막 위치
        # ▲▲▲ [추가] 완료 ▲▲▲

        # ▼▼▼ [수정] Lidar/Camera 창 크기 설정 ▼▼▼
        if lidar_callback_data is not None:
            # Lidar 팝업 (640x240)
            self.ui.lblenlarge.setFixedSize(640, 240)
            self.ui.layoutWidget.resize(640, 37) 
            self.ui.verticalLayoutWidget_2.resize(640, 240)
            self.resize(700, 320) 
            self.setWindowTitle("Enlarge Lidar (640x240)") 
        else:
            # Camera 팝업 (1280x720)
            self.ui.lblenlarge.setFixedSize(1280, 720)
            self.ui.layoutWidget.resize(1280, 37) 
            self.ui.verticalLayoutWidget_2.resize(1280, 720)
            self.resize(1340, 800) 
        # ▲▲▲ [수정] 완료 ▲▲▲
        
        self.setModal(False)
        self.setWindowFlags(self.windowFlags() | Qt.Window)

        self.image_source = image_source
        self.is_video = is_video
        self.video_thread = None
        self.video_worker = None
        self.shared_worker = shared_worker
        self.current_frame = None

        # [수정] 이벤트 필터 설치 및 마우스 트래킹 활성화
        self.lidar_callback_data = lidar_callback_data
        self.ui.lblenlarge.installEventFilter(self)
        self.ui.lblenlarge.setMouseTracking(True) # 드래그 이동을 위해 필요

        # 녹화 관련 초기화
        self.is_recording = False
        self.recording_requested = False

        self.ui.pbnrecordstart.clicked.connect(self.start_recording)
        self.ui.pbnrecordstop.clicked.connect(self.stop_recording)
        self.ui.pbnimgcapture.clicked.connect(self.capture_image)

        self.ui.pbnrecordstop.setEnabled(False)

        if self.is_video and self.image_source:
            if self.shared_worker:
                self.start_shared_video_stream()
            else:
                self.start_video_stream()
        else:
            self.ui.lblenlarge.setText("Image not available.")

    def start_video_stream(self):
        if self.video_thread and self.video_thread.isRunning():
            self.stop_video_stream()

        self.video_thread = QThread()
        self.video_worker = VideoWorker(self.image_source)
        self.video_worker.moveToThread(self.video_thread)

        self.video_thread.started.connect(self.video_worker.start_capture)
        self.video_worker.frame_ready.connect(self.update_image)
        self.video_worker.finished.connect(self.video_thread.quit)
        self.video_worker.finished.connect(self.video_worker.deleteLater)
        self.video_thread.finished.connect(self.video_thread.deleteLater)

        # RecordingManager 시그널 연결
        self.start_record_signal.connect(self.video_worker.start_recording, Qt.QueuedConnection)
        self.stop_record_signal.connect(self.video_worker.stop_recording, Qt.QueuedConnection)
        
        self.video_worker.recording_manager.recording_started.connect(self.handle_recording_started, Qt.QueuedConnection)
        self.video_worker.recording_manager.recording_stopped.connect(self.handle_recording_stopped, Qt.QueuedConnection)
        self.video_worker.recording_manager.recording_error.connect(self.handle_recording_error, Qt.QueuedConnection)

        self.video_thread.start()

    def start_shared_video_stream(self):
        """공유 worker를 사용한 비디오 스트림 시작"""
        if not self.shared_worker:
            return

        try:
            self.shared_worker.frame_shared.connect(self.update_shared_frame, Qt.DirectConnection)
        except Exception as e:
            print(f"[EnlargeDialog] Error connecting frame_shared signal: {e}")

        self.recording_manager = RecordingManager(self)
        self.recording_manager.recording_started.connect(self.handle_recording_started, Qt.QueuedConnection)
        self.recording_manager.recording_stopped.connect(self.handle_recording_stopped, Qt.QueuedConnection)
        self.recording_manager.recording_error.connect(self.handle_recording_error, Qt.QueuedConnection)

    # ▼▼▼ [추가] 줌/팬 처리 헬퍼 메서드 ▼▼▼
    def _get_zoomed_pixmap(self, q_img):
        """현재 zoom_factor와 pan_offset을 기반으로 이미지를 잘라내어(Crop) 확대된 Pixmap을 반환"""
        if q_img is None:
            return QPixmap()

        if self.zoom_factor == 1.0:
            return QPixmap.fromImage(q_img)

        img_w = q_img.width()
        img_h = q_img.height()

        # 보여질 영역(Viewport) 크기 계산
        view_w = int(img_w / self.zoom_factor)
        view_h = int(img_h / self.zoom_factor)

        # 중심점 계산 (기본 중심 + 이동 오프셋)
        center_x = (img_w // 2) - self.pan_offset.x()
        center_y = (img_h // 2) - self.pan_offset.y()

        # 크롭할 사각형(Rect)의 좌상단 좌표 계산
        crop_x = center_x - (view_w // 2)
        crop_y = center_y - (view_h // 2)

        # 경계 처리 (이미지 밖으로 나가지 않도록 Clamp)
        crop_x = max(0, min(crop_x, img_w - view_w))
        crop_y = max(0, min(crop_y, img_h - view_h))

        # 이미지 크롭
        rect = QRect(crop_x, crop_y, view_w, view_h)
        cropped_img = q_img.copy(rect)

        # 라벨 크기에 맞게 확대 (SmoothTransformation 사용)
        pixmap = QPixmap.fromImage(cropped_img)
        return pixmap.scaled(self.ui.lblenlarge.size(), Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
    # ▲▲▲ [추가] 완료 ▲▲▲

    def update_shared_frame(self, frame):
        """공유 worker에서 받은 프레임 처리"""
        try:
            if frame is None or frame.size == 0:
                return

            self.current_frame = frame.copy()

            if hasattr(self, "recording_manager") and self.recording_manager.is_recording:
                try:
                    self.recording_manager.write_frame(frame)
                except Exception: pass

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            bytes_per_line = ch * w
            q_img = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

            # ▼▼▼ [수정] 줌 기능 적용하여 표시 ▼▼▼
            final_pixmap = self._get_zoomed_pixmap(q_img)
            self.ui.lblenlarge.setPixmap(final_pixmap)

        except Exception as e:
            print(f"[EnlargeDialog] Error processing shared frame: {e}")

    def update_image(self, q_img):
        """독립 Worker에서 받은 이미지 업데이트"""
        # 현재 프레임 저장 (캡처용)
        q_img_rgb = q_img.convertToFormat(QImage.Format_RGB888)
        img_np = np.frombuffer(q_img_rgb.bits().tobytes(), dtype=np.uint8).reshape((q_img_rgb.height(), q_img_rgb.width(), 3))
        self.current_frame = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
        
        # ▼▼▼ [수정] 줌 기능 적용하여 표시 ▼▼▼
        final_pixmap = self._get_zoomed_pixmap(q_img)
        self.ui.lblenlarge.setPixmap(final_pixmap)

    def stop_video_stream(self):
        # 공유 worker 연결 해제
        if self.shared_worker:
            try:
                self.shared_worker.frame_shared.disconnect(self.update_shared_frame)
            except Exception: pass

        # RecordingManager 정리
        if hasattr(self, "recording_manager"):
            if self.is_recording:
                self.recording_manager.stop_recording()
            self.recording_manager.cleanup()

        # 독립 모드 VideoWorker 정리
        if self.video_worker:
            if self.is_recording:
                self.stop_recording()
            self.video_worker.stop()

        if self.video_thread and self.video_thread.isRunning():
            self.video_thread.quit()
            self.video_thread.wait(3000)

    def start_recording(self):
        if self.shared_worker:
            if not self.shared_worker.is_running:
                QMessageBox.warning(self, "준비 중", "카메라 스트림 준비 중입니다.")
                return
        else:
            if not self.video_worker or not self.video_thread.isRunning():
                QMessageBox.warning(self, "준비 중", "카메라 스트림 준비 중입니다.")
                return

        if self.is_recording or self.recording_requested:
            self.update_status_label("이미 녹화 중이거나 녹화 요청 중입니다.")
            return

        output_dir = "video_recording"
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = os.path.join(output_dir, f"recording_{timestamp}.mp4")

        fps = 20
        size = (1280, 720)
        self.recording_requested = True

        if self.shared_worker:
            self.recording_manager.set_recording_config(fps=fps)
            self.recording_manager.set_frame_size(size)
            self.recording_manager.start_recording(os.path.basename(output_file))
        else:
            self.start_record_signal.emit(output_file, fps, size)

        self.ui.pbnrecordstart.setEnabled(False)
        self.ui.pbnrecordstop.setEnabled(True)

    def stop_recording(self):
        if not self.is_recording and not self.recording_requested:
            return

        if self.shared_worker:
            self.recording_manager.stop_recording()
        else:
            if self.video_worker:
                self.stop_record_signal.emit()

        self.recording_requested = False
        self.ui.pbnrecordstart.setEnabled(True)
        self.ui.pbnrecordstop.setEnabled(False)

    @Slot(bool, str)
    def handle_recording_started(self, success, message):
        self.recording_requested = False
        if success:
            self.is_recording = True
            self.update_status_label(f"녹화 시작 완료: {message.split(': ')[-1]}")
            QTimer.singleShot(3000, lambda: self.update_status_label("녹화중..."))
        else:
            self.is_recording = False
            QMessageBox.critical(self, "녹화 오류", message)
            self.ui.pbnrecordstart.setEnabled(True)
            self.ui.pbnrecordstop.setEnabled(False)

    @Slot(str)
    def handle_recording_stopped(self, output_file):
        self.is_recording = False
        self.recording_requested = False
        self.update_status_label(f"녹화 중지 완료. 파일: {os.path.basename(output_file)}")
        QTimer.singleShot(3000, lambda: self.ui.lblstatus.setText(""))
        self.ui.pbnrecordstart.setEnabled(True)
        self.ui.pbnrecordstop.setEnabled(False)

    @Slot(str)
    def handle_recording_error(self, error_message):
        self.is_recording = False
        self.recording_requested = False
        QMessageBox.critical(self, "녹화 오류", f"녹화 중 오류 발생:\n{error_message}")
        self.ui.pbnrecordstart.setEnabled(True)
        self.ui.pbnrecordstop.setEnabled(False)

    def capture_image(self):
        if self.current_frame is None:
            QMessageBox.warning(self, "알림", "캡처할 프레임이 없습니다.")
            return

        output_dir = "image_capture"
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = os.path.join(output_dir, f"capture_{timestamp}.png")

        try:
            cv2.imwrite(output_file, self.current_frame)
            self.update_status_label(f"이미지 캡처 성공: {os.path.basename(output_file)}")
            QTimer.singleShot(3000, lambda: self.ui.lblstatus.setText(""))
        except Exception as e:
            self.update_status_label(f"이미지 저장 실패: {e}", is_error=True)
            QTimer.singleShot(3000, lambda: self.ui.lblstatus.setText(""))

    def closeEvent(self, event):
        if self.is_recording or self.recording_requested:
            reply = QMessageBox.question(self, "녹화 중", "녹화가 진행 중입니다. 창을 닫으면 녹화가 중지됩니다. 계속하시겠습니까?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.stop_video_stream()
                super().closeEvent(event)
            else:
                event.ignore()
        else:
            self.stop_video_stream()
            super().closeEvent(event)

    def update_status_label(self, message, is_error=False):
        self.ui.lblstatus.setText(message)
        color = "red" if is_error else "black"
        self.ui.lblstatus.setStyleSheet(f"color: {color};")

    # ▼▼▼ [수정] 이벤트 필터: Lidar 클릭 및 Camera 줌/팬 통합 ▼▼▼
    def eventFilter(self, watched, event):
        if watched == self.ui.lblenlarge:
            # 1. Lidar 모드: 클릭 시 거리 측정 (줌 기능 없음)
            if self.lidar_callback_data:
                if event.type() == QEvent.Type.MouseButtonPress:
                    if event.button() == Qt.MouseButton.LeftButton:
                        self.handle_lidar_click(event)
                        return True
            
            # 2. Camera 모드: 줌/팬 기능
            else:
                # [휠] 확대/축소
                if event.type() == QEvent.Type.Wheel:
                    delta = event.angleDelta().y()
                    if delta > 0:
                        self.zoom_factor *= 1.1  # 10% 확대
                    else:
                        self.zoom_factor /= 1.1  # 10% 축소
                    
                    self.zoom_factor = max(1.0, min(self.zoom_factor, 5.0))
                    
                    if self.zoom_factor == 1.0:
                        self.pan_offset = QPoint(0, 0)
                        self.update_status_label("Zoom Reset")
                    else:
                        self.update_status_label(f"Zoom: x{self.zoom_factor:.1f}")
                        
                    return True

                # [클릭] 드래그 시작 또는 초기화
                elif event.type() == QEvent.Type.MouseButtonPress:
                    if event.button() == Qt.MouseButton.LeftButton:
                        if self.zoom_factor > 1.0:
                            self.is_panning = True
                            self.last_mouse_pos = event.position().toPoint()
                            self.ui.lblenlarge.setCursor(Qt.ClosedHandCursor)
                    
                    elif event.button() == Qt.MouseButton.RightButton:
                        self.zoom_factor = 1.0
                        self.pan_offset = QPoint(0, 0)
                        self.ui.lblenlarge.setCursor(Qt.ArrowCursor)
                        self.update_status_label("Zoom Reset")
                    return True

                # [이동] 드래그 중
                elif event.type() == QEvent.Type.MouseMove:
                    if self.is_panning and self.zoom_factor > 1.0:
                        current_pos = event.position().toPoint()
                        delta = current_pos - self.last_mouse_pos
                        self.last_mouse_pos = current_pos
                        
                        # 오프셋 업데이트 (마우스 이동 방향으로 이미지 이동)
                        self.pan_offset += delta
                    return True

                # [릴리즈] 드래그 종료
                elif event.type() == QEvent.Type.MouseButtonRelease:
                    if event.button() == Qt.MouseButton.LeftButton:
                        self.is_panning = False
                        self.ui.lblenlarge.setCursor(Qt.ArrowCursor)
                    return True

        return super().eventFilter(watched, event)
    # ▲▲▲ [수정] 완료 ▲▲▲

    def handle_lidar_click(self, event):
        callback_data = self.lidar_callback_data
        depth_frame = callback_data.get('depth_frame')
        original_shape = callback_data.get('original_shape')

        if depth_frame is None or original_shape is None:
            return

        x_label = int(event.position().x())
        y_label = int(event.position().y())

        label_width = self.ui.lblenlarge.width()
        label_height = self.ui.lblenlarge.height()
        TARGET_WIDTH = callback_data['target_size'][0]
        TARGET_HEIGHT = callback_data['target_size'][1]
        
        pad_x = (label_width - TARGET_WIDTH) / 2.0
        pad_y = (label_height - TARGET_HEIGHT) / 2.0
        
        x_img = int(x_label - pad_x)
        y_img = int(y_label - pad_y)

        if not (0 <= x_img < TARGET_WIDTH and 0 <= y_img < TARGET_HEIGHT):
            return

        h, w = original_shape
        y_orig = int(y_img * h / TARGET_HEIGHT)
        x_orig = int(x_img * w / TARGET_WIDTH)

        if 0 <= y_orig < h and 0 <= x_orig < w:
            y_start = max(0, y_orig - 1)
            y_end = min(h, y_orig + 2)
            x_start = max(0, x_orig - 1)
            x_end = min(w, x_orig + 2)
            
            roi = depth_frame[y_start:y_end, x_start:x_end]
            valid_pixels = roi[roi > 0]
            
            if valid_pixels.size > 0:
                distance = np.mean(valid_pixels)
            else:
                distance = depth_frame[y_orig, x_orig]
            
            callback_data['click_pos'] = (x_img, y_img)
            callback_data['distance'] = distance
            self.update_status_label(f"Distance: {distance:.2f} mm")