# recording_manager.py
# Recording 기능을 담당하는 별도 클래스

import datetime
import os

import cv2
from PySide6.QtCore import QObject, Signal, Slot
from PySide6.QtWidgets import QMessageBox


class RecordingManager(QObject):
    """
    녹화 기능을 담당하는 클래스
    """

    # 녹화 상태 변경 시그널
    recording_started = Signal(bool, str)  # (성공여부, 메시지)
    recording_stopped = Signal(str)  # (파일경로)
    recording_error = Signal(str)  # (에러메시지)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_recording = False
        self.video_writer = None
        self.output_file = None
        self.frame_size = None
        self.fps = 20  # 기본 fps
        # 녹화 설정 - 기본 코덱 사용
        self.recording_config = {
            "fps": 20,
            "codec": "mp4v",  # 단일 코덱 사용
            "output_dir": "video_recording",
        }

    def set_recording_config(self, fps=None, output_dir=None):
        """녹화 설정 업데이트"""
        if fps is not None:
            self.recording_config["fps"] = fps
        if output_dir is not None:
            self.recording_config["output_dir"] = output_dir

    def set_frame_size(self, size):
        """프레임 크기 설정"""
        self.frame_size = size
        print(f"[RecordingManager] Frame size set to: {size}")

    @Slot()
    def start_recording(self, custom_filename=None):
        """
        녹화 시작
        """
        print("[RecordingManager] start_recording called")

        if self.is_recording:
            self.recording_started.emit(False, "이미 녹화 중입니다.")
            return

        if self.frame_size is None:
            self.recording_error.emit("프레임 크기가 설정되지 않았습니다.")
            return

        # 출력 디렉토리 생성
        output_dir = self.recording_config["output_dir"]
        os.makedirs(output_dir, exist_ok=True)

        # 파일명 생성
        if custom_filename:
            self.output_file = os.path.join(output_dir, custom_filename)
        else:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_file = os.path.join(output_dir, f"recording_{timestamp}.mp4")

        print(f"[RecordingManager] Starting recording: {self.output_file}")

        # 비디오 라이터 초기화
        success = self._initialize_video_writer()

        if success:
            self.is_recording = True
            self.recording_started.emit(True, f"녹화 시작: {self.output_file}")
        else:
            self.recording_started.emit(False, "녹화기 초기화 실패")

    def _initialize_video_writer(self):
        """비디오 라이터 초기화"""
        fps = self.recording_config["fps"]
        size = self.frame_size
        codec = self.recording_config["codec"]

        try:
            fourcc = cv2.VideoWriter_fourcc(*codec)
            self.video_writer = cv2.VideoWriter(self.output_file, fourcc, fps, size)

            if not self.video_writer.isOpened():
                print(f"[RecordingManager] Failed to open video writer with {codec}")
                if self.video_writer:
                    self.video_writer.release()
                    self.video_writer = None
                return False

            print(f"[RecordingManager] Recording started successfully with {codec}: {self.output_file}")
            return True

        except Exception as e:
            print(f"[RecordingManager] Exception with codec {codec}: {e}")
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
            return False

    @Slot()
    def stop_recording(self):
        """녹화 중지"""
        print("[RecordingManager] stop_recording called")

        if not self.is_recording:
            print("[RecordingManager] Not currently recording")
            return

        self.is_recording = False

        if self.video_writer:
            print("[RecordingManager] Releasing video writer")
            try:
                self.video_writer.release()
            except Exception as e:
                print(f"[RecordingManager] Error releasing video writer: {e}")
            finally:
                self.video_writer = None

        output_file = self.output_file
        self.output_file = None

        print("[RecordingManager] Recording stopped, emitting signal")
        self.recording_stopped.emit(output_file)

    def write_frame(self, frame):
        """
        프레임을 녹화 파일에 쓰기
        """
        if not self.is_recording or not self.video_writer or not self.video_writer.isOpened():
            return False

        try:
            # 프레임 크기 조정이 필요한 경우만 처리
            if self.frame_size and frame.shape[:2][::-1] != self.frame_size:
                frame_resized = cv2.resize(frame, self.frame_size)
                return self.video_writer.write(frame_resized)
            else:
                return self.video_writer.write(frame)

        except Exception as e:
            print(f"[RecordingManager] Error writing frame: {e}")
            return False

    def cleanup(self):
        """리소스 정리"""
        print("[RecordingManager] cleanup called")

        if self.is_recording:
            self.stop_recording()

        if self.video_writer:
            try:
                self.video_writer.release()
            except Exception as e:
                print(f"[RecordingManager] Error releasing video writer in cleanup: {e}")
            finally:
                self.video_writer = None

        self.output_file = None
        self.frame_size = None
