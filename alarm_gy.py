# alarm_gy.py
# 9/9 : 카메라 및 알람등급 선택에 따른 테이블 데이터 표시부분 추가
#     : 'Save Data' 버튼 누르면 테이블에 있는 데이터를 *.csv로 저장
# 9/10 : 알람내용으로 관련된 내용 찾기 추가, 영상정보의 특정 셀을 누르면 그 내용을 자세히 볼 수 있는 창 추가
#      : 기간을 설정하여 찾기 기능 구현(버튼 포함)

import os
import sys
import csv
import sqlite3
from PySide6.QtWidgets import (QDialog, QTableWidgetItem, QHeaderView, 
                                QLabel, QVBoxLayout, QMessageBox, QFileDialog)
from PySide6.QtGui import QPixmap
from PySide6.QtCore import Qt, QDate

# -----------------------------------------------------------
# [1] 이미지 뷰어 다이얼로그 (팝업창)
# -----------------------------------------------------------
class ImageViewerDialog(QDialog):
    def __init__(self, image_path, parent=None):
        super().__init__(parent)
        self.setWindowTitle("알람 영상 정보 확인")
        self.resize(800, 600)
        
        # 레이아웃 설정
        layout = QVBoxLayout()
        self.lbl_image = QLabel()
        self.lbl_image.setAlignment(Qt.AlignCenter)
        self.lbl_image.setStyleSheet("background-color: #2b2b2b;") # 배경 어둡게
        
        # 이미지 로드 및 표시
        if os.path.exists(image_path):
            pixmap = QPixmap(image_path)
            if not pixmap.isNull():
                # 창 크기에 맞춰 비율 유지하며 스케일링
                scaled_pixmap = pixmap.scaled(
                    780, 580, 
                    Qt.KeepAspectRatio, 
                    Qt.SmoothTransformation
                )
                self.lbl_image.setPixmap(scaled_pixmap)
            else:
                self.lbl_image.setText("이미지 파일이 손상되었거나 열 수 없습니다.")
                self.lbl_image.setStyleSheet("color: white;")
        else:
            self.lbl_image.setText(f"이미지 파일을 찾을 수 없습니다.\n경로: {image_path}")
            self.lbl_image.setStyleSheet("color: red; font-weight: bold;")
            
        layout.addWidget(self.lbl_image)
        self.setLayout(layout)


# -----------------------------------------------------------
# [2] 알람 다이얼로그 매니저 (검색, 저장 및 테이블 관리)
# -----------------------------------------------------------
class AlarmDialogManager:
    def __init__(self, main_window, ui_class):
        """
        :param main_window: 부모 윈도우 (widget_gy)
        :param ui_class: Ui_alarmDialog 클래스
        """
        self.main_window = main_window
        self.ui_class = ui_class
        self.dialog = None
        self.ui = None
        self.db_name = "alarm_gy.db"
        self.capture_dir = "warning_images" # 이미지가 저장된 폴더명

    def show_alarm_dialog(self):
        # 다이얼로그 생성 및 UI 설정
        self.dialog = QDialog(self.main_window)
        self.ui = self.ui_class()
        self.ui.setupUi(self.dialog)
        
        # ▼▼▼ [수정 1] 다이얼로그가 열릴 때 날짜를 '오늘'로 설정 ▼▼▼
        today = QDate.currentDate()
        self.ui.destart.setDate(today) # 시작일: 오늘
        self.ui.deend.setDate(today)   # 종료일: 오늘
        
        # 테이블 초기화 및 이벤트 연결
        self.setup_table()
        self.connect_signals()
        
        # 초기 데이터 조회
        self.search_data()
        
        # 다이얼로그 실행
        self.dialog.exec()

    def setup_table(self):
        """테이블 헤더 및 속성 설정"""
        header = self.ui.tblalarminfo.horizontalHeader()
        
        # ▼▼▼ [수정 2] 테이블 컬럼 크기 조절 모드 변경 ▼▼▼
        # 기존: Stretch나 ResizeToContents는 사용자가 조절 불가능(고정됨)
        # 변경: Interactive로 설정하면 마우스로 드래그하여 조절 가능
        
        # 0번 컬럼(시간): 사용자가 조절 가능하도록 변경 (기본 너비 지정 가능)
        header.setSectionResizeMode(0, QHeaderView.Interactive)
        self.ui.tblalarminfo.setColumnWidth(0, 200) # (선택사항) 초기 너비 200px 설정
        
        # 1번 컬럼(카메라): 내용에 맞추되 조절은 가능하게 하려면 Interactive 사용
        header.setSectionResizeMode(1, QHeaderView.Interactive)
        self.ui.tblalarminfo.setColumnWidth(1, 100)
        
        # 2번 컬럼(알람등급): 사용자 조절 가능
        header.setSectionResizeMode(2, QHeaderView.Interactive)
        self.ui.tblalarminfo.setColumnWidth(2, 100)
        
        # 3번 컬럼(영상정보): 마지막 컬럼은 남은 공간을 채우도록 Stretch 유지 (권장)
        # 만약 이것도 조절하고 싶다면 QHeaderView.Interactive 로 바꾸면 됩니다.
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        
        self.ui.tblalarminfo.setEditTriggers(QHeaderView.NoEditTriggers)
        self.ui.tblalarminfo.setSelectionBehavior(QHeaderView.SelectRows)
        

    def connect_signals(self):
        """버튼 및 테이블 이벤트 연결"""
        # 검색 버튼
        self.ui.pbsearch.clicked.connect(self.search_data)
        
        # [추가됨] 데이터 저장 버튼 (Save Data)
        self.ui.pbsavedata.clicked.connect(self.save_data_to_csv)
        
        # 테이블 셀 클릭 (이미지 보기)
        self.ui.tblalarminfo.cellClicked.connect(self.on_table_cell_clicked)
        
        # 날짜 단축 버튼 연결
        self.ui.pbtoday.clicked.connect(lambda: self.set_date_range(0))
        self.ui.pbweek.clicked.connect(lambda: self.set_date_range(7))
        self.ui.pb1month.clicked.connect(lambda: self.set_date_range(30))
        self.ui.pb3month.clicked.connect(lambda: self.set_date_range(90))
        self.ui.pb6month.clicked.connect(lambda: self.set_date_range(180))
        self.ui.pbyear.clicked.connect(lambda: self.set_date_range(365))

    def set_date_range(self, days):
        """날짜 단축 버튼 기능"""
        end_date = QDate.currentDate()
        start_date = end_date.addDays(-days)
        self.ui.destart.setDate(start_date)
        self.ui.deend.setDate(end_date)
        self.search_data()

    def search_data(self):
        """DB에서 조건에 맞는 데이터 조회"""
        start_date = self.ui.destart.date().toString("yyyy-MM-dd")
        end_date = self.ui.deend.date().toString("yyyy-MM-dd")
        cam_filter = self.ui.cbcam.currentText()
        grade_filter = self.ui.cbalarmgrade.currentText()
        
        query = "SELECT al_date, al_cam, al_grade, al_file FROM Alarm WHERE date(al_date) BETWEEN ? AND ?"
        params = [start_date, end_date]
        
        if cam_filter != "전체":
            query += " AND al_cam = ?"
            params.append(cam_filter)
            
        if grade_filter != "전체":
            if grade_filter == "주의":
                query += " AND al_grade IN ('주의', 'warning')"
            elif grade_filter == "위험":
                query += " AND al_grade IN ('위험', 'danger')"
        
        query += " ORDER BY al_date DESC"

        conn = None
        try:
            conn = sqlite3.connect(self.db_name)
            cursor = conn.cursor()
            cursor.execute(query, tuple(params))
            rows = cursor.fetchall()
            
            self.ui.tblalarminfo.setRowCount(0)
            
            for row_idx, row_data in enumerate(rows):
                self.ui.tblalarminfo.insertRow(row_idx)
                al_date, al_cam, al_grade, al_file = row_data
                
                display_grade = al_grade
                if al_grade == 'warning': display_grade = "주의"
                elif al_grade == 'danger': display_grade = "위험"
                
                self.ui.tblalarminfo.setItem(row_idx, 0, QTableWidgetItem(str(al_date)))
                self.ui.tblalarminfo.setItem(row_idx, 1, QTableWidgetItem(str(al_cam)))
                
                item_grade = QTableWidgetItem(display_grade)
                if display_grade == "위험":
                    item_grade.setForeground(Qt.red)
                elif display_grade == "주의":
                    item_grade.setForeground(Qt.darkYellow)
                self.ui.tblalarminfo.setItem(row_idx, 2, item_grade)
                self.ui.tblalarminfo.setItem(row_idx, 3, QTableWidgetItem(str(al_file)))
                
                for col in range(4):
                    self.ui.tblalarminfo.item(row_idx, col).setTextAlignment(Qt.AlignCenter)
            
        except sqlite3.Error as e:
            QMessageBox.critical(self.dialog, "DB 오류", f"데이터 조회 중 오류가 발생했습니다.\n{e}")
        finally:
            if conn: conn.close()


    def on_table_cell_clicked(self, row, col):
        """테이블 셀 클릭 (이미지 보기)"""
        if col == 3: # 영상정보 컬럼
            item = self.ui.tblalarminfo.item(row, col)
            if item and item.text():
                filename = item.text()
                
                # 1. DB에 전체 경로(폴더/파일)가 저장된 경우 (최신 수정 반영 시)
                #    예: "danger_images/20251217_....png"
                if "/" in filename or "\\" in filename:
                    # 현재 실행 위치 기준 상대 경로로 바로 사용
                    image_path = os.path.abspath(filename)
                
                # 2. DB에 파일명만 저장된 경우 (이전 데이터 호환용)
                #    예: "20251217_....png"
                else:
                    # 경고 폴더와 위험 폴더를 모두 뒤져봅니다.
                    base_dir = os.path.dirname(os.path.abspath(__file__))
                    path_warning = os.path.join(base_dir, "warning_images", filename)
                    path_danger = os.path.join(base_dir, "danger_images", filename)
                    
                    if os.path.exists(path_danger):
                        image_path = path_danger
                    elif os.path.exists(path_warning):
                        image_path = path_warning
                    else:
                        # 없으면 기본 설정된 폴더로 시도 (표시용)
                        image_path = os.path.join(base_dir, self.capture_dir, filename)

                # 뷰어 실행
                viewer = ImageViewerDialog(image_path, self.dialog)
                viewer.exec()
                

    # ▼▼▼ [추가된 기능] 현재 테이블 데이터를 CSV로 저장 ▼▼▼
    def save_data_to_csv(self):
        row_count = self.ui.tblalarminfo.rowCount()
        if row_count == 0:
            QMessageBox.warning(self.dialog, "알림", "저장할 데이터가 없습니다.")
            return

        # 1. 파일 저장 다이얼로그 띄우기
        file_name, _ = QFileDialog.getSaveFileName(
            self.dialog, 
            "CSV 파일로 저장", 
            "", 
            "CSV Files (*.csv);;All Files (*)"
        )

        if not file_name:
            return

        try:
            # 2. CSV 파일 쓰기 (utf-8-sig: 엑셀에서 한글 깨짐 방지)
            with open(file_name, 'w', newline='', encoding='utf-8-sig') as f:
                writer = csv.writer(f)
                
                # (1) 헤더 저장
                header_labels = []
                for col in range(self.ui.tblalarminfo.columnCount()):
                    header_item = self.ui.tblalarminfo.horizontalHeaderItem(col)
                    if header_item:
                        header_labels.append(header_item.text())
                    else:
                        header_labels.append(f"Col {col}")
                writer.writerow(header_labels)

                # (2) 데이터 행 저장
                for row in range(row_count):
                    row_data = []
                    for col in range(self.ui.tblalarminfo.columnCount()):
                        item = self.ui.tblalarminfo.item(row, col)
                        text = item.text() if item else ""
                        row_data.append(text)
                    writer.writerow(row_data)

            QMessageBox.information(self.dialog, "저장 완료", f"데이터가 성공적으로 저장되었습니다.\n{file_name}")
            
        except Exception as e:
            QMessageBox.critical(self.dialog, "저장 오류", f"파일 저장 중 오류가 발생했습니다.\n{e}")