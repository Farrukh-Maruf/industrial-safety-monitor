#main_gy.py
# 9/3 : widget.py에서 카메라 영상부분 별도로 분리(camera.py)
# 9/4 : alarm 창 추가
# 9/9 : 기존에 *.ui를 load 하던 방식에서 *ui.py로 변환하여 load, alarm 창 부분 수정


import os
import sys
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt
from widget_gy import widget


def main():
    # 2) .ui 경로 안전성(선택) — 실행 디렉터리를 스크립트 위치로 고정
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    print("프로그램 시작")
    app = QApplication(sys.argv) 

    window = widget(app)
    window.show()

    # 3) 종료 코드 일관화(선택)
    exit_code = app.exec()
    print("프로그램 종료")
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
