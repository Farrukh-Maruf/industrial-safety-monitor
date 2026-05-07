# clean_db.py (임시 실행용)
# alarm_gy.db, setting_gy.db

import sqlite3

def clean_alarm_table():
    try:
        conn = sqlite3.connect("alarm_gy.db")
        cursor = conn.cursor()
        
        # 데이터만 삭제 (테이블 구조는 유지)
        cursor.execute("DELETE FROM Alarm")
        conn.commit()
        print("알람 데이터가 모두 초기화되었습니다.")
        
    except Exception as e:
        print(f"초기화 중 오류: {e}")
    finally:
        if conn:
            conn.close()
            

if __name__ == "__main__":
    clean_alarm_table()