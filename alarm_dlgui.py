# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'alarm_dlg.ui'
##
## Created by: Qt User Interface Compiler version 6.10.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QDateEdit, QDialog,
    QHeaderView, QLabel, QLineEdit, QPushButton,
    QSizePolicy, QSplitter, QTableWidget, QTableWidgetItem,
    QWidget)

class Ui_alarmDialog(object):
    def setupUi(self, alarmDialog):
        if not alarmDialog.objectName():
            alarmDialog.setObjectName(u"alarmDialog")
        alarmDialog.resize(955, 606)
        alarmDialog.setStyleSheet(u"QDialog{\n"
"	background-color: white;\n"
"}\n"
"\n"
"QLineEdit{\n"
"	border: 1px solid gray;\n"
"	border-radius: 6px;\n"
"	padding-left: 15px;\n"
"	height: 35px;\n"
"}")
        self.tblalarminfo = QTableWidget(alarmDialog)
        if (self.tblalarminfo.columnCount() < 4):
            self.tblalarminfo.setColumnCount(4)
        font = QFont()
        font.setFamilies([u"Malgun Gothic"])
        font.setPointSize(12)
        __qtablewidgetitem = QTableWidgetItem()
        __qtablewidgetitem.setFont(font);
        self.tblalarminfo.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        __qtablewidgetitem1.setFont(font);
        self.tblalarminfo.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        __qtablewidgetitem2.setFont(font);
        self.tblalarminfo.setHorizontalHeaderItem(2, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        __qtablewidgetitem3.setFont(font);
        self.tblalarminfo.setHorizontalHeaderItem(3, __qtablewidgetitem3)
        self.tblalarminfo.setObjectName(u"tblalarminfo")
        self.tblalarminfo.setGeometry(QRect(20, 120, 911, 461))
        self.tblalarminfo.setStyleSheet(u"QTableWidget{\n"
"    background-color: #f0f0f0;\n"
"    border: 1px solid #c0c0c0;\n"
"    font-family: Arial;\n"
"    font-size: 12pt;\n"
"    gridline-color: #d3d3d3;\n"
"}\n"
"\n"
"QHeaderView::section:horizontal {\n"
"    background-color: #008cba;\n"
"    color: white;\n"
"    font-weight: bold;\n"
"    border: 1px solid #5a5a5a;\n"
"    padding: 5px;\n"
"}")
        self.label = QLabel(alarmDialog)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(30, 20, 51, 16))
        font1 = QFont()
        font1.setFamilies([u"\ud568\ucd08\ub86c\ub3cb\uc6c0"])
        font1.setPointSize(10)
        font1.setBold(True)
        self.label.setFont(font1)
        self.label_2 = QLabel(alarmDialog)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(30, 70, 51, 21))
        self.label_2.setFont(font1)
        self.destart = QDateEdit(alarmDialog)
        self.destart.setObjectName(u"destart")
        self.destart.setGeometry(QRect(90, 20, 110, 22))
        self.destart.setDateTime(QDateTime(QDate(2025, 1, 2), QTime(0, 0, 0)))
        self.destart.setDate(QDate(2025, 1, 2))
        self.deend = QDateEdit(alarmDialog)
        self.deend.setObjectName(u"deend")
        self.deend.setGeometry(QRect(230, 20, 110, 22))
        self.deend.setDateTime(QDateTime(QDate(2025, 1, 31), QTime(0, 0, 0)))
        self.deend.setDate(QDate(2025, 1, 31))
        self.cbcam = QComboBox(alarmDialog)
        self.cbcam.addItem("")
        self.cbcam.addItem("")
        self.cbcam.addItem("")
        self.cbcam.addItem("")
        self.cbcam.addItem("")
        self.cbcam.addItem("")
        self.cbcam.setObjectName(u"cbcam")
        self.cbcam.setGeometry(QRect(90, 70, 111, 22))
        font2 = QFont()
        font2.setFamilies([u"\uad74\ub9bc"])
        font2.setPointSize(10)
        self.cbcam.setFont(font2)
        self.label_3 = QLabel(alarmDialog)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(200, 20, 31, 16))
        font3 = QFont()
        font3.setFamilies([u"Arial"])
        font3.setPointSize(11)
        font3.setBold(True)
        self.label_3.setFont(font3)
        self.label_3.setLayoutDirection(Qt.LeftToRight)
        self.label_3.setAlignment(Qt.AlignCenter)
        self.label_4 = QLabel(alarmDialog)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(230, 70, 61, 21))
        self.label_4.setFont(font1)
        self.cbalarmgrade = QComboBox(alarmDialog)
        self.cbalarmgrade.addItem("")
        self.cbalarmgrade.addItem("")
        self.cbalarmgrade.addItem("")
        self.cbalarmgrade.setObjectName(u"cbalarmgrade")
        self.cbalarmgrade.setGeometry(QRect(300, 70, 111, 22))
        self.cbalarmgrade.setFont(font2)
        self.label_5 = QLabel(alarmDialog)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(440, 70, 61, 21))
        self.label_5.setFont(font1)
        self.lealarmcontent = QLineEdit(alarmDialog)
        self.lealarmcontent.setObjectName(u"lealarmcontent")
        self.lealarmcontent.setGeometry(QRect(510, 70, 211, 25))
        self.splitter = QSplitter(alarmDialog)
        self.splitter.setObjectName(u"splitter")
        self.splitter.setGeometry(QRect(500, 20, 421, 28))
        self.splitter.setOrientation(Qt.Horizontal)
        self.pbtoday = QPushButton(self.splitter)
        self.pbtoday.setObjectName(u"pbtoday")
        font4 = QFont()
        font4.setFamilies([u"\ub098\ub214\uace0\ub515"])
        font4.setPointSize(10)
        font4.setBold(True)
        self.pbtoday.setFont(font4)
        self.splitter.addWidget(self.pbtoday)
        self.pbweek = QPushButton(self.splitter)
        self.pbweek.setObjectName(u"pbweek")
        self.pbweek.setFont(font4)
        self.splitter.addWidget(self.pbweek)
        self.pb1month = QPushButton(self.splitter)
        self.pb1month.setObjectName(u"pb1month")
        self.pb1month.setFont(font4)
        self.splitter.addWidget(self.pb1month)
        self.pb3month = QPushButton(self.splitter)
        self.pb3month.setObjectName(u"pb3month")
        self.pb3month.setFont(font4)
        self.splitter.addWidget(self.pb3month)
        self.pb6month = QPushButton(self.splitter)
        self.pb6month.setObjectName(u"pb6month")
        self.pb6month.setFont(font4)
        self.splitter.addWidget(self.pb6month)
        self.pbyear = QPushButton(self.splitter)
        self.pbyear.setObjectName(u"pbyear")
        self.pbyear.setFont(font4)
        self.splitter.addWidget(self.pbyear)
        self.pbsavedata = QPushButton(alarmDialog)
        self.pbsavedata.setObjectName(u"pbsavedata")
        self.pbsavedata.setGeometry(QRect(790, 67, 131, 31))
        font5 = QFont()
        font5.setFamilies([u"Arial"])
        font5.setPointSize(11)
        font5.setBold(False)
        self.pbsavedata.setFont(font5)
        self.pbsearch = QPushButton(alarmDialog)
        self.pbsearch.setObjectName(u"pbsearch")
        self.pbsearch.setGeometry(QRect(350, 20, 101, 28))
        self.pbsearch.setFont(font4)

        self.retranslateUi(alarmDialog)

        QMetaObject.connectSlotsByName(alarmDialog)
    # setupUi

    def retranslateUi(self, alarmDialog):
        alarmDialog.setWindowTitle(QCoreApplication.translate("alarmDialog", u"Alarm Information", None))
        ___qtablewidgetitem = self.tblalarminfo.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("alarmDialog", u"\uc2dc\uac04", None));
        ___qtablewidgetitem1 = self.tblalarminfo.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("alarmDialog", u"\uce74\uba54\ub77c", None));
        ___qtablewidgetitem2 = self.tblalarminfo.horizontalHeaderItem(2)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("alarmDialog", u"\uc54c\ub78c\ub4f1\uae09", None));
        ___qtablewidgetitem3 = self.tblalarminfo.horizontalHeaderItem(3)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("alarmDialog", u"\uc601\uc0c1\uc815\ubcf4", None));
        self.label.setText(QCoreApplication.translate("alarmDialog", u"\uae30\uac04 :", None))
        self.label_2.setText(QCoreApplication.translate("alarmDialog", u"\uce74\uba54\ub77c :", None))
        self.cbcam.setItemText(0, QCoreApplication.translate("alarmDialog", u"\uc804\uccb4", None))
        self.cbcam.setItemText(1, QCoreApplication.translate("alarmDialog", u"CAM1", None))
        self.cbcam.setItemText(2, QCoreApplication.translate("alarmDialog", u"CAM2", None))
        self.cbcam.setItemText(3, QCoreApplication.translate("alarmDialog", u"CAM3", None))
        self.cbcam.setItemText(4, QCoreApplication.translate("alarmDialog", u"CAM4", None))
        self.cbcam.setItemText(5, QCoreApplication.translate("alarmDialog", u"CAM5", None))

        self.label_3.setText(QCoreApplication.translate("alarmDialog", u"~", None))
        self.label_4.setText(QCoreApplication.translate("alarmDialog", u"\uc54c\ub78c\ub4f1\uae09 :", None))
        self.cbalarmgrade.setItemText(0, QCoreApplication.translate("alarmDialog", u"\uc804\uccb4", None))
        self.cbalarmgrade.setItemText(1, QCoreApplication.translate("alarmDialog", u"\uc8fc\uc758", None))
        self.cbalarmgrade.setItemText(2, QCoreApplication.translate("alarmDialog", u"\uc704\ud5d8", None))

        self.label_5.setText(QCoreApplication.translate("alarmDialog", u"\uc54c\ub78c\ub0b4\uc6a9 :", None))
        self.pbtoday.setText(QCoreApplication.translate("alarmDialog", u"\uc624\ub298", None))
        self.pbweek.setText(QCoreApplication.translate("alarmDialog", u"1\uc8fc", None))
        self.pb1month.setText(QCoreApplication.translate("alarmDialog", u"1\uac1c\uc6d4", None))
        self.pb3month.setText(QCoreApplication.translate("alarmDialog", u"3\uac1c\uc6d4", None))
        self.pb6month.setText(QCoreApplication.translate("alarmDialog", u"6\uac1c\uc6d4", None))
        self.pbyear.setText(QCoreApplication.translate("alarmDialog", u"1\ub144", None))
        self.pbsavedata.setText(QCoreApplication.translate("alarmDialog", u"Save Data", None))
        self.pbsearch.setText(QCoreApplication.translate("alarmDialog", u"\ucc3e\uae30", None))
    # retranslateUi

