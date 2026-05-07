# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainwindow_gy.ui'
##
## Created by: Qt User Interface Compiler version 6.10.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QGridLayout, QLabel,
    QMainWindow, QMenu, QMenuBar, QSizePolicy,
    QStatusBar, QWidget)

class Ui_mainWindow(object):
    def setupUi(self, mainWindow):
        if not mainWindow.objectName():
            mainWindow.setObjectName(u"mainWindow")
        mainWindow.resize(1920, 1080)
        mainWindow.setAutoFillBackground(False)
        mainWindow.setStyleSheet(u"")
        self.actionQuit = QAction(mainWindow)
        self.actionQuit.setObjectName(u"actionQuit")
        self.actionAbout = QAction(mainWindow)
        self.actionAbout.setObjectName(u"actionAbout")
        self.actionCamera = QAction(mainWindow)
        self.actionCamera.setObjectName(u"actionCamera")
        self.actionAlarmInfo = QAction(mainWindow)
        self.actionAlarmInfo.setObjectName(u"actionAlarmInfo")
        self.actionSetting = QAction(mainWindow)
        self.actionSetting.setObjectName(u"actionSetting")
        self.actionLidar = QAction(mainWindow)
        self.actionLidar.setObjectName(u"actionLidar")
        self.actionMapping = QAction(mainWindow)
        self.actionMapping.setObjectName(u"actionMapping")
        self.centralwidget = QWidget(mainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayoutWidget = QWidget(self.centralwidget)
        self.gridLayoutWidget.setObjectName(u"gridLayoutWidget")
        self.gridLayoutWidget.setGeometry(QRect(0, 0, 1911, 1041))
        self.gridLayout = QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setSpacing(5)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(5, 0, 5, 5)
        self.lblcam5 = QLabel(self.gridLayoutWidget)
        self.lblcam5.setObjectName(u"lblcam5")
        self.lblcam5.setFrameShape(QFrame.Box)
        self.lblcam5.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lblcam5, 1, 1, 1, 1)

        self.lblcam4 = QLabel(self.gridLayoutWidget)
        self.lblcam4.setObjectName(u"lblcam4")
        self.lblcam4.setFrameShape(QFrame.Box)
        self.lblcam4.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lblcam4, 0, 2, 1, 1)

        self.lblcam3 = QLabel(self.gridLayoutWidget)
        self.lblcam3.setObjectName(u"lblcam3")
        self.lblcam3.setFrameShape(QFrame.Box)
        self.lblcam3.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lblcam3, 0, 1, 1, 1)

        self.lblcam1 = QLabel(self.gridLayoutWidget)
        self.lblcam1.setObjectName(u"lblcam1")
        self.lblcam1.setFrameShape(QFrame.Box)
        self.lblcam1.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lblcam1, 1, 0, 1, 1)

        self.lbllidar = QLabel(self.gridLayoutWidget)
        self.lbllidar.setObjectName(u"lbllidar")
        self.lbllidar.setFrameShape(QFrame.Box)
        self.lbllidar.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lbllidar, 1, 2, 1, 1)

        self.lblcam2 = QLabel(self.gridLayoutWidget)
        self.lblcam2.setObjectName(u"lblcam2")
        self.lblcam2.setFrameShape(QFrame.Box)
        self.lblcam2.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lblcam2, 0, 0, 1, 1)

        self.lbltmap = QLabel(self.gridLayoutWidget)
        self.lbltmap.setObjectName(u"lbltmap")
        self.lbltmap.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lbltmap, 2, 1, 1, 1)

        mainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(mainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1920, 26))
        self.menuCamera = QMenu(self.menubar)
        self.menuCamera.setObjectName(u"menuCamera")
        self.menuAlarm = QMenu(self.menubar)
        self.menuAlarm.setObjectName(u"menuAlarm")
        self.menuSettings = QMenu(self.menubar)
        self.menuSettings.setObjectName(u"menuSettings")
        self.menuAbout = QMenu(self.menubar)
        self.menuAbout.setObjectName(u"menuAbout")
        mainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(mainWindow)
        self.statusbar.setObjectName(u"statusbar")
        mainWindow.setStatusBar(self.statusbar)

        self.menubar.addAction(self.menuCamera.menuAction())
        self.menubar.addAction(self.menuAlarm.menuAction())
        self.menubar.addAction(self.menuSettings.menuAction())
        self.menubar.addAction(self.menuAbout.menuAction())
        self.menuCamera.addAction(self.actionCamera)
        self.menuCamera.addAction(self.actionLidar)
        self.menuCamera.addAction(self.actionMapping)
        self.menuCamera.addSeparator()
        self.menuCamera.addAction(self.actionQuit)
        self.menuAlarm.addAction(self.actionAlarmInfo)
        self.menuSettings.addAction(self.actionSetting)
        self.menuAbout.addAction(self.actionAbout)

        self.retranslateUi(mainWindow)

        QMetaObject.connectSlotsByName(mainWindow)
    # setupUi

    def retranslateUi(self, mainWindow):
        mainWindow.setWindowTitle(QCoreApplication.translate("mainWindow", u"GYsteel", None))
        self.actionQuit.setText(QCoreApplication.translate("mainWindow", u"Quit", None))
        self.actionAbout.setText(QCoreApplication.translate("mainWindow", u"About", None))
        self.actionCamera.setText(QCoreApplication.translate("mainWindow", u"Camera", None))
        self.actionAlarmInfo.setText(QCoreApplication.translate("mainWindow", u"Infomation", None))
        self.actionSetting.setText(QCoreApplication.translate("mainWindow", u"Setting", None))
        self.actionLidar.setText(QCoreApplication.translate("mainWindow", u"Lidar", None))
        self.actionMapping.setText(QCoreApplication.translate("mainWindow", u"Mapping", None))
        self.lblcam5.setText(QCoreApplication.translate("mainWindow", u"cam5", None))
        self.lblcam4.setText(QCoreApplication.translate("mainWindow", u"cam4", None))
        self.lblcam3.setText(QCoreApplication.translate("mainWindow", u"cam3", None))
        self.lblcam1.setText(QCoreApplication.translate("mainWindow", u"cam1", None))
        self.lbllidar.setText(QCoreApplication.translate("mainWindow", u"lidar", None))
        self.lblcam2.setText(QCoreApplication.translate("mainWindow", u"cam2", None))
        self.lbltmap.setText(QCoreApplication.translate("mainWindow", u"Tmap", None))
        self.menuCamera.setTitle(QCoreApplication.translate("mainWindow", u"\ubaa8\ub2c8\ud130\ub9c1", None))
        self.menuAlarm.setTitle(QCoreApplication.translate("mainWindow", u"\uc54c\ub78c\uae30\ub85d", None))
        self.menuSettings.setTitle(QCoreApplication.translate("mainWindow", u"\uc124\uc815", None))
        self.menuAbout.setTitle(QCoreApplication.translate("mainWindow", u"\ub3c4\uc6c0\ub9d0", None))
    # retranslateUi

