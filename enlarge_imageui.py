# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'enlarge_image.ui'
##
## Created by: Qt User Interface Compiler version 6.9.2
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
from PySide6.QtWidgets import (QApplication, QDialog, QFrame, QHBoxLayout,
    QLabel, QPushButton, QSizePolicy, QVBoxLayout,
    QWidget)

class Ui_enlargeDialog(object):
    def setupUi(self, enlargeDialog):
        if not enlargeDialog.objectName():
            enlargeDialog.setObjectName(u"enlargeDialog")
        enlargeDialog.resize(1111, 640)
        self.verticalLayoutWidget_2 = QWidget(enlargeDialog)
        self.verticalLayoutWidget_2.setObjectName(u"verticalLayoutWidget_2")
        self.verticalLayoutWidget_2.setGeometry(QRect(30, 60, 1051, 561))
        self.verticalLayout_2 = QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.lblenlarge = QLabel(self.verticalLayoutWidget_2)
        self.lblenlarge.setObjectName(u"lblenlarge")
        self.lblenlarge.setMaximumSize(QSize(1280, 720))
        self.lblenlarge.setFrameShape(QFrame.Box)
        self.lblenlarge.setScaledContents(True)
        self.lblenlarge.setAlignment(Qt.AlignCenter)

        self.verticalLayout_2.addWidget(self.lblenlarge)

        self.layoutWidget = QWidget(enlargeDialog)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(30, 10, 1051, 37))
        self.horizontalLayout_2 = QHBoxLayout(self.layoutWidget)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.pbnrecordstart = QPushButton(self.layoutWidget)
        self.pbnrecordstart.setObjectName(u"pbnrecordstart")
        font = QFont()
        font.setFamilies([u"Arial"])
        font.setPointSize(12)
        font.setBold(True)
        self.pbnrecordstart.setFont(font)

        self.horizontalLayout.addWidget(self.pbnrecordstart)

        self.pbnrecordstop = QPushButton(self.layoutWidget)
        self.pbnrecordstop.setObjectName(u"pbnrecordstop")
        self.pbnrecordstop.setFont(font)

        self.horizontalLayout.addWidget(self.pbnrecordstop)

        self.pbnimgcapture = QPushButton(self.layoutWidget)
        self.pbnimgcapture.setObjectName(u"pbnimgcapture")
        self.pbnimgcapture.setFont(font)

        self.horizontalLayout.addWidget(self.pbnimgcapture)


        self.horizontalLayout_2.addLayout(self.horizontalLayout)

        self.lblstatus = QLabel(self.layoutWidget)
        self.lblstatus.setObjectName(u"lblstatus")
        self.lblstatus.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.horizontalLayout_2.addWidget(self.lblstatus)


        self.retranslateUi(enlargeDialog)

        QMetaObject.connectSlotsByName(enlargeDialog)
    # setupUi

    def retranslateUi(self, enlargeDialog):
        enlargeDialog.setWindowTitle(QCoreApplication.translate("enlargeDialog", u"Enlarge Image", None))
        self.lblenlarge.setText("")
        self.pbnrecordstart.setText(QCoreApplication.translate("enlargeDialog", u"\uc601\uc0c1 \ub179\ud654", None))
        self.pbnrecordstop.setText(QCoreApplication.translate("enlargeDialog", u"\ub179\ud654 \uc911\uc9c0", None))
        self.pbnimgcapture.setText(QCoreApplication.translate("enlargeDialog", u"\uc774\ubbf8\uc9c0 \ucea1\uccd0", None))
        self.lblstatus.setText("")
    # retranslateUi

