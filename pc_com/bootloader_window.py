# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'bootloader_window.ui'
##
## Created by: Qt User Interface Compiler version 6.9.1
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
from PySide6.QtWidgets import (QApplication, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QSizePolicy, QTextEdit, QToolButton,
    QWidget)

class Ui_bootloader_window(object):
    def setupUi(self, bootloader_window):
        if not bootloader_window.objectName():
            bootloader_window.setObjectName(u"bootloader_window")
        bootloader_window.resize(655, 400)
        self.btn_boot_mode = QToolButton(bootloader_window)
        self.btn_boot_mode.setObjectName(u"btn_boot_mode")
        self.btn_boot_mode.setGeometry(QRect(10, 60, 91, 26))
        self.btn_boot_mode.setMinimumSize(QSize(0, 0))
        font = QFont()
        font.setPointSize(11)
        self.btn_boot_mode.setFont(font)
        self.btn_boot_mode.setStyleSheet(u"")
        self.btn_boot_mode.setAutoRaise(False)
        self.btn_flash = QToolButton(bootloader_window)
        self.btn_flash.setObjectName(u"btn_flash")
        self.btn_flash.setGeometry(QRect(120, 60, 70, 26))
        self.btn_flash.setMinimumSize(QSize(0, 0))
        self.btn_flash.setFont(font)
        self.btn_flash.setStyleSheet(u"")
        self.btn_flash.setAutoRaise(False)
        self.btn_verify = QToolButton(bootloader_window)
        self.btn_verify.setObjectName(u"btn_verify")
        self.btn_verify.setGeometry(QRect(220, 60, 70, 26))
        self.btn_verify.setMinimumSize(QSize(0, 0))
        self.btn_verify.setFont(font)
        self.btn_verify.setStyleSheet(u"")
        self.btn_verify.setAutoRaise(False)
        self.gb_upper_controls_3 = QGroupBox(bootloader_window)
        self.gb_upper_controls_3.setObjectName(u"gb_upper_controls_3")
        self.gb_upper_controls_3.setGeometry(QRect(10, 10, 641, 38))
        self.gb_upper_controls_3.setMaximumSize(QSize(16777215, 60))
        self.gb_upper_controls_3.setFlat(False)
        self.horizontalLayout_10 = QHBoxLayout(self.gb_upper_controls_3)
        self.horizontalLayout_10.setSpacing(5)
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.horizontalLayout_10.setContentsMargins(5, 5, 5, 5)
        self.lbl_folder_3 = QLabel(self.gb_upper_controls_3)
        self.lbl_folder_3.setObjectName(u"lbl_folder_3")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lbl_folder_3.sizePolicy().hasHeightForWidth())
        self.lbl_folder_3.setSizePolicy(sizePolicy)
        self.lbl_folder_3.setFont(font)
        self.lbl_folder_3.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.horizontalLayout_10.addWidget(self.lbl_folder_3)

        self.bin_file_path = QLineEdit(self.gb_upper_controls_3)
        self.bin_file_path.setObjectName(u"bin_file_path")
        font1 = QFont()
        font1.setFamilies([u"MS Shell Dlg 2"])
        font1.setPointSize(11)
        self.bin_file_path.setFont(font1)

        self.horizontalLayout_10.addWidget(self.bin_file_path)

        self.btn_browse_path = QToolButton(self.gb_upper_controls_3)
        self.btn_browse_path.setObjectName(u"btn_browse_path")
        self.btn_browse_path.setMinimumSize(QSize(110, 0))
        self.btn_browse_path.setFont(font)
        self.btn_browse_path.setStyleSheet(u"")
        self.btn_browse_path.setAutoRaise(False)

        self.horizontalLayout_10.addWidget(self.btn_browse_path)

        self.boot_txt_log = QTextEdit(bootloader_window)
        self.boot_txt_log.setObjectName(u"boot_txt_log")
        self.boot_txt_log.setGeometry(QRect(30, 110, 595, 281))
        font2 = QFont()
        font2.setFamilies([u"Consolas"])
        font2.setPointSize(11)
        self.boot_txt_log.setFont(font2)
        self.boot_txt_log.setReadOnly(True)
        self.btn_bl_disconnect = QToolButton(bootloader_window)
        self.btn_bl_disconnect.setObjectName(u"btn_bl_disconnect")
        self.btn_bl_disconnect.setGeometry(QRect(540, 60, 91, 26))
        self.btn_bl_disconnect.setMinimumSize(QSize(0, 0))
        self.btn_bl_disconnect.setFont(font)
        self.btn_bl_disconnect.setStyleSheet(u"")
        self.btn_bl_disconnect.setAutoRaise(False)

        self.retranslateUi(bootloader_window)

        QMetaObject.connectSlotsByName(bootloader_window)
    # setupUi

    def retranslateUi(self, bootloader_window):
        bootloader_window.setWindowTitle(QCoreApplication.translate("bootloader_window", u"Form", None))
        self.btn_boot_mode.setText(QCoreApplication.translate("bootloader_window", u"Connect", None))
        self.btn_flash.setText(QCoreApplication.translate("bootloader_window", u"Flash", None))
        self.btn_verify.setText(QCoreApplication.translate("bootloader_window", u"Verify", None))
        self.gb_upper_controls_3.setTitle("")
        self.lbl_folder_3.setText(QCoreApplication.translate("bootloader_window", u".bin File:", None))
        self.btn_browse_path.setText(QCoreApplication.translate("bootloader_window", u"Browse", None))
        self.boot_txt_log.setHtml(QCoreApplication.translate("bootloader_window", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Consolas'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", None))
        self.btn_bl_disconnect.setText(QCoreApplication.translate("bootloader_window", u"Disconnect", None))
    # retranslateUi

