# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'bootloader_window.ui'
##
## Created by: Qt User Interface Compiler version 6.9.0
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
from PySide6.QtWidgets import (QApplication, QGridLayout, QHBoxLayout, QLabel,
    QLineEdit, QProgressBar, QSizePolicy, QSpacerItem,
    QTextEdit, QToolButton, QVBoxLayout, QWidget)

class Ui_bootloader_window(object):
    def setupUi(self, bootloader_window):
        if not bootloader_window.objectName():
            bootloader_window.setObjectName(u"bootloader_window")
        bootloader_window.resize(760, 520)
        bootloader_window.setMinimumSize(QSize(760, 520))
        bootloader_window.setStyleSheet(u"QWidget#bootloader_window {\n"
"    background: #f6f8fa;\n"
"}\n"
"QLabel#boot_title {\n"
"    color: #0f172a;\n"
"    font: 700 17pt \"Segoe UI\";\n"
"}\n"
"QLabel[stepState=\"done\"] {\n"
"    background: #dcfce7;\n"
"    border: 1px solid #86efac;\n"
"    color: #14532d;\n"
"}\n"
"QLabel[stepState=\"active\"] {\n"
"    background: #dbeafe;\n"
"    border: 1px solid #93c5fd;\n"
"    color: #1e3a8a;\n"
"}\n"
"QLabel[stepState=\"pending\"] {\n"
"    background: #ffffff;\n"
"    border: 1px solid #cbd5e1;\n"
"    color: #475569;\n"
"}\n"
"QLineEdit, QTextEdit {\n"
"    border: 1px solid #cbd5e1;\n"
"    border-radius: 5px;\n"
"    padding: 6px;\n"
"    background: #ffffff;\n"
"}\n"
"QToolButton {\n"
"    border: 1px solid #b8c4d2;\n"
"    border-radius: 5px;\n"
"    padding: 6px 12px;\n"
"    background: #ffffff;\n"
"}\n"
"QToolButton:hover {\n"
"    background: #f1f5f9;\n"
"}\n"
"QToolButton#btn_primary {\n"
"    background: #2563eb;\n"
"    border-color: #1d4ed8;\n"
"    color: white;\n"
"    font-weight: 700"
                        ";\n"
"}")
        self.verticalLayout = QVBoxLayout(bootloader_window)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(14, 14, 14, 14)
        self.boot_title = QLabel(bootloader_window)
        self.boot_title.setObjectName(u"boot_title")

        self.verticalLayout.addWidget(self.boot_title)

        self.fileLayout = QHBoxLayout()
        self.fileLayout.setObjectName(u"fileLayout")
        self.lbl_binary = QLabel(bootloader_window)
        self.lbl_binary.setObjectName(u"lbl_binary")

        self.fileLayout.addWidget(self.lbl_binary)

        self.bin_file_path = QLineEdit(bootloader_window)
        self.bin_file_path.setObjectName(u"bin_file_path")

        self.fileLayout.addWidget(self.bin_file_path)

        self.btn_browse_path = QToolButton(bootloader_window)
        self.btn_browse_path.setObjectName(u"btn_browse_path")
        self.btn_browse_path.setAutoRaise(False)

        self.fileLayout.addWidget(self.btn_browse_path)


        self.verticalLayout.addLayout(self.fileLayout)

        self.stepsLayout = QGridLayout()
        self.stepsLayout.setObjectName(u"stepsLayout")
        self.stepsLayout.setHorizontalSpacing(10)
        self.stepsLayout.setVerticalSpacing(8)
        self.lbl_step_serial = QLabel(bootloader_window)
        self.lbl_step_serial.setObjectName(u"lbl_step_serial")
        self.lbl_step_serial.setMinimumSize(QSize(0, 34))
        self.lbl_step_serial.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.stepsLayout.addWidget(self.lbl_step_serial, 0, 0, 1, 1)

        self.lbl_step_dfu = QLabel(bootloader_window)
        self.lbl_step_dfu.setObjectName(u"lbl_step_dfu")
        self.lbl_step_dfu.setMinimumSize(QSize(0, 34))
        self.lbl_step_dfu.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.stepsLayout.addWidget(self.lbl_step_dfu, 0, 1, 1, 1)

        self.lbl_step_file = QLabel(bootloader_window)
        self.lbl_step_file.setObjectName(u"lbl_step_file")
        self.lbl_step_file.setMinimumSize(QSize(0, 34))
        self.lbl_step_file.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.stepsLayout.addWidget(self.lbl_step_file, 1, 0, 1, 1)

        self.lbl_step_flash = QLabel(bootloader_window)
        self.lbl_step_flash.setObjectName(u"lbl_step_flash")
        self.lbl_step_flash.setMinimumSize(QSize(0, 34))
        self.lbl_step_flash.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.stepsLayout.addWidget(self.lbl_step_flash, 1, 1, 1, 1)


        self.verticalLayout.addLayout(self.stepsLayout)

        self.buttonLayout = QHBoxLayout()
        self.buttonLayout.setObjectName(u"buttonLayout")
        self.buttonSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.buttonLayout.addItem(self.buttonSpacer)

        self.btn_primary = QToolButton(bootloader_window)
        self.btn_primary.setObjectName(u"btn_primary")
        self.btn_primary.setMinimumSize(QSize(0, 32))
        self.btn_primary.setAutoRaise(False)

        self.buttonLayout.addWidget(self.btn_primary)

        self.btn_verify = QToolButton(bootloader_window)
        self.btn_verify.setObjectName(u"btn_verify")
        self.btn_verify.setMinimumSize(QSize(0, 32))
        self.btn_verify.setAutoRaise(False)

        self.buttonLayout.addWidget(self.btn_verify)

        self.btn_reboot = QToolButton(bootloader_window)
        self.btn_reboot.setObjectName(u"btn_reboot")
        self.btn_reboot.setMinimumSize(QSize(0, 32))
        self.btn_reboot.setAutoRaise(False)

        self.buttonLayout.addWidget(self.btn_reboot)

        self.btn_close = QToolButton(bootloader_window)
        self.btn_close.setObjectName(u"btn_close")
        self.btn_close.setMinimumSize(QSize(0, 32))
        self.btn_close.setAutoRaise(False)

        self.buttonLayout.addWidget(self.btn_close)


        self.verticalLayout.addLayout(self.buttonLayout)

        self.progress_boot = QProgressBar(bootloader_window)
        self.progress_boot.setObjectName(u"progress_boot")
        self.progress_boot.setMaximum(1)
        self.progress_boot.setValue(0)
        self.progress_boot.setTextVisible(False)

        self.verticalLayout.addWidget(self.progress_boot)

        self.boot_txt_log = QTextEdit(bootloader_window)
        self.boot_txt_log.setObjectName(u"boot_txt_log")
        font = QFont()
        font.setFamilies([u"Consolas"])
        self.boot_txt_log.setFont(font)
        self.boot_txt_log.setMinimumSize(QSize(0, 230))
        self.boot_txt_log.setReadOnly(True)

        self.verticalLayout.addWidget(self.boot_txt_log)


        self.retranslateUi(bootloader_window)

        QMetaObject.connectSlotsByName(bootloader_window)
    # setupUi

    def retranslateUi(self, bootloader_window):
        bootloader_window.setWindowTitle(QCoreApplication.translate("bootloader_window", u"Bootloader Tool", None))
        self.boot_title.setText(QCoreApplication.translate("bootloader_window", u"Firmware Update", None))
        self.lbl_binary.setText(QCoreApplication.translate("bootloader_window", u"Binary", None))
        self.bin_file_path.setPlaceholderText(QCoreApplication.translate("bootloader_window", u"Select a .bin firmware image", None))
        self.btn_browse_path.setText(QCoreApplication.translate("bootloader_window", u"Browse", None))
        self.lbl_step_serial.setText(QCoreApplication.translate("bootloader_window", u"1. Serial connected", None))
        self.lbl_step_serial.setProperty(u"stepState", QCoreApplication.translate("bootloader_window", u"pending", None))
        self.lbl_step_dfu.setText(QCoreApplication.translate("bootloader_window", u"2. DFU detected", None))
        self.lbl_step_dfu.setProperty(u"stepState", QCoreApplication.translate("bootloader_window", u"pending", None))
        self.lbl_step_file.setText(QCoreApplication.translate("bootloader_window", u"3. Firmware selected", None))
        self.lbl_step_file.setProperty(u"stepState", QCoreApplication.translate("bootloader_window", u"pending", None))
        self.lbl_step_flash.setText(QCoreApplication.translate("bootloader_window", u"4. Flash and verify", None))
        self.lbl_step_flash.setProperty(u"stepState", QCoreApplication.translate("bootloader_window", u"pending", None))
        self.btn_primary.setText(QCoreApplication.translate("bootloader_window", u"Enter DFU", None))
        self.btn_verify.setText(QCoreApplication.translate("bootloader_window", u"Verify", None))
        self.btn_reboot.setText(QCoreApplication.translate("bootloader_window", u"Reboot To App", None))
        self.btn_close.setText(QCoreApplication.translate("bootloader_window", u"Close", None))
    # retranslateUi

