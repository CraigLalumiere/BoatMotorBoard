# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main_window.ui'
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
from PySide6.QtWidgets import (QApplication, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QMainWindow, QPushButton,
    QSizePolicy, QSpacerItem, QStatusBar, QTabWidget,
    QTextEdit, QToolButton, QVBoxLayout, QWidget)

from pyqtgraph import GraphicsLayoutWidget
from .terminal import Terminal

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1249, 800)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout_2 = QVBoxLayout(self.centralwidget)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.gb_upper_controls = QGroupBox(self.centralwidget)
        self.gb_upper_controls.setObjectName(u"gb_upper_controls")
        self.gb_upper_controls.setMaximumSize(QSize(16777215, 60))
        self.gb_upper_controls.setFlat(False)
        self.horizontalLayout_7 = QHBoxLayout(self.gb_upper_controls)
        self.horizontalLayout_7.setSpacing(5)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.horizontalLayout_7.setContentsMargins(6, 5, 0, 5)
        self.lbl_port = QLabel(self.gb_upper_controls)
        self.lbl_port.setObjectName(u"lbl_port")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lbl_port.sizePolicy().hasHeightForWidth())
        self.lbl_port.setSizePolicy(sizePolicy)
        font = QFont()
        font.setPointSize(11)
        self.lbl_port.setFont(font)
        self.lbl_port.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.horizontalLayout_7.addWidget(self.lbl_port)

        self.txt_port = QLineEdit(self.gb_upper_controls)
        self.txt_port.setObjectName(u"txt_port")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.txt_port.sizePolicy().hasHeightForWidth())
        self.txt_port.setSizePolicy(sizePolicy1)
        self.txt_port.setMinimumSize(QSize(0, 0))
        self.txt_port.setMaximumSize(QSize(75, 16777215))
        font1 = QFont()
        font1.setFamilies([u"MS Shell Dlg 2"])
        font1.setPointSize(11)
        self.txt_port.setFont(font1)
        self.txt_port.setReadOnly(True)

        self.horizontalLayout_7.addWidget(self.txt_port)

        self.btn_port = QToolButton(self.gb_upper_controls)
        self.btn_port.setObjectName(u"btn_port")
        self.btn_port.setMinimumSize(QSize(100, 0))
        self.btn_port.setMaximumSize(QSize(100, 16777215))
        self.btn_port.setFont(font)
        self.btn_port.setStyleSheet(u"")
        self.btn_port.setAutoRaise(False)

        self.horizontalLayout_7.addWidget(self.btn_port)

        self.label = QLabel(self.gb_upper_controls)
        self.label.setObjectName(u"label")
        self.label.setFont(font)
        self.label.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.horizontalLayout_7.addWidget(self.label)

        self.txt_status = QLineEdit(self.gb_upper_controls)
        self.txt_status.setObjectName(u"txt_status")
        sizePolicy1.setHeightForWidth(self.txt_status.sizePolicy().hasHeightForWidth())
        self.txt_status.setSizePolicy(sizePolicy1)
        self.txt_status.setMinimumSize(QSize(100, 0))
        self.txt_status.setMaximumSize(QSize(80, 16777215))
        font2 = QFont()
        font2.setFamilies([u"MS Shell Dlg 2"])
        font2.setPointSize(11)
        font2.setBold(False)
        self.txt_status.setFont(font2)
        self.txt_status.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.txt_status.setCursorPosition(12)
        self.txt_status.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.txt_status.setReadOnly(True)

        self.horizontalLayout_7.addWidget(self.txt_status)

        self.btn_connect = QToolButton(self.gb_upper_controls)
        self.btn_connect.setObjectName(u"btn_connect")
        self.btn_connect.setMinimumSize(QSize(0, 0))
        self.btn_connect.setFont(font)
        self.btn_connect.setStyleSheet(u"")
        self.btn_connect.setAutoRaise(False)

        self.horizontalLayout_7.addWidget(self.btn_connect)

        self.btn_disconnect = QToolButton(self.gb_upper_controls)
        self.btn_disconnect.setObjectName(u"btn_disconnect")
        self.btn_disconnect.setMinimumSize(QSize(0, 0))
        self.btn_disconnect.setFont(font)
        self.btn_disconnect.setStyleSheet(u"")

        self.horizontalLayout_7.addWidget(self.btn_disconnect)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_7.addItem(self.horizontalSpacer)

        self.btn_open_config = QPushButton(self.gb_upper_controls)
        self.btn_open_config.setObjectName(u"btn_open_config")
        self.btn_open_config.setEnabled(True)
        sizePolicy1.setHeightForWidth(self.btn_open_config.sizePolicy().hasHeightForWidth())
        self.btn_open_config.setSizePolicy(sizePolicy1)
        self.btn_open_config.setFont(font)
        self.btn_open_config.setFocusPolicy(Qt.FocusPolicy.TabFocus)
        self.btn_open_config.setAutoDefault(False)
        self.btn_open_config.setFlat(False)

        self.horizontalLayout_7.addWidget(self.btn_open_config)

        self.btn_bootloader_clicked = QPushButton(self.gb_upper_controls)
        self.btn_bootloader_clicked.setObjectName(u"btn_bootloader_clicked")
        self.btn_bootloader_clicked.setEnabled(True)
        sizePolicy1.setHeightForWidth(self.btn_bootloader_clicked.sizePolicy().hasHeightForWidth())
        self.btn_bootloader_clicked.setSizePolicy(sizePolicy1)
        self.btn_bootloader_clicked.setFont(font)
        self.btn_bootloader_clicked.setFocusPolicy(Qt.FocusPolicy.TabFocus)
        self.btn_bootloader_clicked.setAutoDefault(False)
        self.btn_bootloader_clicked.setFlat(False)

        self.horizontalLayout_7.addWidget(self.btn_bootloader_clicked)

        self.btn_start_record = QToolButton(self.gb_upper_controls)
        self.btn_start_record.setObjectName(u"btn_start_record")
        self.btn_start_record.setMinimumSize(QSize(0, 0))
        self.btn_start_record.setFont(font)
        self.btn_start_record.setStyleSheet(u"")

        self.horizontalLayout_7.addWidget(self.btn_start_record)

        self.btn_stop_record = QToolButton(self.gb_upper_controls)
        self.btn_stop_record.setObjectName(u"btn_stop_record")
        self.btn_stop_record.setMinimumSize(QSize(0, 0))
        self.btn_stop_record.setFont(font)
        self.btn_stop_record.setStyleSheet(u"")

        self.horizontalLayout_7.addWidget(self.btn_stop_record)

        self.label_7 = QLabel(self.gb_upper_controls)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setMaximumSize(QSize(50, 16777215))

        self.horizontalLayout_7.addWidget(self.label_7)


        self.verticalLayout_2.addWidget(self.gb_upper_controls)

        self.gb_upper_controls_2 = QGroupBox(self.centralwidget)
        self.gb_upper_controls_2.setObjectName(u"gb_upper_controls_2")
        self.gb_upper_controls_2.setMaximumSize(QSize(16777215, 60))
        self.gb_upper_controls_2.setFlat(False)
        self.horizontalLayout_8 = QHBoxLayout(self.gb_upper_controls_2)
        self.horizontalLayout_8.setSpacing(5)
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.horizontalLayout_8.setContentsMargins(5, 5, 5, 5)
        self.lbl_folder = QLabel(self.gb_upper_controls_2)
        self.lbl_folder.setObjectName(u"lbl_folder")
        sizePolicy.setHeightForWidth(self.lbl_folder.sizePolicy().hasHeightForWidth())
        self.lbl_folder.setSizePolicy(sizePolicy)
        self.lbl_folder.setFont(font)
        self.lbl_folder.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.horizontalLayout_8.addWidget(self.lbl_folder)

        self.txt_data_dir = QLineEdit(self.gb_upper_controls_2)
        self.txt_data_dir.setObjectName(u"txt_data_dir")
        self.txt_data_dir.setFont(font1)

        self.horizontalLayout_8.addWidget(self.txt_data_dir)

        self.btn_browse_dir = QToolButton(self.gb_upper_controls_2)
        self.btn_browse_dir.setObjectName(u"btn_browse_dir")
        self.btn_browse_dir.setMinimumSize(QSize(110, 0))
        self.btn_browse_dir.setFont(font)
        self.btn_browse_dir.setStyleSheet(u"")
        self.btn_browse_dir.setAutoRaise(False)

        self.horizontalLayout_8.addWidget(self.btn_browse_dir)


        self.verticalLayout_2.addWidget(self.gb_upper_controls_2)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")

        self.verticalLayout_2.addLayout(self.verticalLayout_3)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.gb_left_controls = QGroupBox(self.centralwidget)
        self.gb_left_controls.setObjectName(u"gb_left_controls")
        self.gb_left_controls.setMaximumSize(QSize(700, 16777215))
        self.gb_left_controls.setFlat(False)
        self.verticalLayout = QVBoxLayout(self.gb_left_controls)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.txt_log = QTextEdit(self.gb_left_controls)
        self.txt_log.setObjectName(u"txt_log")
        font3 = QFont()
        font3.setFamilies([u"Consolas"])
        font3.setPointSize(11)
        self.txt_log.setFont(font3)
        self.txt_log.setReadOnly(True)

        self.verticalLayout.addWidget(self.txt_log)

        self.terminal = Terminal(self.gb_left_controls)
        self.terminal.setObjectName(u"terminal")
        self.terminal.setMinimumSize(QSize(0, 300))

        self.verticalLayout.addWidget(self.terminal)


        self.horizontalLayout.addWidget(self.gb_left_controls)

        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tab_5 = QWidget()
        self.tab_5.setObjectName(u"tab_5")
        self.gridLayout = QGridLayout(self.tab_5)
        self.gridLayout.setObjectName(u"gridLayout")
        self.plot_1 = GraphicsLayoutWidget(self.tab_5)
        self.plot_1.setObjectName(u"plot_1")
        self.plot_1.setMinimumSize(QSize(0, 200))

        self.gridLayout.addWidget(self.plot_1, 0, 0, 1, 1)

        self.tabWidget.addTab(self.tab_5, "")
        self.tab_6 = QWidget()
        self.tab_6.setObjectName(u"tab_6")
        self.gridLayout_2 = QGridLayout(self.tab_6)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.plot_2 = GraphicsLayoutWidget(self.tab_6)
        self.plot_2.setObjectName(u"plot_2")
        self.plot_2.setMinimumSize(QSize(0, 200))

        self.gridLayout_2.addWidget(self.plot_2, 0, 0, 1, 1)

        self.tabWidget.addTab(self.tab_6, "")

        self.horizontalLayout.addWidget(self.tabWidget)


        self.verticalLayout_2.addLayout(self.horizontalLayout)

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        self.btn_open_config.setDefault(False)
        self.btn_bootloader_clicked.setDefault(False)
        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.gb_upper_controls.setTitle("")
        self.lbl_port.setText(QCoreApplication.translate("MainWindow", u"Port:", None))
        self.btn_port.setText(QCoreApplication.translate("MainWindow", u"Select Port", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Status:", None))
        self.txt_status.setText(QCoreApplication.translate("MainWindow", u"Disconnected", None))
        self.btn_connect.setText(QCoreApplication.translate("MainWindow", u"Connect", None))
        self.btn_disconnect.setText(QCoreApplication.translate("MainWindow", u"Disconnect", None))
        self.btn_open_config.setText(QCoreApplication.translate("MainWindow", u"Open Config", None))
        self.btn_bootloader_clicked.setText(QCoreApplication.translate("MainWindow", u"Bootloader", None))
        self.btn_start_record.setText(QCoreApplication.translate("MainWindow", u"Start Record", None))
        self.btn_stop_record.setText(QCoreApplication.translate("MainWindow", u"Stop Record", None))
        self.label_7.setText("")
        self.gb_upper_controls_2.setTitle("")
        self.lbl_folder.setText(QCoreApplication.translate("MainWindow", u"Folder:", None))
        self.btn_browse_dir.setText(QCoreApplication.translate("MainWindow", u"Browse", None))
        self.gb_left_controls.setTitle("")
        self.txt_log.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Consolas'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_5), QCoreApplication.translate("MainWindow", u"Output", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_6), QCoreApplication.translate("MainWindow", u"Raw Data", None))
    # retranslateUi
