# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'config_window.ui'
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
from PySide6.QtWidgets import (QApplication, QGroupBox, QHBoxLayout, QMainWindow,
    QScrollArea, QSizePolicy, QSpacerItem, QStatusBar,
    QToolButton, QVBoxLayout, QWidget)

class Ui_ConfigWindow(object):
    def setupUi(self, ConfigWindow):
        if not ConfigWindow.objectName():
            ConfigWindow.setObjectName(u"ConfigWindow")
        ConfigWindow.resize(800, 600)
        self.centralwidget = QWidget(ConfigWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.gb_config_controls = QGroupBox(self.centralwidget)
        self.gb_config_controls.setObjectName(u"gb_config_controls")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.gb_config_controls.sizePolicy().hasHeightForWidth())
        self.gb_config_controls.setSizePolicy(sizePolicy)
        self.gb_config_controls.setMinimumSize(QSize(60, 0))
        self.gb_config_controls.setMaximumSize(QSize(16777215, 60))
        self.gb_config_controls.setFlat(False)
        self.horizontalLayout = QHBoxLayout(self.gb_config_controls)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalSpacer = QSpacerItem(547, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)

        self.btn_load_config_from_file = QToolButton(self.gb_config_controls)
        self.btn_load_config_from_file.setObjectName(u"btn_load_config_from_file")
        self.btn_load_config_from_file.setMinimumSize(QSize(170, 0))
        font = QFont()
        font.setPointSize(11)
        self.btn_load_config_from_file.setFont(font)
        self.btn_load_config_from_file.setStyleSheet(u"")

        self.horizontalLayout.addWidget(self.btn_load_config_from_file)

        self.btn_save_config_to_file = QToolButton(self.gb_config_controls)
        self.btn_save_config_to_file.setObjectName(u"btn_save_config_to_file")
        self.btn_save_config_to_file.setMinimumSize(QSize(170, 0))
        self.btn_save_config_to_file.setFont(font)
        self.btn_save_config_to_file.setStyleSheet(u"")

        self.horizontalLayout.addWidget(self.btn_save_config_to_file)

        self.btn_commit = QToolButton(self.gb_config_controls)
        self.btn_commit.setObjectName(u"btn_commit")
        self.btn_commit.setMinimumSize(QSize(120, 0))
        self.btn_commit.setFont(font)
        self.btn_commit.setStyleSheet(u"")

        self.horizontalLayout.addWidget(self.btn_commit)


        self.verticalLayout.addWidget(self.gb_config_controls)

        self.scroll_frame_config_viewer = QScrollArea(self.centralwidget)
        self.scroll_frame_config_viewer.setObjectName(u"scroll_frame_config_viewer")
        self.scroll_frame_config_viewer.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll_frame_config_viewer.setWidgetResizable(True)
        self.frame_config_viewer = QWidget()
        self.frame_config_viewer.setObjectName(u"frame_config_viewer")
        self.frame_config_viewer.setGeometry(QRect(0, 0, 780, 507))
        self.scroll_frame_config_viewer.setWidget(self.frame_config_viewer)

        self.verticalLayout.addWidget(self.scroll_frame_config_viewer)

        ConfigWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(ConfigWindow)
        self.statusbar.setObjectName(u"statusbar")
        ConfigWindow.setStatusBar(self.statusbar)

        self.retranslateUi(ConfigWindow)

        QMetaObject.connectSlotsByName(ConfigWindow)
    # setupUi

    def retranslateUi(self, ConfigWindow):
        ConfigWindow.setWindowTitle(QCoreApplication.translate("ConfigWindow", u"MainWindow", None))
        self.gb_config_controls.setTitle("")
        self.btn_load_config_from_file.setText(QCoreApplication.translate("ConfigWindow", u"Load Config From File", None))
        self.btn_save_config_to_file.setText(QCoreApplication.translate("ConfigWindow", u"Save Config To File", None))
        self.btn_commit.setText(QCoreApplication.translate("ConfigWindow", u"Commit", None))
    # retranslateUi

