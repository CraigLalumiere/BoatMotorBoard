# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'config_window.ui'
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
from PySide6.QtWidgets import (QAbstractItemView, QApplication, QGroupBox, QHBoxLayout,
    QHeaderView, QLabel, QLineEdit, QMainWindow,
    QProgressBar, QSizePolicy, QSpacerItem, QStatusBar,
    QTableView, QToolButton, QVBoxLayout, QWidget)

class Ui_ConfigWindow(object):
    def setupUi(self, ConfigWindow):
        if not ConfigWindow.objectName():
            ConfigWindow.setObjectName(u"ConfigWindow")
        ConfigWindow.resize(980, 620)
        ConfigWindow.setMinimumSize(QSize(980, 620))
        ConfigWindow.setStyleSheet(u"QMainWindow {\n"
"    background: #f6f8fa;\n"
"}\n"
"QGroupBox {\n"
"    border: 0;\n"
"    background: #eef2f6;\n"
"}\n"
"QTableView {\n"
"    background: #ffffff;\n"
"    alternate-background-color: #f8fafc;\n"
"    border: 1px solid #cbd5e1;\n"
"    gridline-color: #e2e8f0;\n"
"    selection-background-color: #dbeafe;\n"
"    selection-color: #0f172a;\n"
"}\n"
"QHeaderView::section {\n"
"    background: #e8eef5;\n"
"    border: 0;\n"
"    border-right: 1px solid #cbd5e1;\n"
"    padding: 7px;\n"
"    font-weight: 700;\n"
"}\n"
"QLineEdit {\n"
"    border: 1px solid #cbd5e1;\n"
"    border-radius: 5px;\n"
"    padding: 5px 8px;\n"
"    background: #ffffff;\n"
"}\n"
"QToolButton {\n"
"    border: 1px solid #b8c4d2;\n"
"    border-radius: 5px;\n"
"    padding: 5px 10px;\n"
"    background: #ffffff;\n"
"}\n"
"QToolButton:hover {\n"
"    background: #f1f5f9;\n"
"}\n"
"QToolButton#btn_apply_config {\n"
"    background: #2563eb;\n"
"    border-color: #1d4ed8;\n"
"    color: white;\n"
"    font-weight: 700;\n"
"}\n"
""
                        "QToolButton#btn_commit {\n"
"    background: #0f766e;\n"
"    border-color: #0f766e;\n"
"    color: white;\n"
"    font-weight: 700;\n"
"}\n"
"QLabel#lbl_config_dirty {\n"
"    color: #334155;\n"
"    font-weight: 700;\n"
"    background: #e8eef5;\n"
"    border: 1px solid #cbd5e1;\n"
"    border-radius: 5px;\n"
"    padding: 5px 8px;\n"
"}")
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
        self.gb_config_controls.setMaximumSize(QSize(16777215, 70))
        self.horizontalLayout = QHBoxLayout(self.gb_config_controls)
        self.horizontalLayout.setSpacing(8)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(8, 8, 8, 8)
        self.txt_config_search = QLineEdit(self.gb_config_controls)
        self.txt_config_search.setObjectName(u"txt_config_search")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.txt_config_search.sizePolicy().hasHeightForWidth())
        self.txt_config_search.setSizePolicy(sizePolicy1)
        self.txt_config_search.setMinimumSize(QSize(220, 0))
        self.txt_config_search.setMaximumSize(QSize(260, 16777215))
        self.txt_config_search.setClearButtonEnabled(True)

        self.horizontalLayout.addWidget(self.txt_config_search)

        self.lbl_config_dirty = QLabel(self.gb_config_controls)
        self.lbl_config_dirty.setObjectName(u"lbl_config_dirty")
        sizePolicy1.setHeightForWidth(self.lbl_config_dirty.sizePolicy().hasHeightForWidth())
        self.lbl_config_dirty.setSizePolicy(sizePolicy1)
        self.lbl_config_dirty.setMinimumSize(QSize(140, 0))
        self.lbl_config_dirty.setMaximumSize(QSize(160, 16777215))
        self.lbl_config_dirty.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout.addWidget(self.lbl_config_dirty)

        self.progress_config_load = QProgressBar(self.gb_config_controls)
        self.progress_config_load.setObjectName(u"progress_config_load")
        self.progress_config_load.setVisible(False)
        self.progress_config_load.setMaximumSize(QSize(90, 16777215))
        self.progress_config_load.setMaximum(1)
        self.progress_config_load.setValue(0)
        self.progress_config_load.setTextVisible(False)

        self.horizontalLayout.addWidget(self.progress_config_load)

        self.horizontalSpacer = QSpacerItem(20, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)

        self.btn_refresh_config = QToolButton(self.gb_config_controls)
        self.btn_refresh_config.setObjectName(u"btn_refresh_config")
        self.btn_refresh_config.setMinimumSize(QSize(0, 30))

        self.horizontalLayout.addWidget(self.btn_refresh_config)

        self.btn_load_config_from_file = QToolButton(self.gb_config_controls)
        self.btn_load_config_from_file.setObjectName(u"btn_load_config_from_file")
        self.btn_load_config_from_file.setMinimumSize(QSize(0, 30))

        self.horizontalLayout.addWidget(self.btn_load_config_from_file)

        self.btn_save_config_to_file = QToolButton(self.gb_config_controls)
        self.btn_save_config_to_file.setObjectName(u"btn_save_config_to_file")
        self.btn_save_config_to_file.setMinimumSize(QSize(0, 30))

        self.horizontalLayout.addWidget(self.btn_save_config_to_file)

        self.btn_apply_config = QToolButton(self.gb_config_controls)
        self.btn_apply_config.setObjectName(u"btn_apply_config")
        self.btn_apply_config.setMinimumSize(QSize(0, 30))

        self.horizontalLayout.addWidget(self.btn_apply_config)

        self.btn_commit = QToolButton(self.gb_config_controls)
        self.btn_commit.setObjectName(u"btn_commit")
        self.btn_commit.setMinimumSize(QSize(0, 30))

        self.horizontalLayout.addWidget(self.btn_commit)


        self.verticalLayout.addWidget(self.gb_config_controls)

        self.table_config_entries = QTableView(self.centralwidget)
        self.table_config_entries.setObjectName(u"table_config_entries")
        self.table_config_entries.setAlternatingRowColors(True)
        self.table_config_entries.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_config_entries.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.table_config_entries.setSortingEnabled(True)

        self.verticalLayout.addWidget(self.table_config_entries)

        ConfigWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(ConfigWindow)
        self.statusbar.setObjectName(u"statusbar")
        ConfigWindow.setStatusBar(self.statusbar)

        self.retranslateUi(ConfigWindow)

        QMetaObject.connectSlotsByName(ConfigWindow)
    # setupUi

    def retranslateUi(self, ConfigWindow):
        ConfigWindow.setWindowTitle(QCoreApplication.translate("ConfigWindow", u"Config Manager", None))
        self.gb_config_controls.setTitle("")
        self.txt_config_search.setPlaceholderText(QCoreApplication.translate("ConfigWindow", u"Search config", None))
        self.lbl_config_dirty.setText(QCoreApplication.translate("ConfigWindow", u"No pending edits", None))
        self.btn_refresh_config.setText(QCoreApplication.translate("ConfigWindow", u"Refresh", None))
        self.btn_load_config_from_file.setText(QCoreApplication.translate("ConfigWindow", u"Import", None))
        self.btn_save_config_to_file.setText(QCoreApplication.translate("ConfigWindow", u"Export", None))
        self.btn_apply_config.setText(QCoreApplication.translate("ConfigWindow", u"Apply Changes", None))
        self.btn_commit.setText(QCoreApplication.translate("ConfigWindow", u"Save To NVM", None))
    # retranslateUi

