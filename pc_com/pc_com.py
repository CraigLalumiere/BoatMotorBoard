import sys
import os
import logging
from PySide6 import QtWidgets
from PySide6.QtWidgets import QFileDialog, QMessageBox
from . import config_manager
from .config_manager import ConfigManager, ConfigTableModel, ConfigValueDelegate
from PySide6 import QtCore, QtWidgets
from PySide6.QtCore import Qt
from .messages.CLIData_pb2 import CLIData
from .messages.LogPrint_pb2 import LogPrint
from .messages.MotorData_pb2 import MotorData

from .bootloader_tool import BootloaderWindow
from .main_window import Ui_MainWindow
from .messages.ConfigDB_pb2 import ConfigEntryDataResp, ConfigDBInfoResp
from .config_window import Ui_ConfigWindow
from .com_controller import ComController, get_com_port_options
from .motor_dashboard import MotorDashboard
import signal
# from com_controller_fake import ComControllerFake

import time
import usb.backend.libusb1
import libusb_package
import usb.core

log = logging.getLogger(__name__)


class PortSelectDialog(QtWidgets.QDialog):
    def __init__(self, com_ports, current_port, parent=None):
        super().__init__(parent)
        self.selected_port = None
        self.current_port = current_port
        self.port_devices = []
        self.setWindowTitle("Select a port")
        self.setMinimumWidth(420)

        self.status_label = QtWidgets.QLabel()
        self.status_label.setStyleSheet("color: #64748b; padding: 2px;")

        self.port_list = QtWidgets.QListWidget()
        self.port_list.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.port_list.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.port_list.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.port_list.itemClicked.connect(self.on_port_clicked)
        self.port_list.setStyleSheet("""
            QListWidget {
                background: #f6f8fa;
                border: 1px solid #c9d1d9;
                border-radius: 6px;
                outline: 0;
            }
            QListWidget::item {
                padding: 8px;
                border-bottom: 1px solid #d8dee4;
            }
            QListWidget::item:selected {
                background: #dbeafe;
                color: #0f172a;
            }
            QListWidget::item:hover {
                background: #eef2ff;
            }
        """)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)
        layout.addWidget(self.status_label)
        layout.addWidget(self.port_list)

        self.refresh_timer = QtCore.QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_ports)
        self.refresh_timer.start(1000)

        self._populate_ports(com_ports)

    def refresh_ports(self):
        self._populate_ports(get_com_port_options())

    def _populate_ports(self, com_ports):
        devices = [port.device for port in com_ports]
        if devices == self.port_devices:
            return

        selected_device = self.selected_port
        current_item = self.port_list.currentItem()
        if current_item is not None:
            selected_device = current_item.data(Qt.UserRole)

        self.port_devices = devices
        self.port_list.clear()

        if not com_ports:
            self.status_label.setText("No serial ports found. Waiting for devices...")
            return

        self.status_label.setText("Select a port. This list refreshes automatically.")

        fallback_item = None
        for port in com_ports:
            item = QtWidgets.QListWidgetItem()
            item.setData(Qt.UserRole, port.device)
            item.setSizeHint(QtCore.QSize(0, 54))
            self.port_list.addItem(item)
            self.port_list.setItemWidget(item, self._build_port_row(port))

            if fallback_item is None:
                fallback_item = item

            if port.device == selected_device or port.device == self.current_port:
                item.setSelected(True)
                self.port_list.setCurrentItem(item)
                selected_device = port.device

        if self.port_list.currentItem() is None and fallback_item is not None:
            self.port_list.setCurrentItem(fallback_item)

    def _build_port_row(self, port):
        description = port.label
        prefix = f"{port.device}: "
        if description.startswith(prefix):
            description = description[len(prefix):]

        row = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(row)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(2)

        device_label = QtWidgets.QLabel(port.device)
        device_label.setStyleSheet("font-weight: 600; color: #111827;")

        description_label = QtWidgets.QLabel(description)
        description_label.setStyleSheet("color: #4b5563;")
        description_label.setTextInteractionFlags(Qt.NoTextInteraction)
        description_label.setWordWrap(True)

        layout.addWidget(device_label)
        layout.addWidget(description_label)
        return row

    def on_port_clicked(self, item):
        self.selected_port = item.data(Qt.UserRole)
        self.accept()


class ConfigWindow(QtWidgets.QMainWindow):
    def __init__(self, config_manager: ConfigManager):
        super(ConfigWindow, self).__init__()

        self.ui = Ui_ConfigWindow()
        self.ui.setupUi(self)

        self.config_manager = config_manager
        self.setWindowTitle("Config Manager")
        self.setMinimumSize(980, 620)

        self.model = ConfigTableModel(self.config_manager, self)
        self.proxy_model = QtCore.QSortFilterProxyModel(self)
        self.proxy_model.setSourceModel(self.model)
        self.proxy_model.setFilterCaseSensitivity(Qt.CaseInsensitive)
        self.proxy_model.setFilterKeyColumn(-1)

        self._configure_table()
        self._setup_ui_signals()
        self._update_dirty_count(self.config_manager.dirty_count())
        self._update_state("Disconnected")

    def _configure_table(self):
        self.table = self.ui.table_config_entries
        self.table.setModel(self.proxy_model)
        self.table.setItemDelegate(ConfigValueDelegate(self.table))
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setDefaultSectionSize(34)
        self.table.horizontalHeader().setStretchLastSection(False)
        self.table.horizontalHeader().setSectionResizeMode(
            ConfigTableModel.COL_NAME, QtWidgets.QHeaderView.Stretch
        )
        self.table.horizontalHeader().setSectionResizeMode(
            ConfigTableModel.COL_VALUE, QtWidgets.QHeaderView.ResizeToContents
        )
        self.table.horizontalHeader().setSectionResizeMode(
            ConfigTableModel.COL_DEFAULT, QtWidgets.QHeaderView.ResizeToContents
        )
        self.table.horizontalHeader().setSectionResizeMode(
            ConfigTableModel.COL_TYPE, QtWidgets.QHeaderView.ResizeToContents
        )
        self.table.horizontalHeader().setSectionResizeMode(
            ConfigTableModel.COL_STATE, QtWidgets.QHeaderView.ResizeToContents
        )
        self.table.horizontalHeader().setSectionResizeMode(
            ConfigTableModel.COL_REVERT, QtWidgets.QHeaderView.Fixed
        )
        self.table.horizontalHeader().setSectionResizeMode(
            ConfigTableModel.COL_RESTORE_DEFAULT, QtWidgets.QHeaderView.Fixed
        )
        self.table.setColumnWidth(ConfigTableModel.COL_REVERT, 38)
        self.table.setColumnWidth(ConfigTableModel.COL_RESTORE_DEFAULT, 38)

    def _setup_ui_signals(self):
        self.ui.txt_config_search.textChanged.connect(
            lambda text: self.proxy_model.setFilterRegularExpression(
                QtCore.QRegularExpression(QtCore.QRegularExpression.escape(text))
            )
        )
        self.ui.btn_refresh_config.clicked.connect(self.on_btn_refresh_clicked)
        self.ui.btn_load_config_from_file.clicked.connect(self.on_btn_load_config_from_file_clicked)
        self.ui.btn_save_config_to_file.clicked.connect(self.on_btn_save_config_to_file_clicked)
        self.ui.btn_apply_config.clicked.connect(self.on_btn_apply_clicked)
        self.ui.btn_commit.clicked.connect(self.on_btn_commit_clicked)
        self.config_manager.dirty_count_changed.connect(self._update_dirty_count)
        self.config_manager.load_progress_changed.connect(self._update_progress)
        self.config_manager.state_changed.connect(self._update_state)

    def _update_dirty_count(self, count):
        if count:
            self.ui.lbl_config_dirty.setText(f"{count} edited")
        else:
            self.ui.lbl_config_dirty.setText("No pending edits")
        self.ui.btn_apply_config.setEnabled(count > 0)

    def _update_progress(self, current, total):
        self.ui.progress_config_load.setVisible(total > 0 and current < total)
        self.ui.progress_config_load.setRange(0, max(total, 1))
        self.ui.progress_config_load.setValue(current)

    def _update_state(self, state):
        self.statusBar().showMessage(state)

    def on_btn_refresh_clicked(self):
        if self.config_manager.check_for_uncommitted_ui_changes():
            answer = QMessageBox.question(
                self,
                "Discard edits?",
                "Refreshing will discard edits that have not been applied.",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        self.config_manager.load()

    def on_btn_load_config_from_file_clicked(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            caption="Select Config File",
            dir="",
            filter="JSON Files (*.json);;All Files (*)"
        )

        if not file_path:  # User canceled
            return
        
        try:
            self.config_manager.load_config_entries_from_json(file_path)
        except config_manager.FileVersionMismatchError as e:
            msg = QMessageBox(QMessageBox.Warning, 
                          "Version Mismatch",
                          str(e),
                          QMessageBox.Close,
                          self)
            
            msg.setWindowFlag(Qt.WindowStaysOnTopHint)  # Keep it on top
            msg.show()
            msg.exec()
        except (KeyError, ValueError, TypeError) as e:
            QMessageBox.warning(self, "Import Failed", str(e))


    def on_btn_save_config_to_file_clicked(self):
        if self.config_manager.check_for_uncommitted_ui_changes() == True:
            msg = QMessageBox(QMessageBox.Warning, 
                          "Uncommitted Entries", 
                          "Highlighted entries have not been committed.\nPlease commit them before saving.",
                          QMessageBox.Close,
                          self)

            msg.setWindowFlag(Qt.WindowStaysOnTopHint)  # Keep it on top
            msg.show()
            msg.exec()

        else:
            file_path, _ = QFileDialog.getSaveFileName(
                self,
                caption="Save Config File",
                dir="",
                filter="JSON Files (*.json);;All Files (*)"
            )

            if not file_path:  # User canceled
                return
            
            self.config_manager.save_config_entries_to_json(file_path)

    def on_btn_apply_clicked(self):
        self.config_manager.apply_changed_entries()

    def on_btn_commit_clicked(self):
        if self.config_manager.check_for_uncommitted_ui_changes():
            QMessageBox.warning(
                self,
                "Apply Changes First",
                "Apply edited values to the device before saving to NVM.",
            )
            return
        self.config_manager.commit_database_to_flash()
           


class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(ApplicationWindow, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.controller = ComController()
        # self.controller = ComControllerFake()

        self.config_manager = ConfigManager(self.controller, self.ui.txt_log)  
        self.config_window = ConfigWindow(self.config_manager) 
        self.dashboard = MotorDashboard(self.ui.dashboard_host)
        self.ui.dashboard_host_layout.addWidget(self.dashboard)

        self.recording = False
        self.outfile = None
        self.outfile_errors = None
        self.last_motor_data_log_time = 0.0
        self._auto_reconnect_port = ""
        self._auto_reconnect_deadline = 0.0
        self._auto_reconnect_timer = QtCore.QTimer(self)
        self._auto_reconnect_timer.setInterval(750)
        self._auto_reconnect_timer.timeout.connect(self._attempt_auto_reconnect)

        self.data_folder = os.path.dirname(os.path.normpath(__file__))
        self.ui.txt_data_dir.setText(self.data_folder)

        # set app title
        self.setWindowTitle("InertiaStat Com Tool")

        self.update_interface_state()

        self.gui_refresh_interval = 50
        self.gui_refresh_timer = QtCore.QTimer()
        self.gui_refresh_timer.timeout.connect(self.on_gui_refresh_timer)
        self.gui_refresh_timer.start(self.gui_refresh_interval)

        self.console_test_timer = QtCore.QTimer()
        self.console_test_timer.timeout.connect(self.on_console_test_timer)
        # self.console_test_timer.start(3000)
        self.console_test_int = 64

        self._setup_ui_signals()

        ### Setup USB backend with patch ###
        
        # USB backend setup
        backend = usb.backend.libusb1.get_backend(
            find_library=libusb_package.find_library
        )

        # Patch usb.core.find
        _real_find = usb.core.find
        def _patched_find(*args, **kwargs):
            kwargs.setdefault("backend", backend)
            return _real_find(*args, **kwargs)
        usb.core.find = _patched_find

    def on_console_test_timer(self):
        self.ui.terminal.feed(self.console_test_int.to_bytes())
        self.console_test_int += 1

    def _setup_ui_signals(self):
        self.ui.btn_browse_dir.clicked.connect(self.on_btn_browse_clicked)
        #self.ui.btn_browse_path.clicked.connect(self.on_btn_browse_bin_clicked)
        self.ui.btn_port.clicked.connect(self.on_btn_port_clicked)
        self.ui.btn_connect.clicked.connect(self.on_btn_connect_clicked)
        self.ui.btn_disconnect.clicked.connect(self.on_btn_disconnect_clicked)
        self.ui.btn_start_record.clicked.connect(self.on_btn_start_record_clicked)
        self.ui.btn_stop_record.clicked.connect(self.on_btn_stop_record_clicked)
        self.ui.terminal.key_pressed_evt.connect(self.on_terminal_key_pressed)
        self.ui.btn_bootloader_clicked.clicked.connect(self.on_btn_bootloader_clicked)
        self.ui.btn_open_config.clicked.connect(self.on_btn_open_config_clicked)

    def on_gui_refresh_timer(self):
        evts = self.controller.update_and_get_events()
        for e in evts:
            self.handle_controller_event(e)

        msgs = self.controller.get_received_messages()
        for msg in msgs:
            # print(type(msg))
            self.handle_controller_msg_received(msg)

    def handle_controller_msg_received(self, message):
        if isinstance(message, CLIData):
            # print("Receive CLI data: {0}".format(message.msg))
            self.ui.terminal.feed(message.msg)

        if isinstance(message, LogPrint):
            msg_string = "{0} {1}".format(message.milliseconds_tick, message.msg)
            self.ui.txt_log.append(msg_string)

            if self.recording:
                self.outfile.write(msg_string + '\n')

        if isinstance(message, MotorData):
            self.dashboard.update_motor_data(message)
            msg_string = (
                "{0} RPM:{1:5.0f} VBat:{2:5.2f}V Temp:{3:4.1f}C "
                "Press:{4:4.1f} EngMin:{5} Neutral:{6} Start:{7} TG:{8} PG:{9} Bz:{10}"
            ).format(
                message.milliseconds_tick,
                message.tachometer,
                message.vbat,
                message.temperature,
                message.pressure,
                message.engine_minutes,
                int(message.neutral),
                int(message.start),
                int(message.temp_good),
                int(message.pres_good),
                int(message.buzzer),
            )

            now = time.monotonic()
            if (now - self.last_motor_data_log_time) >= 0.5:
                self.last_motor_data_log_time = now
                self.ui.txt_log.append(msg_string)

            if self.recording:
                self.outfile.write(msg_string + '\n')

        # for config manager
        if type(message) in [ConfigDBInfoResp, ConfigEntryDataResp]:
            self.config_manager.handle_msg_received(message)  



    def handle_controller_event(self, evt):
        if evt['event'] == 'error':
            if self.recording:
                self.recording = False

                if self.outfile is not None:
                    self.outfile.close()

                if self.outfile_errors is not None:
                    self.outfile_errors.close()

            self.update_interface_state()

        elif evt['event'] == 'connection_status_changed':
            pass

    def on_btn_browse_clicked(self):
        folder = QFileDialog.getExistingDirectory(None,
                                                  'Select Data Directory',
                                                  self.data_folder)
        if folder != "":
            self.data_folder = os.path.normpath(folder)
            self.ui.txt_data_dir.setText(self.data_folder)

    def on_btn_port_clicked(self):
        com_ports = get_com_port_options()
        if not com_ports:
            QMessageBox.information(self, "Select a port", "No serial ports were found.")
            self.update_interface_state()
            return

        dialog = PortSelectDialog(com_ports, self.ui.txt_port.text(), self)
        if dialog.exec() == QtWidgets.QDialog.Accepted and dialog.selected_port:
            self.ui.txt_port.setText(dialog.selected_port)
            self.connect_to_selected_port()
            return

        self.update_interface_state()

    def on_btn_connect_clicked(self):
        self.connect_to_selected_port()

    def connect_to_selected_port(self, clear_display=True):
        selected_port = self.ui.txt_port.text()
        if selected_port == "":
            self.update_interface_state()
            return

        if self.controller.connected:
            self.controller.disconnect()

        self.controller.connect(selected_port)
        if clear_display:
            self.ui.txt_log.setText("")
            self.ui.terminal.reset()
        self.update_interface_state() 

    def start_auto_reconnect_to_selected_port(self, timeout_ms=10000):
        selected_port = self.ui.txt_port.text().strip()
        if selected_port == "" or self.controller.connected:
            self.update_interface_state()
            return

        log.info("Starting automatic reconnect to %s", selected_port)
        self._auto_reconnect_port = selected_port
        self._auto_reconnect_deadline = time.monotonic() + (timeout_ms / 1000)
        self._attempt_auto_reconnect()
        if not self.controller.connected:
            self._auto_reconnect_timer.start()

    def _attempt_auto_reconnect(self):
        if self.controller.connected:
            self._auto_reconnect_timer.stop()
            self.update_interface_state()
            return

        if self._auto_reconnect_port == "" or time.monotonic() > self._auto_reconnect_deadline:
            self._auto_reconnect_timer.stop()
            self.update_interface_state()
            return

        available_ports = [port.device for port in get_com_port_options()]
        is_virtual_port = "://" in self._auto_reconnect_port
        if self._auto_reconnect_port not in available_ports and not is_virtual_port:
            self.ui.txt_status.setText("Reconnecting")
            self.ui.txt_status.setStyleSheet("QLineEdit{background: rgb(237, 125, 49);}")
            return

        log.info("Attempting automatic reconnect to %s", self._auto_reconnect_port)
        self.ui.txt_port.setText(self._auto_reconnect_port)
        self.connect_to_selected_port(clear_display=False)

        if self.controller.connected:
            self._auto_reconnect_timer.stop()

    def on_btn_disconnect_clicked(self):
        self.controller.disconnect()

        self.recording = False

        if self.outfile is not None:
            self.outfile.close()

        if self.outfile_errors is not None:
            self.outfile_errors.close()

        self.update_interface_state()

    def on_btn_start_record_clicked(self):
        self.outfile = open(os.path.join(self.ui.txt_data_dir.text(), "log.txt"), 'w')
        self.outfile_errors = open(os.path.join(self.ui.txt_data_dir.text(), "log_errors.txt"), 'w')
        self.recording = True
        self.update_interface_state()

    def on_btn_stop_record_clicked(self):
        self.recording = False

        if self.outfile is not None:
            self.outfile.close()

        if self.outfile_errors is not None:
            self.outfile_errors.close()

        self.update_interface_state()

    def on_terminal_key_pressed(self, data):
        if self.controller.connected:
            self.controller.transmit_cli_data(data)

    def on_btn_bootloader_clicked(self):
        popup = BootloaderWindow(self)
        popup.exec()

    def on_btn_open_config_clicked(self, data):
        if self.controller.connected:
            pass
        self.config_window.show()
        # self.config_manager.load_fake_entries()
        self.config_manager.load()

    def update_interface_state(self):
        if self.ui.txt_port.text() == '':
            self.ui.txt_status.setText("Disconnected")
            self.ui.txt_status.setStyleSheet("QLineEdit{background: rgb(237, 125, 49);}")
            self.ui.btn_connect.setEnabled(False)
            self.ui.btn_disconnect.setEnabled(False)
            self.ui.btn_start_record.setEnabled(False)
            self.ui.btn_stop_record.setEnabled(False)
            self.ui.btn_open_config.setEnabled(False)
        else:
            if self.controller.connected:
                if self.recording:
                    self.ui.txt_status.setText("Recording")
                    self.ui.txt_status.setStyleSheet("QLineEdit{background: rgb(237, 125, 49);}")
                    self.ui.btn_start_record.setEnabled(False)
                    self.ui.btn_stop_record.setEnabled(True)
                    self.ui.btn_open_config.setEnabled(True)
                else:
                    self.ui.txt_status.setText("Connected")
                    self.ui.txt_status.setStyleSheet("QLineEdit{background: rgb(146, 208, 80);}")
                    self.ui.btn_connect.setEnabled(False)
                    self.ui.btn_disconnect.setEnabled(True)
                    self.ui.btn_start_record.setEnabled(True)
                    self.ui.btn_stop_record.setEnabled(False)
                    self.ui.btn_open_config.setEnabled(True)
            else:
                self.ui.txt_status.setText("Disconnected")
                self.ui.txt_status.setStyleSheet("QLineEdit{background: rgb(237, 125, 49);}")
                self.ui.btn_connect.setEnabled(True) 
                self.ui.btn_disconnect.setEnabled(False)                
                self.ui.btn_start_record.setEnabled(False)
                self.ui.btn_stop_record.setEnabled(False)
                self.ui.btn_open_config.setEnabled(False)

    def closeEvent(self, event):
        """
        Application closed. Close gracefully.
        """
        self.controller.disconnect()

        if self.recording and self.outfile is not None:
            self.outfile.close()

        if self.recording and self.outfile_errors is not None:
            self.outfile_errors.close()

        event.accept()

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    app = QtWidgets.QApplication(sys.argv)
    application = ApplicationWindow()
    application.show()

    # Allow Ctrl+C (SIGINT) from terminal to quit the Qt event loop cleanly.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    app._sigint_timer = QtCore.QTimer()
    app._sigint_timer.timeout.connect(lambda: None)
    app._sigint_timer.start(200)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
