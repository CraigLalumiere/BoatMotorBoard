import sys
import os
from PySide6 import QtWidgets
from PySide6.QtWidgets import QFileDialog, QMessageBox
from . import config_manager
from .config_manager import ConfigManager
from PySide6 import QtCore, QtWidgets
from PySide6.QtCore import Qt
from .messages.CLIData_pb2 import CLIData
from .messages.LogPrint_pb2 import LogPrint
from .messages.MotorData_pb2 import MotorData

from .main_window import Ui_MainWindow
from .bootloader_window import Ui_bootloader_window
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
from .includes.dfu import download, dfuse_exit, dfuse_upload, dfuse_upload_block, wait_for_dfu_device, wait_for_no_dfu_device


class PortSelectDialog(QtWidgets.QDialog):
    def __init__(self, com_ports, current_port, parent=None):
        super().__init__(parent)
        self.selected_port = None
        self.setWindowTitle("Select a port")
        self.setMinimumWidth(420)

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

        for port in com_ports:
            item = QtWidgets.QListWidgetItem()
            item.setData(Qt.UserRole, port.device)
            item.setSizeHint(QtCore.QSize(0, 54))
            self.port_list.addItem(item)
            self.port_list.setItemWidget(item, self._build_port_row(port))

            if port.device == current_port:
                item.setSelected(True)
                self.port_list.setCurrentItem(item)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.addWidget(self.port_list)

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
        self._setup_ui_signals() 

        # set app title
        self.setWindowTitle("Config Manager")        

        self.config_manager = config_manager
        self.config_manager.register_config_view_frame(self.ui.frame_config_viewer)

    def _setup_ui_signals(self):
        self.ui.btn_load_config_from_file.clicked.connect(self.on_btn_load_config_from_file_clicked)      
        self.ui.btn_save_config_to_file.clicked.connect(self.on_btn_save_config_to_file_clicked)      
        self.ui.btn_commit.clicked.connect(self.on_btn_commit_clicked)      

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

    def on_btn_commit_clicked(self):
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

    def on_btn_browse_bin_clicked(self):
        file_path, _ = QFileDialog.getOpenFileName(None,
                                                  'Select Binary File',
                                                  self.data_folder)
        if file_path != "":
            #self.data_folder = os.path.normpath(folder)
            self.ui.bin_file_path.setText(file_path)

    def on_btn_port_clicked(self):
        com_ports = get_com_port_options()
        if not com_ports:
            QMessageBox.information(self, "Select a port", "No serial ports were found.")
            self.update_interface_state()
            return

        dialog = PortSelectDialog(com_ports, self.ui.txt_port.text(), self)
        if dialog.exec() == QtWidgets.QDialog.Accepted and dialog.selected_port:
            self.ui.txt_port.setText(dialog.selected_port)

        self.update_interface_state()

    def on_btn_connect_clicked(self):
        self.controller.connect(self.ui.txt_port.text())
        self.ui.txt_log.setText("")
        self.ui.terminal.reset()
        self.update_interface_state() 

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
        '''self.controller.transmit_cli_data(b"bootloader\n")
        self.ui.txt_log.append("Sent 'bootloader' command, entering bootloader, resetting device...")
        time.sleep(2) #Wait for USB re-enumeration

        self.ui.txt_log.append("Finding bin path...")

        bin_path = self.ui.bin_file_path.text()

        self.ui.txt_log.append("Bin path found, running script")

        if bin_path:
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

            # Flash
            start_addr = 0x08000000
            download(filename=bin_path, address=start_addr)

            self.ui.txt_log.append("Flash complete. Please reconnect the device to continue.")
            time.sleep(2) #Wait for USB re-enumeration

            self.ui.txt_log.append(self.ui.txt_port.text())
            self.connected = True
            self.controller.connect(self.ui.txt_port.text())
            self.update_interface_state() '''
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

from PySide6.QtWidgets import QDialog, QVBoxLayout, QLabel, QPushButton

class BootloaderWindow(QDialog):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.ui = Ui_bootloader_window()
        self.ui.setupUi(self)

        self.controller = ComController()
        # self.controller = ComControllerFake()

        # set app title
        self.setWindowTitle("Bootloader Tool")

        self._setup_ui_signals()

    def _setup_ui_signals(self):
        self.ui.btn_browse_path.clicked.connect(self.on_btn_browse_bin_clicked)
        self.ui.btn_verify.clicked.connect(self.on_btn_verify_clicked)
        self.ui.btn_boot_mode.clicked.connect(self.on_btn_boot_mode_clicked)
        self.ui.btn_flash.clicked.connect(self.on_btn_flash_clicked)
        self.ui.btn_bl_disconnect.clicked.connect(self.on_btn_bl_disconnect_clicked)

    def on_btn_bl_disconnect_clicked(self):
        self.ui.boot_txt_log.append("Exiting DFU and jumping to application...")
        try:
            start_addr = 0x08000000
            dfuse_exit(address=start_addr, vid=0x0483)
            self.ui.boot_txt_log.append("DFU exit command sent. Waiting for device to leave DFU...")
            QtWidgets.QApplication.processEvents()
            wait_for_no_dfu_device(vid=0x0483, timeout=8.0)
            self.ui.boot_txt_log.append("DFU device disconnected. Reconnect the serial port after the board restarts.")
        except Exception as exc:
            self.ui.boot_txt_log.append(f"[ERROR] {exc}")
            self.ui.boot_txt_log.append("The board still appears to be in DFU mode.")

    def on_btn_browse_bin_clicked(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            'Select Binary File',
            self.main_window.data_folder,
            'Binary Files (*.bin);;All Files (*)',
        )
        if file_path != "":
            #self.data_folder = os.path.normpath(folder)
            self.ui.bin_file_path.setText(file_path)

    def on_btn_boot_mode_clicked(self):
        
        if self.main_window.controller.connected:
            self.main_window.controller.transmit_cli_data(b"bootloader\n")  
            self.ui.boot_txt_log.append("Sent 'bootloader' command, entering bootloader, resetting device...")
            QtWidgets.QApplication.processEvents()
            time.sleep(0.25)
            self.main_window.controller.disconnect()
            self.main_window.update_interface_state()
            self.ui.boot_txt_log.append("Serial connection released. Waiting for STM32 DFU...")
            QtWidgets.QApplication.processEvents()

            try:
                wait_for_dfu_device(vid=0x0483, timeout=8.0)
                self.ui.boot_txt_log.append("STM32 DFU device detected. Choose a .bin file, then click Flash.")
            except Exception as exc:
                self.ui.boot_txt_log.append(f"[ERROR] {exc}")
                self.ui.boot_txt_log.append("If the board LED stopped, it may be in bootloader mode but not visible to PyUSB.")
        else:
            self.ui.boot_txt_log.append("Please connect to device serial port first")

    def on_btn_flash_clicked(self):
        self.ui.boot_txt_log.append("Finding bin path...")

        bin_path = self.ui.bin_file_path.text()

        if bin_path:
            self.ui.boot_txt_log.append("Bin path found, running script")

            try:
                start_addr = 0x08000000
                download(filename=bin_path, address=start_addr, vid=0x0483)
                self.ui.boot_txt_log.append("Flash complete. Click Disconnect to jump back to the application.")
            except Exception as exc:
                self.ui.boot_txt_log.append(f"[ERROR] Flash failed: {exc}")
        else:
            self.ui.boot_txt_log.append("[ERROR] No bin path found")

           
    def on_btn_verify_clicked(self):
        self.ui.boot_txt_log.append("Verifying flash...")
        bin_path = self.ui.bin_file_path.text()

        if not bin_path:
            self.ui.boot_txt_log.append("[ERROR] No binary file selected.")
            return

        # Load .bin file
        with open(bin_path, "rb") as f:
            bin_data = f.read()

        total_size = len(bin_data)
        self.ui.boot_txt_log.append(f"[INFO] Loaded {total_size} bytes from {bin_path}")

        start_addr = 0x08000000

        try:
            readback = dfuse_upload_block(address=start_addr, data_size=total_size, vid=0x0483)
        except Exception as exc:
            self.ui.boot_txt_log.append(f"[ERROR] Verify failed: {exc}")
            return

        # Compare memory
        self.ui.boot_txt_log.append("[INFO] Comparing file to flash contents...")
        mismatch_count = 0
        for i, (expected, actual) in enumerate(zip(bin_data, readback)):
            if expected != actual:
                if mismatch_count < 10:
                    self.ui.boot_txt_log.append(
                        f"[MISMATCH @ 0x{start_addr + i:08X}] File: 0x{expected:02X}, Device: 0x{actual:02X}"
                    )
                mismatch_count += 1

        if mismatch_count == 0:
            self.ui.boot_txt_log.append("[SUCCESS] Flash contents match the binary.")
        else:
            self.ui.boot_txt_log.append(f"[FAIL] {mismatch_count} mismatched bytes found.")

        print(f"[VERIFY] Done. {mismatch_count} mismatches.")



def main():
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
