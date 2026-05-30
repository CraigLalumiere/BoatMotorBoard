import logging

from PySide6 import QtCore
from PySide6.QtWidgets import QFileDialog, QDialog

from .includes.dfu import (
    dfuse_exit,
    dfuse_upload_block,
    download,
    wait_for_dfu_device,
    wait_for_no_dfu_device,
)
from .bootloader_window import Ui_bootloader_window


log = logging.getLogger(__name__)


class BootloaderWorker(QtCore.QObject):
    log_message = QtCore.Signal(str)
    progress_changed = QtCore.Signal(int, int)
    finished = QtCore.Signal(bool, str)

    def __init__(self, operation, bin_path=None):
        super().__init__()
        self.operation = operation
        self.bin_path = bin_path

    @QtCore.Slot()
    def run(self):
        try:
            start_addr = 0x08000000

            if self.operation == "wait_dfu":
                self.log_message.emit("Waiting for STM32 DFU device...")
                wait_for_dfu_device(vid=0x0483, timeout=8.0)
                self.finished.emit(True, "STM32 DFU device detected.")
                return

            if self.operation == "flash":
                self.log_message.emit(f"Flashing {self.bin_path}...")
                download(
                    filename=self.bin_path,
                    address=start_addr,
                    vid=0x0483,
                    progress_callback=self.progress_changed.emit,
                )
                self.finished.emit(True, "Flash complete.")
                return

            if self.operation == "verify":
                with open(self.bin_path, "rb") as f:
                    bin_data = f.read()

                total_size = len(bin_data)
                self.log_message.emit(f"Reading back {total_size} bytes from flash...")
                readback = dfuse_upload_block(address=start_addr, data_size=total_size, vid=0x0483)
                self.log_message.emit("Comparing binary to flash contents...")

                mismatch_count = 0
                for i, (expected, actual) in enumerate(zip(bin_data, readback)):
                    if expected != actual:
                        if mismatch_count < 10:
                            self.log_message.emit(
                                f"Mismatch @ 0x{start_addr + i:08X}: "
                                f"file 0x{expected:02X}, device 0x{actual:02X}"
                            )
                        mismatch_count += 1

                if mismatch_count == 0:
                    self.finished.emit(True, "Verify passed. Flash contents match the binary.")
                else:
                    self.finished.emit(False, f"Verify failed. {mismatch_count} mismatched bytes found.")
                return

            if self.operation == "reboot":
                self.log_message.emit("Sending DFU exit command...")
                dfuse_exit(address=start_addr, vid=0x0483)
                self.log_message.emit("Waiting for DFU device to disconnect...")
                wait_for_no_dfu_device(vid=0x0483, timeout=8.0)
                self.finished.emit(True, "Board left DFU mode.")
                return

            self.finished.emit(False, f"Unknown bootloader operation: {self.operation}")

        except Exception as exc:
            log.exception("Bootloader operation failed")
            self.finished.emit(False, str(exc))


class BootloaderWindow(QDialog):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.ui = Ui_bootloader_window()
        self.ui.setupUi(self)
        self.mode = "serial" if self.main_window.controller.connected else "needs_serial"
        self.worker_thread = None
        self.worker = None
        self.worker_done_mode = None
        self.worker_fail_mode = None
        self._reconnect_on_close_requested = False
        self._bind_ui_widgets()
        self._setup_ui_signals()
        self._set_mode(self.mode)

    def _bind_ui_widgets(self):
        self.bin_file_path = self.ui.bin_file_path
        self.btn_browse_path = self.ui.btn_browse_path
        self.btn_primary = self.ui.btn_primary
        self.btn_verify = self.ui.btn_verify
        self.btn_reboot = self.ui.btn_reboot
        self.btn_close = self.ui.btn_close
        self.progress = self.ui.progress_boot
        self.boot_txt_log = self.ui.boot_txt_log
        self.step_serial = self.ui.lbl_step_serial
        self.step_dfu = self.ui.lbl_step_dfu
        self.step_file = self.ui.lbl_step_file
        self.step_flash = self.ui.lbl_step_flash

    def _setup_ui_signals(self):
        self.btn_browse_path.clicked.connect(self.on_btn_browse_bin_clicked)
        self.btn_primary.clicked.connect(self.on_primary_clicked)
        self.btn_verify.clicked.connect(self.on_btn_verify_clicked)
        self.btn_reboot.clicked.connect(self.on_btn_reboot_clicked)
        self.btn_close.clicked.connect(self.accept)
        self.bin_file_path.textChanged.connect(lambda _: self._set_mode(self.mode))

    def _set_step_state(self, label, state):
        label.setProperty("stepState", state)
        label.style().unpolish(label)
        label.style().polish(label)

    def _set_mode(self, mode):
        self.mode = mode
        has_file = bool(self.bin_file_path.text().strip())

        if mode == "needs_serial":
            self.btn_primary.setText("Connect Serial First")
            self.btn_primary.setEnabled(False)
            self.btn_verify.setEnabled(False)
            self.btn_reboot.setEnabled(False)
            self._set_step_state(self.step_serial, "pending")
            self._set_step_state(self.step_dfu, "pending")
        elif mode == "serial":
            self.btn_primary.setText("Enter DFU")
            self.btn_primary.setEnabled(True)
            self.btn_verify.setEnabled(False)
            self.btn_reboot.setEnabled(False)
            self._set_step_state(self.step_serial, "done")
            self._set_step_state(self.step_dfu, "pending")
        elif mode == "dfu":
            self.btn_primary.setText("Flash Firmware")
            self.btn_primary.setEnabled(has_file)
            self.btn_verify.setEnabled(has_file)
            self.btn_reboot.setEnabled(True)
            self._set_step_state(self.step_serial, "done")
            self._set_step_state(self.step_dfu, "done")
        elif mode == "busy":
            self.btn_primary.setEnabled(False)
            self.btn_verify.setEnabled(False)
            self.btn_reboot.setEnabled(False)

        self.btn_close.setEnabled(mode != "busy")
        self._set_step_state(self.step_file, "done" if has_file else "pending")

    def _append_log(self, message):
        self.boot_txt_log.append(message)

    def _start_worker(self, operation, done_mode, bin_path=None, fail_mode="dfu"):
        self.worker_done_mode = done_mode
        self.worker_fail_mode = fail_mode
        self._set_mode("busy")
        self.progress.setRange(0, 0)
        self.progress.setTextVisible(False)

        self.worker_thread = QtCore.QThread(self)
        self.worker = BootloaderWorker(operation, bin_path)
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.worker.run)
        self.worker.log_message.connect(self._append_log)
        self.worker.progress_changed.connect(self._worker_progress_changed)
        self.worker.finished.connect(self._worker_finished)
        self.worker.finished.connect(self.worker_thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.worker_thread.finished.connect(self.worker_thread.deleteLater)
        self.worker_thread.finished.connect(self._worker_thread_finished)
        self.worker_thread.start()

    @QtCore.Slot(int, int)
    def _worker_progress_changed(self, current, total):
        self.progress.setTextVisible(True)
        self.progress.setRange(0, max(total, 1))
        self.progress.setValue(current)
        self.progress.setFormat(f"{int((current / max(total, 1)) * 100)}%")

    @QtCore.Slot(bool, str)
    def _worker_finished(self, ok, message):
        self.progress.setRange(0, 1)
        self.progress.setValue(1 if ok else 0)
        self.progress.setTextVisible(ok)
        if ok:
            self.progress.setFormat("100%")
        self._append_log(("[OK] " if ok else "[ERROR] ") + message)
        next_mode = self.worker_done_mode if ok else self.worker_fail_mode
        self._set_mode(next_mode)

    @QtCore.Slot()
    def _worker_thread_finished(self):
        self.worker_thread = None
        self.worker = None
        self.worker_done_mode = None
        self.worker_fail_mode = None

    def on_btn_browse_bin_clicked(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Binary File",
            self.main_window.data_folder,
            "Binary Files (*.bin);;All Files (*)",
        )
        if file_path:
            self.bin_file_path.setText(file_path)

    def on_primary_clicked(self):
        if self.mode == "serial":
            self.on_btn_boot_mode_clicked()
        elif self.mode == "dfu":
            self.on_btn_flash_clicked()

    def on_btn_boot_mode_clicked(self):
        if not self.main_window.controller.connected:
            self._append_log("[ERROR] Connect to the serial port first.")
            self._set_mode("needs_serial")
            return

        log.info(
            "Bootloader connect clicked; sending bootloader CLI command on %s",
            self.main_window.ui.txt_port.text(),
        )
        self._append_log("Sending bootloader command...")
        self.main_window.controller.transmit_cli_data(b"bootloader\r")
        QtCore.QTimer.singleShot(350, self._finish_bootloader_handoff)

    def _finish_bootloader_handoff(self):
        self._append_log("Releasing serial connection. Waiting for STM32 DFU...")
        self.main_window.controller.disconnect()
        self.main_window.update_interface_state()
        self._start_worker("wait_dfu", "dfu", fail_mode="serial")

    def on_btn_flash_clicked(self):
        bin_path = self.bin_file_path.text().strip()
        if not bin_path:
            self._append_log("[ERROR] No binary file selected.")
            self._set_mode("dfu")
            return
        self._set_step_state(self.step_flash, "active")
        self._start_worker("flash", "dfu", bin_path)

    def on_btn_verify_clicked(self):
        bin_path = self.bin_file_path.text().strip()
        if not bin_path:
            self._append_log("[ERROR] No binary file selected.")
            return
        self._set_step_state(self.step_flash, "active")
        self._start_worker("verify", "dfu", bin_path)

    def on_btn_reboot_clicked(self):
        self._start_worker("reboot", "needs_serial", fail_mode="dfu")

    def closeEvent(self, event):
        if self.worker_thread is not None and self.worker_thread.isRunning():
            self._append_log("Wait for the current bootloader operation to finish before closing.")
            event.ignore()
            return

        event.accept()

    def done(self, result):
        should_reconnect = self.worker_thread is None or not self.worker_thread.isRunning()
        super().done(result)
        if should_reconnect:
            self._request_reconnect_on_close()

    def _request_reconnect_on_close(self):
        if self._reconnect_on_close_requested:
            return

        self._reconnect_on_close_requested = True
        QtCore.QTimer.singleShot(
            1000,
            self.main_window.start_auto_reconnect_to_selected_port,
        )
