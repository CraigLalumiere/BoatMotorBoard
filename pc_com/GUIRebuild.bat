@echo off
set SCRIPT_DIR=%~dp0
set REPO_ROOT=%SCRIPT_DIR%..
call "%REPO_ROOT%\.venv\Scripts\activate.bat"
pyside6-uic --from-imports "%SCRIPT_DIR%main_window.ui" -o "%SCRIPT_DIR%main_window.py"
pyside6-uic --from-imports "%SCRIPT_DIR%bootloader_window.ui" -o "%SCRIPT_DIR%bootloader_window.py"
pyside6-uic --from-imports "%SCRIPT_DIR%config_window.ui" -o "%SCRIPT_DIR%config_window.py"
