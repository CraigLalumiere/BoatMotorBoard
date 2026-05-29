# PC Communication Tool

This tool provides the host-side interface for firmware CLI and protobuf packets.

It supports both `motor` and `gauge` boards over USB CDC and can:

1. Send/receive CLI text
2. Query config database metadata
3. Read config entries
4. Set config entries
5. Request config save-to-NVM

## Windows (PowerShell): Setup and Launch

1. Install Python 3.12+.
2. Open PowerShell.
3. From repo root, create/activate the virtual environment, install dependencies, and launch:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".[dev]"
python -m pc_com
```

## Message Generation Dependency

The Python message modules are generated from `messages/*.proto`.

If protobuf definitions change, regenerate:

```bash
cd messages
./build.sh
```

This updates `pc_com/messages/*.py`.

## Typical Workflow

1. Connect board via USB.
2. Launch `python -m pc_com`.
3. Use CLI tab/console for direct commands.
4. Use config manager controls to sync settings with firmware.

## MotorData Simulator

To preview the dashboard without STM32 hardware, start the fake MotorData stream in a second terminal:

```powershell
python -m pc_com.motor_data_simulator
```

Then launch the app, click **Select Port**, choose `MotorData simulator: socket://127.0.0.1:7777`, and connect.

## GUI Editing

The GUI uses Qt/PySide.

Humans and LLM coding assistants should edit the `.ui` files, not the generated Python files created by `GUIRebuild.bat`. Generated UI Python files are overwritten when the GUI is rebuilt.

1. From the repo root with the virtual environment active, launch Qt Designer:

```powershell
pyside6-designer .\pc_com\main_window.ui
```

2. Edit and save `main_window.ui` in Qt Designer.
3. Rebuild the generated Python UI files:

```powershell
.\pc_com\GUIRebuild.bat
```



# Contacts

| Person        | Role               | Email                  |
|---------------|--------------------|------------------------|
| Craig Lalumiere | Project Lead       | clalumiere@inertiapd.com |
