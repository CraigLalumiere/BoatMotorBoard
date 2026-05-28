# PC Communication Tool

This tool provides the host-side interface for firmware CLI and protobuf packets.

It supports both `motor` and `gauge` boards over USB CDC and can:

1. Send/receive CLI text
2. Query config database metadata
3. Read config entries
4. Set config entries
5. Request config save-to-NVM

## Installation (Windows - Command Prompt)

1. Install Python 3.12+.
2. Open a Command Prompt (not PowerShell).
3. From repo root:

```commandline
python -m venv .venv
.venv\Scripts\activate.bat
pip install -r pc_com\requirements.txt
```

## Installation (Windows - PowerShell)

1. Install Python 3.12+.
2. Open PowerShell.
3. From repo root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r .\pc_com\requirements.txt
```

## Running (Command Prompt)

From repo root:

```commandline
pc_com.bat
```

## Running (PowerShell)

From repo root:

```powershell
.\.venv\Scripts\Activate.ps1
python .\pc_com\pc_com.py
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
2. Launch `pc_com.bat` (Command Prompt) or `python .\pc_com\pc_com.py` (PowerShell).
3. Use CLI tab/console for direct commands.
4. Use config manager controls to sync settings with firmware.

## Updating Python Dependencies

If packages change:

```commandline
pip freeze > pc_com\requirements.txt
```

## GUI Editing

The GUI uses Qt/PySide.

1. Edit `main_window.ui` with Qt Designer.
2. Rebuild generated UI Python files:

```commandline
GUIRebuild.bat
```



# Contacts

| Person        | Role               | Email                  |
|---------------|--------------------|------------------------|
| Craig Lalumiere | Project Lead       | clalumiere@inertiapd.com |
