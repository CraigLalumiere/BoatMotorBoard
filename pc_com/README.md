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

## Linux: Setup and Launch

1. Install Python 3.12+ and the Python virtual environment package for your distribution.
2. Install Qt/PySide runtime libraries. On Ubuntu/Debian:

```bash
sudo apt update
sudo apt install python3-venv libgl1 libegl1 libfontconfig1 libfreetype6 libdbus-1-3 libxkbcommon-x11-0 libxcb-cursor0 libxcb-icccm4 libxcb-keysyms1 libxcb-shape0
```

3. From repo root, create/activate the virtual environment, install dependencies, and launch:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e ".[dev]"
python -m pc_com
```

## Serial Ports in Linux Containers

The port selector can only show serial devices that exist inside the Linux environment running the app. If Windows sees the STM32 as a COM port but Linux shows no `/dev/ttyACM*` or `/dev/ttyUSB*` device, the board has not been forwarded into Linux or into the container.

Check what Python can see:

```bash
python -m serial.tools.list_ports -v
```

On Windows with WSL 2, attach the USB device to WSL from an Administrator PowerShell:

```powershell
usbipd list
usbipd bind --busid <busid>
usbipd attach --wsl --busid <busid>
```

Then verify from Linux:

```bash
lsusb
ls -l /dev/ttyACM* /dev/ttyUSB*
```

If running inside Docker, the container must also be started with the serial device passed through, for example:

```bash
docker run --device=/dev/ttyACM0 ...
```

For VS Code dev containers, add the equivalent device passthrough to `devcontainer.json`, for example:

```json
"runArgs": ["--device=/dev/ttyACM0"]
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

## Updating Python Dependencies

If packages change:

```toml
# Edit dependency lists in pyproject.toml, then reinstall:
python -m pip install -e ".[dev]"
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
