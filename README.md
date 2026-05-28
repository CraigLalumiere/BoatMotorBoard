# Boat Motor + Gauge Firmware

This repository contains two STM32G4 firmware targets plus shared infrastructure:

1. `motor/` - motor-side controller firmware
2. `gauge/` - gauge-cluster firmware
3. `shared/` - shared services, BSP abstractions, and common headers
4. `messages/` - protobuf/nanopb schema generation
5. `pc_com/` - Python desktop tool for CLI + protobuf config access

## Core Architecture

Both firmware projects use:

1. QP/QK active-object framework
2. TinyUSB CDC for PC communications
3. Nanopb for compact protobuf messaging
4. Shared PC packet framing (`HDLC` + CRC)

Both projects now also support the config-manager protobuf path:

1. `CONFIG_DB_INFO`
2. `CONFIG_DB_GET_ENTRY`
3. `CONFIG_DB_SET_ENTRY`
4. `CONFIG_DB_SAVE_TO_NVM`

## Repository Setup

1. Clone this repo.
2. Open in VS Code (devcontainer recommended).
3. Select a build profile (`Debug`).
4. Ensure your ARM toolchain is available (`arm-none-eabi`).

## Build Firmware (VS Code)

Primary workflow is the VS Code build/debug buttons:

1. Open the `motor` or `gauge` workspace.
2. Select `Debug` variant in the status bar.
3. Click the Build button (or press `Ctrl+Shift+B`).
4. Click Debug/Run to flash with your configured debugger.


## Generate Protobuf/Nanopb Messages

Run this whenever `.proto` files change:

```bash
cd messages
./build.sh
```

Generated outputs are copied to:

1. `messages/generated/c`
2. `pc_com/messages`

## Flashing with Ozone

Build first using the VS Code Build button so the latest firmware image exists.

1. Connect your J-Link/ST-Link to the board SWD header.
2. Launch Ozone.
3. Click **Open Existing Project**.
4. For motor firmware, open `motor/BoatMotorSTM32.jdebug`.
5. For gauge firmware, open `gauge/BoatGaugeSTM32.jdebug`.
6. Press `F5` (or click the power icon) to download/flash and start debug.
