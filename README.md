## Setup

1. Clone this repo to somewhere on your disk, e.g. C:\Data\Jobs\FirmwareReferenceDesign\
2. Compile with ctrl+b (or click the little hammer icon in the top tool bar)
3. Connect the ST-link debugger into the SWD header on the PCB
4. Click on the debug icon in the top toolbar to flash 
5. The MCU will be halted at the first breakpoint. Press F8 to continue


---

## Flashing with Ozone

The STM32CubeIDE project is setup already to generate .elf and .hex files in the 'Debug' directory. These can be used to flash the board using Ozone. 

1. Download the Ozone debugger if you haven't already
2. Launch ozone
2. In the popup window, click **Open Existing Project**
3. Navigate to **stm32g4_ozone.jdebug** in this repo
4. Press F5 to flash the board, or click the power icon in the top tool bar
