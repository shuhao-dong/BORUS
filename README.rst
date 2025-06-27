1. Overview
1. Overview
********

This is the project code for TORUS using Zephyr RTOS to collect sensor data and 
transimit or log data based on the current state of the device. The code is primarily 
tested on Nordic Thingy53. Helper tools are included.

.. code-block:: none

    BORUS/
    ├── boards                    # Board definition files to be included in the sysbuild
    │   └── nordic                # Nordic board definition files
    │       └── torus53/          # Board definition files speficially for torus53 board
    ├── src                       # Sources files for wearable
    │   ├── driver/               # Driver files (battery, IMU, pressure)
    |   ├── feature/              # Feature files (butterworth filter, gait analysis)
    │   └── main.c                # Main source code firmware
    ├── sysbuild                  # Configuration used for sysbuild (ipc_radio and mcuboot)
    │   ├── ipc_radio             # Configuration for network core firmware: ipc_radio
    └── tools                     # Helper tools
        ├── hci_usb_ext_receiver  # HCI controller firmware for nRF52840 dongle attached to RPi
        └── littlefs-fuse         # Tool to convert .bin file to .csv file


2. Requirements
************

You must use nRF Connect SDK and toolchain version v3.0.2 or above in order to better 
support the partition manager and sysbuild. This project is tested with NCS 3.0.2. Note that is you are using a version below, you have to change app.overlay, 
speficially deleting the msc_disk0 node and the parent node of it. 

This firmware is tested with Thingy53 although the target of the build should be set to torus53

3. Board Definition files
**********************

The torus53 board definition files are located in the `boards/nordic/torus53` directory. Attached peripherals are defined in the `torus53_nrf5340_common.dtsi` file.

4. Building and Running
********************

Currently, there is an issue related to TFM in NCS v3.0.2 that when enabled, the code size becomes siginificantly huge that will cause overflow. Therefore, build with non-secure option 
should be avoided until fix. 

4.1 Flash via VS Code NRF Extension
-----------------------------------
The nRF Connect Extension in VSCode is recommended to build and test the code.

To get started with nRF Connect Extension in VSCode, please refer to the official `tutorial <https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-VS-Code/Tutorials>`_

When flash, please remember the Thingy53 is based on nRF5340 dual-core SoC, you need to flash 
both the application core and the network core to enable functions like BLE.

After the build process, in order to flash the firmware, you will need a JTAG debugger. You can 
either use a dedicated debugger, or use the one that comes with the nRF5340DK. See How to Programme
Thingy53 for more details. 

On successful build and flash, your Thingy53 should light the 3 colour LEDs in series. Note that 
current code disabled log output over UART, you will need a JTAG connector to achieve RTT log output. 

4.2 Flash via DFU From Application
----------------------------------
You could also flash the new firmware via USB to perform a DFU. You will need a signed bin file to perform
such an action. It is recommended to use dfu-util command line tool or AuTerm GUI software. 

When use dfu-util, first run::

    dfu-util -l

Remember the serial number from the USB device with PID and VID 0001:0001. Then run::

    dfu-util -s <serial number> -e

To enumerate the USB device as a USB DFU class in order to perform DFU. Once the project has been built, you will need two files to flash both Application core and Network core::

    dfu-util -s <serial number> -a 1 -D <~/build/BORUS/zephyr/zephyr.signed.bin>          # For the application core
    dfu-util -s <serial number> -a 1 -D <~/build/signed_by_mcuboot_and_b0_ipc_radio.bin>  # For the network core

For the two image swap mechanism, always download the NEW firmware to alt 1.

4.3 Flash via Jlink and nRF Programmer
--------------------------------------
Alternatively, one can flash the new firmware using the debug interface via Jlink. You will need a nRF5340/nRF54l15DK for this. Connect the Debug Out port to the port on the wearable. Then connect the IMCU USB to your host machine 
that has a nRF Connect for Desktop installed. Open the programmer app and update the JLink version if necessary. Add the following two files in your build directory::

  merged_CPUNET.hex   # For network core
  merged.hex          # For application core

Then click Erase & write, wait until the three LEDs on the wearable flashes. 

5. Read External Flash File
************************

To extract file saved in the external flash, we use `littlefs-fuse <https://github.com/littlefs-project/littlefs-fuse>`_ 
After setting up, navigate to littlefs-fuse, make sure to have sudo right for the following operation::

  sudo chmod a+rw /dev/sda # Run lsblk to confirm the disk name
  mkdir mount
  ./lfs --block_count=1760 --block_size=4096 --read_size=16 --prog_size=16 --cache_size=64 --lookahead_size=32 /dev/sda mount 

Make sure the block size, read size, prog size, cache size and lookahead size is the same as the output from your application::

  [00:00:00.059,661] <inf> littlefs: LittleFS version 2.8, disk version 2.1  
  [00:00:00.061,065] <inf> littlefs: FS at mx25r6435f@0:0x120000 is 1760 0x1000-byte blocks with 512 cycle
  [00:00:00.061,065] <inf> littlefs: sizes: rd 16 ; pr 16 ; ca 64 ; la 32

The above log shows: block size = 4096 as 0x1000, read size = 16 as rd 16, prog size=16 as pr 16, cache size = 64 as ca 64, lookahead size = 32 as la 32.
After extracting the file, you can use::

  cd ..
  umount mount

6. Use with Extended Advertisement
*******************************

Extended advertisement is a new feature introduced since Bluetooth 5.0. Before implementing it, one has to make sure that the controller on both receiver
and the transimitter support extended advertisement. Most commercially available USB Bluetooth dongles does NOT support this function.

You will also need to compile and run a programme on RPi to process the extended packet, see `tools/hci_usb_ext_receiver` for more details.

6.1 Configure Static Random Address
------------------------------

To configure the static random address, you need to set the variable `wearable_static_addr` in `src/main.c` to the desired address. The address should be a 6-byte array, for example::

    EE:54:52:53:00:00

where the two MS-bits of the first byte must bt set to 1, this means you can choose from 0xC0 to 0xFF. The rest bytes can be selected freely. We use ASCII representation if TRS, short for TORUS, 54:52:53 as an example. The last 2 bytes can be an incrementing number or house number OR participant number OR wearable number. 
