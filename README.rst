Overview
********

This is the project code for TORUS using Zephyr RTOS to collect sensor data and 
transimit or log data based on the current state of the device. The code is primarily 
tested on Nordic Thingy53. Helper tools are included.

.. code-block:: none

    BORUS/
    ├── src/                 # Sources files for wearable
    │   ├── driver/          # Driver files
    │   └── main.c           # Main source code firmware
    ├── sysbuild/            # Configuration used for sysbuild
    └── tools/               # Helper tools

Requirements
************

You must use nRF Connect SDK and toolchain version v2.9.1 or above in order to better 
support the partition manager and sysbuild. This project is tested with NCS 2.9.1. Note that is you are using a version below, you have to change app.overlay, 
speficially deleting the msc_disk0 node and the parent node of it. 

This firmware is tested with Thingy53 although the target of the build should be set to torus53

Building and Running
********************

The nRF Connect Extension in VSCode is recommended to build and test the code.

To get started with nRF Connect Extension in VSCode, please refer to the official `tutorial <https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-VS-Code/Tutorials>`_

When flash, please remember the Thingy53 is based on nRF5340 dual-core SoC, you need to flash 
both the application core and the network core to enable functions like BLE.

After the build process, in order to flash the firmware, you will need a JTAG debugger. You can 
either use a dedicated debugger, or use the one that comes with the nRF5340DK. See How to Programme
Thingy53 for more details. 

On successful build and flash, your Thingy53 should light the 3 colour LEDs in series. Note that 
current code disabled log output over UART, you will need a JTAG connector to achieve RTT log output. 

You could also flash the new firmware via USB to perform a DFU. You will need a signed bin file to perform
such an action. It is recommended to use dfu-util command line tool or AuTerm GUI software. 

Read External Flash File
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

Use with Extended Advertisement
*******************************

Extended advertisement is a new feature introduced since Bluetooth 5.0. Before implementing it, one has to make sure that the controller on both receiver
and the transimitter support extended advertisement. Most commercially available USB Bluetooth dongles does NOT support this function.

You will also need to compile and run a programme on RPi to process the extended packet, see `tools/hci_usb_ext_receiver` for more details.

Configure Static Random Address
------------------------------

To configure the static random address, you need to set the variable `wearable_static_addr` in `src/main.c` to the desired address. The address should be a 6-byte array, for example: EE:54:52:53:00:00
The two MS-bits of the first byte must bt set to 1, this means you can choose from 0xC0 to 0xFF. 