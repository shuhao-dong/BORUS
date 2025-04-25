Overview
********

This is the prototype code for TORUS using Zephyr RTOS to collect sensor data and 
transimit or log data based on the current state of the device. The code is primarily 
tested on Nordic Thingy53. 

Requirements
************

You must use nRF Connect SDK and toolchain version v2.6.0 or above in order to better 
support the partition manager and sysbuild.

For board information, please refer to the datasheet of Thingy53. 

Building and Running
********************

The nRF Connect Extension in VSCode is recommended to build and test the code.

To get started with nRF Connect Extension in VSCode, please refer to the official tutorial.

When flash, please remember the Thingy53 is based on nRF5340 dual-core SoC, you need to flash 
both the application core and the network core to enable functions like BLE.

After the build process, in order to flash the firmware, you will need a JTAG debugger. You can 
either use a dedicated debugger, or use the one that comes with the nRF5340DK. See How to Programme
Thingy53 for more details. 

On successful build and flash, your Thingy53 should light the 3 colour LEDs in series. Note that 
current code disabled log output over UART, you will need a JTAG connector to achieve RTT log output. 

You could also flash the new firmware via USB to perform a DFU. You will need a signed bin file to perform
such an action. It is recommended to use dfu-util command line tool or AuTerm GUI software. 

Build errors
************

Read External Flash File
************************



