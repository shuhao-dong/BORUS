.. _bluetooth-hci-usb-sample:

Bluetooth: HCI USB
##################

Overview
********

Make a USB Bluetooth dongle out of Zephyr. Requires USB device support from the
board it runs on (e.g. :ref:`nrf52840dk_nrf52840` supports both BLE and USB).

Requirements
************

* Bluetooth stack running on the host (e.g. BlueZ). Current firmware has been tested with BlueZ version 5.72 & 5.82, you can confirm your BlueZ version in terminal with::
  
  bluetoothctl --version

* A board with Bluetooth and USB support in Zephyr. This firmware has been tested with nrf52840dk_nrf52840. 


Building and Running
********************
This sample can be found under :zephyr_file:`samples/bluetooth/hci_usb` in the
Zephyr tree.

See :ref:`bluetooth samples section <bluetooth-samples>` for details.

Extra Configurations
********************

The sample has been tested with nRF52840DK flashed with some extra configurations as below::

  CONFIG_BT=y
  CONFIG_BT_HCI_RAW=y
  CONFIG_BT_HCI=y
  CONFIG_BT_LL_SOFTDEVICE=n
  CONFIG_BT_LL_SW_SPLIT=y
  CONFIG_BT_EXT_ADV=y
  CONFIG_BT_OBSERVER=y
  CONFIG_BT_CENTRAL=y
  CONFIG_BT_PERIPHERAL=y

  CONFIG_BT_CTLR_ADV_EXT=y
  CONFIG_BT_CTLR_PHY_CODED=y
  CONFIG_BT_CTLR_PHY_2M=y

  CONFIG_USB_DEVICE_STACK=y
  CONFIG_USB_DEVICE_PID=0x000B
  CONFIG_USB_DEVICE_BLUETOOTH=y
  CONFIG_USB_DEVICE_BLUETOOTH_VS_H4=n
  CONFIG_USB_DEVICE_INITIALIZE_AT_BOOT=n


  CONFIG_SERIAL=n
  CONFIG_CONSOLE=n
  CONFIG_UART_CONSOLE=n

  CONFIG_BT_BUF_CMD_TX_COUNT=10

  CONFIG_BT_EXT_SCAN_BUF_SIZE=1650
  CONFIG_BT_BUF_EVT_RX_COUNT=16
  CONFIG_BT_CTLR_SCAN_DATA_LEN_MAX=1650
  CONFIG_BT_CTLR_RX_BUFFERS=9

We used the experimental link layer (LL) from Nordic to achieve this. Softdevice LL does not work with RPi when tested. 

On Linux Side
*************

Once plugged in, you should check first if the controller is recognised by Linux::
  
  sudo btmon -i hci1

This should return your controller information and select the btmon to run on hci1. You will need to run `this <https://github.com/shuhao-dong/ble-scan-advertise/tree/feature/ext-scan-sync>`_ 
on your Linux machine to receive the extended advertisement packets. 