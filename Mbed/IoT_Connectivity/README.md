# Overview
Microcontroller **L475VG-IOT01A** transmit live on-board sensor data to a mobile app that can communicate with BLE devices.

# Hardware and Software Requirements
* The __DISCO-L475VG-IOT01A__ board
* A mobile phone, with a Bluetooth LE Scanner App; here __BLE Scanner__
* Mbed Studio as the development environment

# Necessary Libraries
In __Mbed Studio__ please go to __File__ and choose __Add library to active program__
* https://os.mbed.com/teams/ST/code/BSP_B-L475E-IOT01/
* https://github.com/ARMmbed/mbed-os-ble-utils

# High Level Overview
* Initialise BLE
* Advertise presence
* Accept connections, and periodically transmit sensor readings over the connection
* Respond to write events, to allow the on-board LED to be toggled

