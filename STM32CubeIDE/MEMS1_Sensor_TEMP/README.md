# Overview
Microcontroller **DISCO-L475VG-IOT01A** is equipped with MEMS sensors. In this project, we use the HTS221. This is an ultracompact sensor for relative humidity and temperature. It includes a sensing element and a mixed signal ASIC to provide the measurement information through 
digital serial interfaces.

# Hardware and Software Requirements
* The __DISCO-L475VG-IOT01A__ development board
* A Laptop or PC connected to the microcontroller via a serial port to display sensor values in real time
* STM32CubeIDE as the development environment

# Importing the project
* Open STM32CubeIDE.
* Navigate to File > Import > General > Import an Existing STM32CubeMX Configuration File (.ioc).
* If prompted to convert the project for compatibility with your IDE version, confirm the conversion.

# High Level Overview
* Configure the necessary peripherals and sensor.
* Assign and connect the I2C peripheral to the sensor hardware.
* Capture and continuously update temperature and relative humidity data.
* Output sensor data via USART serial port for monitoring.

# Applications
* Air conditioning, heating and ventilation
* Air humidifiers
* Refrigerators
* Wearable devices
* Smart home automation
* Industrial automation
* Respiratory equipment
* Asset and goods tracking
