# Overview
Microcontroller **DISCO-L475VG-IOT01A** is equipped with MEMS sensors. In this project, we use the LSM6DSL, a MEMS sensor that includes a 3D digital accelerometer and a 3D digital gyroscope. It operates at 0.65 mA in high-performance mode and supports always-on low-power features.

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
* Capture and update 6-axis motion data from the accelerometer and gyroscope.
* Output sensor data via USART serial port for monitoring.

# Applications
* Motion tracking and gesture detection
* Collecting sensor data
* Indoor navigation
* IoT and connected devices
* Intelligent power saving for handheld devices
* Vibration monitoring and compensation
