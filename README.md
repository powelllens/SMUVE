# SMUVE
litte rover

Functionality:
MainBoard Arduino Uno:
Master Board, Serial I/O to other Systems like Raspberry/Computer
Connected to USB Host Shield for USB interaction
USB Host Shield connected to Bluetooth Dongle for PS3 Controller support

All other Boards are connected via I2C

Ultrasonic Modul Arduino Mini Pro:
Connected as I2C Slave
Handles up to 10 Ultrasonic Modules provides distance for each modul in one byte in cm

Motor Driver Modul Arduino Mini Pro:
Connected as I2C Slave
Controlls 2 Motors with Hall Sensors:
- PWM
- Direction
- Enable
- Current Sensor

PID Controller for Speed
PI Controller for Current limiting

Complete MotorDriver functionality is provided via MotorDriver Class inside the lib
