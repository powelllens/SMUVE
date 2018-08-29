# SMUVE
## litte rover

## Functionality:
MainBoard Arduino Uno:

## Master Board
- Serial I/O to other Systems like Raspberry/Computer
- Connected to USB Host Shield for USB interaction
- USB Host Shield connected to Bluetooth Dongle for PS3 Controller support
* Modes:
0 - Distance Mode
- Can recieve distances to drive for each motor and each direction with a maximum speed
- PID Controller to regulate the MotorSpeed to reach the settingpoint distance
1 - Error Mode
- Something went wrong (Motor Current/Speed/Sensor/Timing)
2 - PS3 Mode Analog Tank Mode
- Control each Motor via Analog Stick (left side left motor - richt side right motor)
3 - PS3 Mode Analog 2D Joystick
- Control both Motors via the left Analog Stick
4 - PS3 Mode L2/R2 Tank Mode
- Control each Motor via L2/R2 (left side left motor - richt side right motor)
- Direction can be changed by pressing L1/R1 for each motor

* PS3 Controller
- When connected Mode 2 is activated
- Mode 2-4 can be changed by pressing CICRCLE or TRIANGLE
- Mode is indicated by PS3 LED

* Other Boards
- All other Boards are connected via I2C

## Ultrasonic Modul Arduino Mini Pro:
- Connected as I2C Slave
Handles up to 10 Ultrasonic Modules
- provides distance for each modul in one byte (cm)

## Motor Driver Modul Arduino Mini Pro:
- Connected as I2C Slave
Controlls 2 Motors with Hall Sensors:
- PWM
- Direction
- Enable
- Current Sensor

PID Controller for Speed
PI Controller for Current limiting

Complete MotorDriver functionality is provided via MotorDriver Class inside the lib
