# Manetic Encoder Dummy Servo

This project is rlated to 
https://github.com/huggingface/lerobot
and
https://github.com/TheRobotStudio/SO-ARM100

It explores to use much cheaper magnetic encoders for the Leader Arm instead of the servos currently used.

## Hardware

### Magnetic Rotary Encoders
Currently there are two cheap and popular magnetic encoders on Aliexpress, the AS5600 MT6701.
The AS5600 is older, 12 bit and supports only I2C. Since the chip has only one hard coded ID, that makes it hard to use when many of them are needed.
The MT6701 has 14 bit resolution and also supports a SSI interface, which is similar to SPI.

Aliexpress offers several MT6701 breakout boards some of which can be purchased in low quantites for less than $2 including magnet.
https://www.aliexpress.com/w/wholesale-MT6701.html

Small Big PCB

![MT6701 Comparison](./Images/Servo_Comparison.jpg)

### 3D Printed Servo Housing




