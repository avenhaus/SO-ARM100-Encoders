# Controller to read MT6701 magnetic encoders

PlatformIO / Arduino code to read six MT6701 magnetic encoders over SSI (SPI) and send the data to the UART serial port.

The MT6701 has a 14 bit resolution (0-16383).
To address six encoders, 10 wires are needed: VCC, GND, Clock, Data, 6 x CS.

https://www.lcsc.com/datasheet/lcsc_datasheet_2110291630_Magn-Tek-MT6701QT-STD_C2913974.pdf


## Output Format

The output are comma separated values. The first value is the timestamp in ms since the controller reset. This is followed by angle (float) and status (int) pairs for each encoder.

Lines that start with # are text status messages.


## Calibration

Move all arm joints to the home postion and press the button on the dev board.
This will save the current positions to EEPROM. 
All angles from now on will be sent as realtive to this position.

## Connections

* SPI_SCK_PIN 12
* SPI_MISO_PIN 13
* SPI_CS0_PIN 4
* SPI_CS1_PIN 5
* SPI_CS2_PIN 6
* SPI_CS3_PIN 7
* SPI_CS4_PIN 8
* SPI_CS5_PIN 9