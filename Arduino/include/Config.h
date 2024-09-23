#pragma once

#include <Arduino.h>

/********************************************\
|*  Pin Definitions
\********************************************/

/*
ESP32 S3 Pins
https://www.wemos.cc/en/latest/s3/s3.html
---+------+----+-----+-----+-----------+---------------------------
No.| GPIO | IO | RTC | ADC | Default   | Function
---+------+----+-----+-----+-----------+---------------------------
   |   0* | IO |   0 |     | Boot      | Button
   |   1  | IO |   1 | 1_0 |           | 
   |   2  | IO |   2 | 1_1 |           | 
   |   3* | IO |   3 | 1_2 |           | 
   |   4  | IO |   4 | 1_3 |           | CS0
   |   5  | IO |   5 | 1_4 |           | CS1
   |   6  | IO |   6 | 1_5 |           | CS2
   |   7  | IO |   7 | 1_6 |           | CS3
   |   8  | IO |   8 | 1_7 |           | CS4
   |   9  | IO |   9 | 1_8 |           | CS5
   |  10  | IO |  10 | 1_9 | SPI-SS    | 
   |  11  | IO |  11 | 2_0 | SPI-MOSI  | MOSI
   |  12  | IO |  12 | 2_1 | SPI-SCK   | SCK (MT6701-CLK)
   |  13  | IO |  13 | 2_2 | SPI-MISO  | MISO (MT6701-DOut)
   |  14  | IO |  14 | 2_3 |           | 
   |  15  | IO |  15 | 2_4 |           |
   |  16  | IO |  16 | 2_5 |           |
   |  17  | IO |  17 | 2_6 |           |
   |  18  | IO |  18 | 2_7 |           |
   |  19  | IO |  19 | 2_8 | USB/JTAG  |
   |  20  | IO |  20 | 2_9 | USB/JTAG  |
   |  21  | IO |  21 |     |           |
   |  38  | IO |     |     |           | RGB LED
   |  39  | IO |     |     |           | 
   |  40  | IO |     |     |           | 
   |  41  | IO |     |     | I2C_SCL   | 
   |  42  | IO |     |     | I2C_SDA   | 
   |  43  | IO |     |     | UART_TX0  |
   |  44  | IO |     |     | UART_RX0  |
   |  45* | IO |     |     |           |
   |  46* | IO |     |     |           |
   |  47  | IO |     |     |           |
   |  48  | IO |     |     |           |
---+------+----+-----+-----+-----------+---------------------------
* Strapping pins: IO0, IO3, IO45, IO46
*/


#define BUTTON_PIN 0
#define RGB_LED_PIN 38

#define I2C_SCL_PIN 41
#define I2C_SDA1_PIN 42
#define I2C_SDA2_PIN 40
#define I2C_SDA3_PIN 39

#define SPI_MOSI_PIN 11
#define SPI_SCK_PIN 12
#define SPI_MISO_PIN 13
#define SPI_SS_PIN 10

#define ENCODER_COUNT 6
#define SPI_CS0_PIN 4
#define SPI_CS1_PIN 5
#define SPI_CS2_PIN 6
#define SPI_CS3_PIN 7
#define SPI_CS4_PIN 8
#define SPI_CS5_PIN 9

#define LOOP_DELAY 100
#define ALWAYS_SEND 0   // 1 to send data even if it hasn't changed

// Programming and debug: connect to UART USB, not OTG
#define LOGGER Serial0

/* ============================================== *\
 * Constants
\* ============================================== */

#define __PROJECT_NAME__ "SO-ARM100-Leader";
#define __PROJECT_COMPILE__ "0.2";
#define SERIAL_DEBUG 1
#define SERIAL_SPEED 115200

#define MT6701_ALTERNATE_ADDRESS 0x46

extern const char EMPTY_STRING[];
extern const char NEW_LINE[];

extern const char PROJECT_NAME[] PROGMEM;
extern const char PROJECT_VERSION[] PROGMEM;
extern const char COMPILE_DATE[] PROGMEM;
extern const char COMPILE_TIME[] PROGMEM;

#define FST (const char *)F
#define PM (const __FlashStringHelper *)

/* ============================================== *\
 * EEPROM
\* ============================================== */

# define EEPROM_MAGIC 0x4242

typedef struct {
  uint16_t magic;
  float home_angle[ENCODER_COUNT];
} eeprom_data_t;

#define EEPROM_SIZE sizeof(eeprom_data_t)

/* ============================================== *\
 * Debug
\* ============================================== */

#if SERIAL_DEBUG < 1
#define DEBUG_println(...) 
#define DEBUG_print(...) 
#define DEBUG_printf(...) 
#else
#define DEBUG_println(...) if (debugStream) {debugStream->println(__VA_ARGS__);}
#define DEBUG_print(...) if (debugStream) {debugStream->print(__VA_ARGS__);}
#define DEBUG_printf(...) if (debugStream) {debugStream->printf(__VA_ARGS__);}
#endif

extern Print* debugStream;
