/* 
   Minimal non blocking AS5600  driver using the non-blocking I2C library.

   Register defines were taken from the AS5600 library by Rob Tillaart.
   URL: https://github.com/RobTillaart/AS5600
*/

#pragma once
#ifndef AS5600_NB_H
#define AS5600_NB_H

#include "ch32v003fun.h"
#include "i2c.h"

#define AS_DEBUG 1

//  default addresses
static const uint8_t AS5600_DEFAULT_ADDRESS    = 0x36;
static const uint8_t AS5600_SW_DIRECTION_PIN   = 255;


//  CONFIGURATION REGISTERS
static const uint8_t AS5600_ZMCO = 0x00;
static const uint8_t AS5600_ZPOS = 0x01;   //  + 0x02
static const uint8_t AS5600_MPOS = 0x03;   //  + 0x04
static const uint8_t AS5600_MANG = 0x05;   //  + 0x06
static const uint8_t AS5600_CONF = 0x07;   //  + 0x08

//  CONFIGURATION BIT MASKS - byte level
static const uint8_t AS5600_CONF_POWER_MODE    = 0x03;
static const uint8_t AS5600_CONF_HYSTERESIS    = 0x0C;
static const uint8_t AS5600_CONF_OUTPUT_MODE   = 0x30;
static const uint8_t AS5600_CONF_PWM_FREQUENCY = 0xC0;
static const uint8_t AS5600_CONF_SLOW_FILTER   = 0x03;
static const uint8_t AS5600_CONF_FAST_FILTER   = 0x1C;
static const uint8_t AS5600_CONF_WATCH_DOG     = 0x20;


//  UNKNOWN REGISTERS 0x09-0x0A

//  OUTPUT REGISTERS
static const uint8_t AS5600_RAW_ANGLE = 0x0C;   //  + 0x0D
static const uint8_t AS5600_ANGLE     = 0x0E;   //  + 0x0F

// I2C_ADDRESS REGISTERS (AS5600L)
static const uint8_t AS5600_I2CADDR   = 0x20;
static const uint8_t AS5600_I2CUPDT   = 0x21;

//  STATUS REGISTERS
static const uint8_t AS5600_STATUS    = 0x0B;
static const uint8_t AS5600_AGC       = 0x1A;
static const uint8_t AS5600_MAGNITUDE = 0x1B;   //  + 0x1C
static const uint8_t AS5600_BURN      = 0xFF;

//  STATUS BITS
static const uint8_t AS5600_MAGNET_HIGH   = 0x08;
static const uint8_t AS5600_MAGNET_LOW    = 0x10;
static const uint8_t AS5600_MAGNET_DETECT = 0x20;


//  setDirection
static const uint8_t AS5600_CLOCKWISE         = 0;  //  LOW
static const uint8_t AS5600_COUNTERCLOCKWISE  = 1;  //  HIGH

typedef enum as5600_state_enum {
    AS_IDLE,
    AS_READ_ANGLE
} as5600_state_t;

typedef struct as5600_s {
    uint8_t address;
    uint8_t direction_pin;
    uint8_t direction;
    bool is_connected;
    uint8_t status;
    uint8_t data[2]; // Buffer to receive data and swap bytes for endianness.
    uint16_t angle;
    uint16_t old_angle;
    int32_t rotations;
    as5600_state_t state;
} as5600_t;

bool as5600_init(as5600_t* dev, uint8_t direction_pin, uint8_t address);
bool as5600_run(as5600_t* dev);
void as5600_set_direction(as5600_t* dev, uint8_t direction);

inline static bool as5600_is_connected(as5600_t* dev) { return dev->is_connected; }   
inline static uint8_t as5600_get_status(as5600_t* dev) { return dev->status; }
inline static uint16_t as5600_get_angle(as5600_t* dev) { return dev->angle; }
inline static int32_t as5600_get_rotations(as5600_t* dev) { return dev->rotations; }
inline static int32_t as5600_get_position(as5600_t* dev) { return dev->angle + (dev->rotations << 12); }
inline static as5600_state_t as5600_get_state(as5600_t* dev) { return dev->state; }

inline static void as5600_set_state(as5600_t* dev, as5600_state_t state) { dev->state = state; }
inline static void as5600_start(as5600_t* dev) { dev->state = AS_READ_ANGLE; }
inline static void as5600_stop(as5600_t* dev) { dev->state = AS_IDLE; }

#endif