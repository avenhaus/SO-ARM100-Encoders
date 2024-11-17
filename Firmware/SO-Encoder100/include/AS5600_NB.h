/* 
   Minimal non blocking AS5600  driver using the non-blocking I2C library.

   Register defines were taken from the AS5600 library by Rob Tillaart.
   URL: https://github.com/RobTillaart/AS5600
*/

#pragma once
#include "Arduino.h"
#include "i2c.h"

//  default addresses
const uint8_t AS5600_DEFAULT_ADDRESS    = 0x36;
const uint8_t AS5600L_DEFAULT_ADDRESS   = 0x40;
const uint8_t AS5600_SW_DIRECTION_PIN   = 255;


//  CONFIGURATION REGISTERS
const uint8_t AS5600_ZMCO = 0x00;
const uint8_t AS5600_ZPOS = 0x01;   //  + 0x02
const uint8_t AS5600_MPOS = 0x03;   //  + 0x04
const uint8_t AS5600_MANG = 0x05;   //  + 0x06
const uint8_t AS5600_CONF = 0x07;   //  + 0x08

//  CONFIGURATION BIT MASKS - byte level
const uint8_t AS5600_CONF_POWER_MODE    = 0x03;
const uint8_t AS5600_CONF_HYSTERESIS    = 0x0C;
const uint8_t AS5600_CONF_OUTPUT_MODE   = 0x30;
const uint8_t AS5600_CONF_PWM_FREQUENCY = 0xC0;
const uint8_t AS5600_CONF_SLOW_FILTER   = 0x03;
const uint8_t AS5600_CONF_FAST_FILTER   = 0x1C;
const uint8_t AS5600_CONF_WATCH_DOG     = 0x20;


//  UNKNOWN REGISTERS 0x09-0x0A

//  OUTPUT REGISTERS
const uint8_t AS5600_RAW_ANGLE = 0x0C;   //  + 0x0D
const uint8_t AS5600_ANGLE     = 0x0E;   //  + 0x0F

// I2C_ADDRESS REGISTERS (AS5600L)
const uint8_t AS5600_I2CADDR   = 0x20;
const uint8_t AS5600_I2CUPDT   = 0x21;

//  STATUS REGISTERS
const uint8_t AS5600_STATUS    = 0x0B;
const uint8_t AS5600_AGC       = 0x1A;
const uint8_t AS5600_MAGNITUDE = 0x1B;   //  + 0x1C
const uint8_t AS5600_BURN      = 0xFF;

//  STATUS BITS
const uint8_t AS5600_MAGNET_HIGH   = 0x08;
const uint8_t AS5600_MAGNET_LOW    = 0x10;
const uint8_t AS5600_MAGNET_DETECT = 0x20;


//  setDirection
const uint8_t AS5600_CLOCKWISE         = 0;  //  LOW
const uint8_t AS5600_COUNTERCLOCKWISE  = 1;  //  HIGH

class AS5600_NB  {
public:
    typedef enum state_enum {
        AS_IDLE,
        AS_READ_ANGLE
    } state_t;
    
    AS5600_NB(I2C& i2c, uint8_t direction_pin=255, uint8_t address = AS5600_DEFAULT_ADDRESS) : _i2c(i2c) {
        _direction_pin = direction_pin;
        _address = address;
    }
    bool begin();
    bool run(u_int32_t now=0);

    void setDirection(uint8_t direction) {
        _direction = direction;
        if (_direction_pin != AS5600_SW_DIRECTION_PIN)
        {
            digitalWrite(_direction_pin, _direction);
        }
    }

    inline bool isConnected() { return _is_connected; }   
    inline uint8_t getStatus() { return _status; }
    inline uint16_t getAngle() { return _angle; }
    inline int32_t getRotations() { return _rotations; }
    inline int32_t getPosition() { return _angle + (_rotations << 12); }
    inline state_t getState() { return _state; }
  
    inline void setState(state_t state) { _state = state; }
    inline void start() { _state = AS_READ_ANGLE; }
    inline void stop() { _state = AS_IDLE; }


protected:
    I2C& _i2c;
    uint8_t _address;
    uint8_t _direction_pin = 255;
    uint8_t _direction = AS5600_CLOCKWISE;
    bool _is_connected = false;
    uint8_t _status = 0;
    uint8_t _data[2] = {0}; // Buffer to receive data and swap bytes for endianness.
    uint16_t _angle = 0xFFFF;
    uint16_t _old_angle = 0;
    int32_t _rotations = 0;
    state_t _state = AS_IDLE;
};