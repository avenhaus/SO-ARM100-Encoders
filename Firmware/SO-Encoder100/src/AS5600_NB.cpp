/* 
Minimal non blocking AS5600  driver using the non-blocking I2C library.

Register defines were taken from the AS5600 library by Rob Tillaart.
URL: https://github.com/RobTillaart/AS5600
*/

#include "AS5600_NB.h"

bool AS5600_NB::begin() {

  if (_direction_pin != AS5600_SW_DIRECTION_PIN) {
    pinMode(_direction_pin, OUTPUT);
  }
  setDirection(AS5600_CLOCKWISE);

  _status = 0;
  _is_connected = false;
  _rotations = 0;
  _old_angle = 0xFFFF; // Mark as invalid.

  _i2c.read_register(_address, AS5600_STATUS, &_status);
  _i2c.waitForIdle();

#ifdef AS_DEBUG
  AS_DEBUG.print(F("AS5600 begin() status: "));
  AS_DEBUG.print(_status);
  AS_DEBUG.print(F("connected: "));
  AS_DEBUG.println(_is_connected);
#endif

  if (_i2c.getStatus() != I2C_OK || _status == 0) {
    return _is_connected;
  }
  _is_connected = true;

  // Read angle register. Following reads do not need to set the register again.
  _i2c.read_register(_address, AS5600_ANGLE, &_data, 2);
  start();
  return _is_connected;
}

bool AS5600_NB::run(uint32_t now/*=0*/) {
  if (_state == AS_IDLE) { return false; }
  if (now==0) { now = millis(); }
  switch (_state) {

    case AS_READ_ANGLE:
      if (_i2c.isIdle()) {
        // Serial2.print("AS_READ_ANGLE ");
        //           Serial2.print(_data[0], HEX);
        //           Serial2.print(" ");
        //           Serial2.println(_data[1], HEX);

        
        _angle = _data[0] << 8 | _data[1];
        if (_old_angle == 0xFFFF) { _old_angle = _angle;} // First time old_angle is invalid.
        if (_old_angle > 1024*3 && _angle < 1024) {
          _rotations++;
        } else if (_old_angle < 1024 && _angle > 1024*3) {
          _rotations--;
        }
        _old_angle = _angle;
        _data[0] = 0x42;
        _data[1] = 0x42;

        // The angle register does not auto increment so we just read directly every time.
        _i2c.read(_address, &_data, 2);

        // static int cnt = 0;
        // if (cnt++ > 4) {
        //   _state = AS_IDLE;
        // }

        return true; // New data available.
    }

    default:
      break;
  }
  return false;
}

