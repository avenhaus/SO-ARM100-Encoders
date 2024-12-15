/* 
Minimal non blocking AS5600  driver using the non-blocking I2C library.

Register defines were taken from the AS5600 library by Rob Tillaart.
URL: https://github.com/RobTillaart/AS5600
*/

#include <stdio.h>
#include "ch32v003fun.h"
#include "AS5600_NB.h"
#include "systime.h"


bool as5600_init(as5600_t* dev, uint8_t direction_pin, uint8_t address) {

  dev->address = address;
  dev->direction_pin = direction_pin;

  if (dev->direction_pin != AS5600_SW_DIRECTION_PIN) {
    funPinMode(dev->direction_pin, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    funDigitalWrite(dev->direction_pin, AS5600_CLOCKWISE);
  }
  dev->direction = AS5600_CLOCKWISE;

  dev->status = 0;
  dev->is_connected = false;
  dev->rotations = 0;
  dev->angle = 0xFFFF;
  dev->old_angle = 0xFFFF; // Mark as invalid.
  dev->state = AS_IDLE;

  i2c_read_register(dev->address, AS5600_STATUS, &dev->status, 1);
  i2c_wait_for_idle();

  if (i2c_get_status() != I2C_OK || dev->status == 0) { return false; }
  dev->is_connected = true;

#ifdef AS_DEBUG
  printf("AS5600 begin() status: %d connected: %d\n", dev->status, dev->is_connected);
#endif

  // Read angle register. Following reads do not need to set the register again.
  i2c_read_register(dev->address, AS5600_ANGLE, dev->data, 2);

  as5600_start(dev);
  return dev->is_connected;
}

void as5600_set_direction(as5600_t* dev, uint8_t direction) {
    dev->direction = direction;
    if (dev->direction_pin != AS5600_SW_DIRECTION_PIN) {
        funDigitalWrite(dev->direction_pin, dev->direction);
    }
}


bool as5600_run(as5600_t* dev) {
  if (dev->state == AS_IDLE) { return false; }
  switch (dev->state) {
    case AS_READ_ANGLE:
      if (i2c_is_idle()) {
        // Serial2.print("AS_READ_ANGLE ");
        //           Serial2.print(_data[0], HEX);
        //           Serial2.print(" ");
        //           Serial2.println(_data[1], HEX);

        
        dev->angle = dev->data[0] << 8 | dev->data[1];
        if (dev->old_angle == 0xFFFF) { dev->old_angle = dev->angle;} // First time old_angle is invalid.
        if (dev->old_angle > 1024*3 && dev->angle < 1024) {
          dev->rotations++;
        } else if (dev->old_angle < 1024 && dev->angle > 1024*3) {
          dev->rotations--;
        }
        dev->old_angle = dev->angle;
        dev->data[0] = 0x23;
        dev->data[1] = 0x42;

        // The angle register does not auto increment so we just read directly every time.
        i2c_read(dev->address, dev->data, 2);
        //i2c_read_register(dev->address, AS5600_ANGLE, dev->data, 2);

        // static int cnt = 0;
        // if (cnt++ > 4) {
        //   dev->state = AS_IDLE;
        // }

        return true; // New data available.
    }

    default:
      break;
  }
  return false;
}

