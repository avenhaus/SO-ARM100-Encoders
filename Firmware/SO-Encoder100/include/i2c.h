/*
  Non blocking I2C library for GD32F microcontrollers (and possibly STM32)
  
  The logic is based around a state machine. The state machine is driven by the run() function which should be called
  frequently (e.g. in the main loop). The run() function always returns immediately. I if the state machine switched to idle,
  the transaction is finished and any result is ready to be consumed.

  Some definitions and logic is taken from the Wire library for Arduino:

  TwoWire.h - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

#include "Arduino.h"
#include "utility/twi.h"

#define I2C_MASTER_ADDRESS 0x33

#if !defined(WIRE_BUFFER_LENGTH)
#define WIRE_BUFFER_LENGTH 32
#endif

#define MASTER_ADDRESS 0x33

//#define I2C_DEBUG Serial2

class I2C {
public:
    typedef enum i2c_state {
        I2C_IDLE,
        I2C_SEND_START,
        I2C_SEND_RESTART,
        I2C_SEND_ADDR,
        I2C_SEND_REGISTER,
        I2C_SEND_DATA,
        I2C_SEND_TX_STOP,
        I2C_SEND_RX_STOP,
        I2C_SEND_RX_ACK,
        I2C_RECV_DATA
    } i2c_state_t;

    I2C(int i2c_index, uint8_t sda, uint8_t scl, uint32_t speed=100000) {
        _i2c.sda = DIGITAL_TO_PINNAME(sda);
        _i2c.scl = DIGITAL_TO_PINNAME(scl);

        _i2c.rx_buffer_ptr = 0;
        _i2c.tx_buffer_ptr = 0;
        _i2c.tx_rx_buffer_size = (uint16_t) 0;
        _i2c.tx_count = 0;
        _i2c.rx_count = 0;
        _i2c.index = i2c_index;

        _speed = speed;
        _own_address = I2C_MASTER_ADDRESS << 1;
        _state = I2C_IDLE;
    }

    void begin() {
        i2c_init(&_i2c, _i2c.sda, _i2c.scl, _own_address);
        setClock(_speed);
    }

    void end(void) {
        waitForIdle();
        i2c_deinit(_i2c.i2c);
    }

    bool read_register(uint8_t address, uint8_t reg, void* rx_buffer, uint8_t rx_bytes=1);
    bool write_register(uint8_t address, uint8_t reg, void* tx_buffer, uint8_t tx_bytes=1);
    bool startTransaction(uint8_t address, const void* tx_buffer=0, uint8_t tx_bytes=0, void* rx_buffer=0, uint8_t rx_bytes=0, bool send_stop=true);
    inline bool read(uint8_t address, void* rx_buffer, uint8_t rx_bytes=1, bool send_stop=true) { return startTransaction(address, 0, 0, rx_buffer, rx_bytes, send_stop); }
    inline bool write(uint8_t address, const void* tx_buffer, uint8_t tx_bytes=1, bool send_stop=true) { return startTransaction(address, tx_buffer, tx_bytes, 0, 0, send_stop); }

    i2c_state_t run(uint32_t now=0);

    inline bool isIdle() { return _state == I2C_IDLE; }
    void waitForIdle() {
        while (!isIdle()) { 
            /* Busy wait for transaction to be finished. */ 
            run();
        } 
    }

    void setClock(uint32_t clock_hz)
    {
        // clock can only be changed while the I2C peripheral is **off**.
        i2c_disable(_i2c.i2c);
        i2c_set_clock(&_i2c, clock_hz);
        i2c_enable(_i2c.i2c);
    }

    inline i2c_state_t getState() { return _state; }
    const char* getStateName(i2c_state_t state);
    inline i2c_status_enum getStatus() { return _status; }

protected:
    void _startBus();
    i2c_state_t _nextState(uint32_t now);
    
    i2c_t _i2c;
    uint32_t _speed = 100000;
    uint8_t _own_address = 0;
    uint8_t _tx_address = 0;
    bool _send_stop = false;
    uint8_t _register;
    bool _send_register = false;
    uint8_t* _tx_buffer = 0;
    size_t _tx_count = 0;
    size_t _bytes_to_write = 0;
    uint8_t* _rx_buffer = 0;
    size_t _rx_count = 0;
    size_t _bytes_to_read = 0;
    uint32_t _timeout = 0;
    i2c_state_t _state = I2C_IDLE;
    i2c_status_enum _status = I2C_OK;
};


#if 0
================================================================

#include <stdint.h>
#include <twi.h>

typedef enum {
#if defined(I2C0)
    I2C0_INDEX,
#endif
#if defined(I2C1)
    I2C1_INDEX,
#endif
#if defined(I2C2)
    I2C2_INDEX,
#endif
    I2C_NUM
} internal_i2c_index_t;

#define I2C_MASTER_ADDRESS 0x33

typedef struct i2c_s {
    /* basic information */
    uint32_t i2c;
    uint8_t index;
    PinName sda;
    PinName scl;

    /* operating parameters */
    uint8_t    buffer[32];

    void* pWireObj;
} i2c_t;

typedef enum {
    /* transfer status */
    I2C_OK            = 0,
    I2C_DATA_TOO_LONG = 1,
    I2C_NACK_ADDR     = 2,
    I2C_NACK_DATA     = 3,
    I2C_ERROR         = 4,
    I2C_TIMEOUT       = 5,
    I2C_BUSY          = 6
} i2c_status_enum;


#include "pinmap.h"
#include "gd32_def.h"
#include "PeripheralPins.h"
#include "gd32xxyy.h"

void i2c_init(i2c_t *obj, PinName sda, PinName scl, uint32_t speed=100000)
{
    /* find the I2C by pins */
    uint32_t i2c_sda = pinmap_peripheral(sda, PinMap_I2C_SDA);
    uint32_t i2c_scl = pinmap_peripheral(scl, PinMap_I2C_SCL);

    obj->sda = sda;
    obj->scl = scl;
    obj->i2c = pinmap_merge(i2c_sda, i2c_scl);

    switch (obj->i2c) {
        case I2C0:
            /* enable I2C0 clock and configure the pins of I2C0 */
            obj->index = 0;
            rcu_periph_clock_enable(RCU_I2C0);
            break;
        case I2C1:
            /* enable I2C1 clock and configure the pins of I2C1 */
            obj->index = 1;
            rcu_periph_clock_enable(RCU_I2C1);
            break;
#ifdef I2C2
        case I2C2:
            /* enable I2C2 clock and configure the pins of I2C2 */
            obj_s->index = 2;
            rcu_periph_clock_enable(RCU_I2C2);
            break;
#endif
        default:
            break;
    }

    /* configure the pins of I2C */
    pinmap_pinout(sda, PinMap_I2C_SDA);
    pinmap_pinout(scl, PinMap_I2C_SCL);

    /* I2C clock configure */
    i2c_clock_config(obj->i2c, speed, I2C_DTCY_2);

    /* I2C address configure */
    i2c_mode_addr_config(obj->i2c, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C_MASTER_ADDRESS);


    /* enable I2C */
    i2c_enable(obj->i2c);
    /* enable acknowledge */
    i2c_ack_config(obj->i2c, I2C_ACK_ENABLE);
}

#endif
