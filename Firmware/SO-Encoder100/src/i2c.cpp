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

#include "i2c.h"

#ifndef I2C_FLAG_TIMEOUT
#define I2C_FLAG_TIMEOUT  (10U)
#endif

#ifndef I2C_FLAG_TIMEOUT_BUSY
#define I2C_FLAG_TIMEOUT_BUSY I2C_FLAG_TIMEOUT
#endif

#ifndef I2C_FLAG_TIMEOUT_START
#define I2C_FLAG_TIMEOUT_START I2C_FLAG_TIMEOUT
#endif

#ifndef I2C_FLAG_TIMEOUT_STOP_BIT_RESET
#define I2C_FLAG_TIMEOUT_STOP_BIT_RESET I2C_FLAG_TIMEOUT
#endif

#ifndef I2C_FLAG_TIMEOUT_ADDR_ACK
#define I2C_FLAG_TIMEOUT_ADDR_ACK I2C_FLAG_TIMEOUT
#endif

#ifndef I2C_FLAG_TIMEOUT_DATA_ACK
#define I2C_FLAG_TIMEOUT_DATA_ACK I2C_FLAG_TIMEOUT
#endif

#ifndef I2C_FLAG_TIMEOUT_BYTE_TRANSMITTED
#define I2C_FLAG_TIMEOUT_BYTE_TRANSMITTED I2C_FLAG_TIMEOUT
#endif

#ifndef I2C_FLAG_TIMEOUT_BYTE_RECEIVED
#define I2C_FLAG_TIMEOUT_BYTE_RECEIVED I2C_FLAG_TIMEOUT
#endif

PROGMEM const char* I2C_STATE[] = {
    (const char *) F("IDLE"),
    (const char *) F("SEND_START"),
    (const char *) F("SEND_RESTART"),
    (const char *) F("SEND_ADDR"),
    (const char *) F("SEND_REGISTER"),
    (const char *) F("SEND_DATA"),
    (const char *) F("SEND_TX_STOP"),
    (const char *) F("SEND_RX_STOP"),
    (const char *) F("SEND_RX_ACK"),
    (const char *) F("RECV_DATA")
};

const char* I2C::getStateName(i2c_state_t state) {
    if (state <= I2C_RECV_DATA) {
        return I2C_STATE[state];
    }
    return (const char *) F("INVALID");
}

bool I2C::read_register(uint8_t address, uint8_t reg, void* rx_buffer, uint8_t rx_bytes/*=1*/) {
    if (_state != I2C_IDLE) { return false; }
    startTransaction(address, 0, 0, rx_buffer, rx_bytes);
    _register = reg;
    _send_register = true;
    return true;
}

bool I2C::write_register(uint8_t address, uint8_t reg, void* tx_buffer, uint8_t tx_bytes/*=1*/) {
    if (_state != I2C_IDLE) { return false; }
    startTransaction(address, tx_buffer, tx_bytes);
    _register = reg;
    _send_register = true;
    return true;
}

bool I2C::startTransaction(uint8_t address, const void* tx_buffer/*=0*/, uint8_t tx_bytes/*=0*/, void* rx_buffer/*=0*/, uint8_t rx_bytes/*=0*/, bool send_stop/*=true*/) {
    if (_state != I2C_IDLE) { return false; }
    _tx_address = address << 1;
    _send_register = false;
    _tx_buffer = (uint8_t*)tx_buffer;
    _tx_count = 0;
    _bytes_to_write = tx_bytes;
    _rx_buffer = (uint8_t*)rx_buffer;
    _rx_count = 0;
    _bytes_to_read = rx_bytes;
    _send_stop = send_stop;
    _status = I2C_OK;
    _startBus();
    return true;
}


void I2C::_startBus() {
    if (!_send_register && _bytes_to_write == 0 && _bytes_to_read > 0) {
        if (_bytes_to_read == 1) {
            /* disable acknowledge */
            i2c_ack_config(_i2c.i2c, I2C_ACK_DISABLE);
            /* send a stop condition to I2C bus*/
        } else if (_bytes_to_read == 2) {
            /* send a NACK for the next data byte which will be received into the shift register */
            i2c_ackpos_config(_i2c.i2c, I2C_ACKPOS_NEXT);
            /* disable acknowledge */
            i2c_ack_config(_i2c.i2c, I2C_ACK_DISABLE);
        } else {
            /* enable acknowledge */
            i2c_ack_config(_i2c.i2c, I2C_ACK_ENABLE);
        }
    }
    i2c_start_on_bus(_i2c.i2c);
    _state = I2C_SEND_START;
    _timeout = millis() + I2C_FLAG_TIMEOUT_START;
}

 // Transition state
I2C::i2c_state_t I2C::_nextState(uint32_t now) {
    switch (_state) {
        case I2C_SEND_START:
            _state = I2C_SEND_ADDR;
            _timeout = now + I2C_FLAG_TIMEOUT_ADDR_ACK;
            if (_send_register || _bytes_to_write > 0) {
                i2c_master_addressing(_i2c.i2c, _tx_address, I2C_TRANSMITTER);
            } else if (_bytes_to_read > 0) {    
                i2c_master_addressing(_i2c.i2c, _tx_address, I2C_RECEIVER);
            } else {
                _state = I2C_IDLE;
                _status = I2C_ERROR;
            }
            break;

        case I2C_SEND_REGISTER:
            _send_register = false;
            if (_bytes_to_write > 0) {
                _state = I2C_SEND_DATA;
                _timeout = now + I2C_FLAG_TIMEOUT_BYTE_TRANSMITTED;
                I2C_DATA(_i2c.i2c) = _tx_buffer[_tx_count++];
            } else if (_send_stop) {
                _state = I2C_SEND_TX_STOP;
                _timeout = now + I2C_FLAG_TIMEOUT_STOP_BIT_RESET;
                i2c_stop_on_bus(_i2c.i2c);
            } else { _state = I2C_IDLE; }
            break;

        case I2C_SEND_ADDR:
            if (_send_register) {
                i2c_flag_clear(_i2c.i2c, I2C_FLAG_ADDSEND);
                _state = I2C_SEND_REGISTER;
                _timeout = now + I2C_FLAG_TIMEOUT_BYTE_TRANSMITTED;
                I2C_DATA(_i2c.i2c) = _register;
            } else if (_bytes_to_write > 0) {
                i2c_flag_clear(_i2c.i2c, I2C_FLAG_ADDSEND);
                _state = I2C_SEND_DATA;
                _timeout = now + I2C_FLAG_TIMEOUT_BYTE_TRANSMITTED;
                I2C_DATA(_i2c.i2c) = _tx_buffer[_tx_count++];
            } else if (_bytes_to_read > 0) {
                _state = I2C_RECV_DATA;
                _timeout = now + I2C_FLAG_TIMEOUT_BYTE_RECEIVED;
                if (_bytes_to_read == 1) {
                    i2c_ack_config(_i2c.i2c, I2C_ACK_DISABLE);
                    i2c_flag_clear(_i2c.i2c, I2C_FLAG_ADDSEND);
                    i2c_stop_on_bus(_i2c.i2c);
                } else if (_bytes_to_read == 2) {
                    /* send a NACK for the next data byte which will be received into the shift register */
                    i2c_ack_config(_i2c.i2c, I2C_ACK_DISABLE);
                    i2c_flag_clear(_i2c.i2c, I2C_FLAG_ADDSEND);
                   _state = I2C_SEND_RX_ACK;
                   _timeout = now + I2C_FLAG_TIMEOUT_DATA_ACK;
                 } else {
                    i2c_ack_config(_i2c.i2c, I2C_ACK_ENABLE);
                    i2c_flag_clear(_i2c.i2c, I2C_FLAG_ADDSEND);
                 }

            } else if (_send_stop) {
                i2c_flag_clear(_i2c.i2c, I2C_FLAG_ADDSEND);
                _state = I2C_SEND_TX_STOP;
                _timeout = now + I2C_FLAG_TIMEOUT_STOP_BIT_RESET;
                i2c_stop_on_bus(_i2c.i2c);
            } else { 
                i2c_flag_clear(_i2c.i2c, I2C_FLAG_ADDSEND);
                _state = I2C_IDLE; 
            }
            break;
            
        case I2C_SEND_DATA:
            _bytes_to_write = 0;
            if (_send_stop) {
                _state = I2C_SEND_TX_STOP;
                _timeout = now + I2C_FLAG_TIMEOUT_STOP_BIT_RESET;
                i2c_stop_on_bus(_i2c.i2c);
            } else { _state = I2C_IDLE; }
            break;

        case I2C_SEND_TX_STOP:
            if (_bytes_to_read > 0) {
                _startBus();
            } else { _state = I2C_IDLE; }
            break;

        case I2C_RECV_DATA:
            if (_send_stop) {
                _state = I2C_SEND_RX_STOP;
                _timeout = now + I2C_FLAG_TIMEOUT_STOP_BIT_RESET;
                if (_bytes_to_read > 2) {
                    i2c_stop_on_bus(_i2c.i2c);
                }
            } else { _state = I2C_IDLE; }
            _bytes_to_read = 0;
            break;

        case I2C_SEND_RX_STOP:
            _state = I2C_IDLE;
            break;

        default:
#ifdef I2C_DEBUG
            I2C_DEBUG.print(F("I2C _nextState() bad transition: "));
            I2C_DEBUG.print(_state);
            I2C_DEBUG.print(F(" "));
            I2C_DEBUG.println(getStateName(_state));
#endif
            _status = I2C_ERROR;
            _state = I2C_IDLE;
            break;
    }
// #ifdef I2C_DEBUG
//             I2C_DEBUG.print(F("I2C new state: "));
//             I2C_DEBUG.print(_state);
//             I2C_DEBUG.print(F(" "));
//             I2C_DEBUG.println(getStateName(_state));
// #endif
    return _state;
}


// Check if state needs transition
I2C::i2c_state_t I2C::run(uint32_t now/*=0*/) {
    if (_state == I2C_IDLE) { return I2C_IDLE; }
    if (now == 0) { now = millis(); }
    if (now > _timeout) {
#ifdef I2C_DEBUG
        I2C_DEBUG.print(getStateName(_state));
        I2C_DEBUG.println(" I2C timeout");
        I2C_DEBUG.println(_rx_count);
        I2C_DEBUG.println(_bytes_to_read);
#endif
        _state = I2C_IDLE;
        _status = I2C_TIMEOUT;
        return _state;
    }
    switch (_state) {
        case I2C_SEND_START:
            if (i2c_flag_get(_i2c.i2c, I2C_FLAG_SBSEND)) {
                _nextState(now);
            }
            break;

        case I2C_SEND_ADDR:
            if (i2c_flag_get(_i2c.i2c, I2C_FLAG_ADDSEND)) {
                _nextState(now);
            }
            break;

        case I2C_SEND_REGISTER:
            if (i2c_flag_get(_i2c.i2c, I2C_FLAG_TBE) != RESET || i2c_flag_get(_i2c.i2c, I2C_FLAG_BTC) != RESET) {
                _nextState(now);
            }

        case I2C_SEND_DATA:
            if (i2c_flag_get(_i2c.i2c, I2C_FLAG_TBE) != RESET || i2c_flag_get(_i2c.i2c, I2C_FLAG_BTC) != RESET) {
                if (_tx_count < _bytes_to_write) {
                    _timeout = now + I2C_FLAG_TIMEOUT_BYTE_TRANSMITTED;
                    I2C_DATA(_i2c.i2c) = _tx_buffer[_tx_count++];
                } else {
                    _nextState(now);
                }
            }
            break;


        case I2C_SEND_RX_ACK:
            if(i2c_flag_get(_i2c.i2c, I2C_FLAG_BTC)) {
               _state = I2C_RECV_DATA;
               _timeout = now + I2C_FLAG_TIMEOUT_BYTE_RECEIVED;
            }
            break;

        case I2C_RECV_DATA:
            if(i2c_flag_get(_i2c.i2c, I2C_FLAG_RBNE)) {
                _rx_buffer[_rx_count++] = i2c_data_receive(_i2c.i2c);
                if (_bytes_to_read == 2) { _rx_buffer[_rx_count++] = i2c_data_receive(_i2c.i2c); }
// #ifdef I2C_DEBUG 
//                 I2C_DEBUG.print(F("I2C Recv: "));
//                 I2C_DEBUG.print(_rx_count);
//                 I2C_DEBUG.print(F(" "));
//                 I2C_DEBUG.println(_rx_buffer[_rx_count-1], HEX);
// #endif
                if (_rx_count < _bytes_to_read) {
                   _state = I2C_SEND_RX_ACK;
                   _timeout = now + I2C_FLAG_TIMEOUT_DATA_ACK;
                    if (_bytes_to_read - _rx_count == 1) {
                        i2c_ack_config(_i2c.i2c, I2C_ACK_DISABLE);
                    }                    
                } else {
                    _nextState(now);
                }
            }
            break;

        case I2C_SEND_TX_STOP:
        case I2C_SEND_RX_STOP:
            if (!((I2C_CTL0(_i2c.i2c) & I2C_CTL0_STOP))) {
                _nextState(now);
            }
            break;

        default:
#ifdef I2C_DEBUG
            I2C_DEBUG.print(F("I2C run() bad State: "));
            I2C_DEBUG.print(_state);
            I2C_DEBUG.print(F(" "));
            I2C_DEBUG.println(getStateName(_state));
#endif
            _status = I2C_ERROR;
            _state = I2C_IDLE;
            break;
    }

    return _state;
}


