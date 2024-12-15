/******************************************************************************
* Lightweight and simple CH32V0003 I2C Library.
*
* This library provides functions to init, read and write to the hardware I2C
* Bus - in Default, and Alternative Pinout Modes.
* Default:	SCL = PC2		SDA = PC1
* Alt 1:	SCL = PD1		SDA = PD0
* Alt 2:	SCL = PC5		SDA = PC6
*
* See GitHub Repo for more information: 
* https://github.com/ADBeta/CH32V000x-lib_i2c
*
* Released under the MIT Licence
* Copyright ADBeta (c) 2024
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the 
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
* sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* The above copyright notice and this permission notice shall be included in 
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
* OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE 
* USE OR OTHER DEALINGS IN THE SOFTWARE.
******************************************************************************/
#include <stddef.h>
#include <stdbool.h>
#include "config.h"
#include "i2c.h"
#include "systime.h"

#ifdef I2C_DEBUG
#include <stdio.h>
const char* I2C_STATE[] = {
    (const char *) "DISABLED",
    (const char *) "IDLE",
    (const char *) "SEND_START",
    (const char *) "SEND_ADDR_TX",
    (const char *) "SEND_ADDR_RX",
    (const char *) "SEND_REGISTER",
    (const char *) "SEND_DATA",
    (const char *) "RECV_DATA",
    (const char *) "SEND_STOP"
};

const char* i2c_get_state_name(i2c_state_t state) {
    if (state <= I2C_SEND_STOP) {
        return I2C_STATE[state];
    }
    return (const char *) "INVALID";
}
#endif

i2c_state_t _i2c_state = I2C_DISABLED;
i2c_err_t _i2c_status = I2C_OK;
uint32_t _i2c_timeout = 0;
uint8_t _i2c_address = 0;
uint8_t _i2c_register = 0;
bool _i2c_send_register = false;
uint8_t _i2c_rx_bytes = 0;
uint8_t _i2c_tx_bytes = 0;
uint8_t* _i2c_buffer = 0;

/*** Static Functions ********************************************************/
/// @brief Checks the I2C Status against a mask value, returns 1 if it matches
/// @param Status To match to
/// @return uint32_t masked status value: 1 if mask and status match
__attribute__((always_inline))
static inline uint32_t i2c_status(const uint32_t status_mask)
{
	uint32_t status = (uint32_t)I2C1->STAR1 | (uint32_t)(I2C1->STAR2 << 16);
	return (status & status_mask) == status_mask; 
}

/// @brief Gets and returns any error state on the I2C Interface, and resets
/// the bit flags
/// @param none
/// @return i2c_err_t error value
__attribute__((always_inline))
static inline i2c_err_t i2c_error(void)
{
	// BERR
	if(I2C1->STAR1 & I2C_STAR1_BERR) {I2C1->STAR1 &= ~I2C_STAR1_BERR; return I2C_ERR_BERR;}
	// NACK
	if(I2C1->STAR1 & I2C_STAR1_AF) {I2C1->STAR1 &= ~I2C_STAR1_AF; return I2C_ERR_NACK;}
	// ARLO
	if(I2C1->STAR1 & I2C_STAR1_ARLO) {I2C1->STAR1 &= ~I2C_STAR1_ARLO; return I2C_ERR_ARLO;}
	// OVR
	if(I2C1->STAR1 & I2C_STAR1_OVR) {I2C1->STAR1 &= ~I2C_STAR1_OVR; return I2C_ERR_OVR;}

	return I2C_OK;
}

/// @brief Checks the current I2C Status, if it does not have an error state,
/// it defaults to I2C_ERR_BUSY
/// @param None
/// @return i2c_err_t error value
__attribute__((always_inline))
static inline uint32_t i2c_get_busy_error(void)
{
	i2c_err_t i2c_err = i2c_error();
	if(i2c_err == I2C_OK) i2c_err = I2C_ERR_BUSY;
	return i2c_err;
}

void i2c_reset_bus() {
	// Disable the I2C Peripheral
	I2C1->CTLR1 &= ~I2C_CTLR1_PE;

	// Clear, then set the GPIO Settings for SCL and SDA, on the selected port
	I2C_PORT->CFGLR &= ~(0x0F << (4 * I2C_PIN_SDA));
	I2C_PORT->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF) << (4 * I2C_PIN_SDA);	
    funDigitalWrite(I2C_PIN_SDA, FUN_HIGH);
	I2C_PORT->CFGLR &= ~(0x0F << (4 * I2C_PIN_SCL));
	I2C_PORT->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF) << (4 * I2C_PIN_SCL);
    funDigitalWrite(I2C_PIN_SCL, FUN_HIGH);
    Delay_Ms(100);

    // Clock out 9 bits to reset the bus
    for (int i = 0; i < 9; i++) {
        funDigitalWrite(I2C_PIN_SCL, FUN_LOW);
        Delay_Ms(20);
        funDigitalWrite(I2C_PIN_SCL, FUN_HIGH);
        Delay_Ms(20);
    }

    // Stop condition
    funDigitalWrite(I2C_PIN_SDA, FUN_LOW);
    Delay_Ms(20);
    funDigitalWrite(I2C_PIN_SDA, FUN_HIGH);
    Delay_Ms(20);

	// Enable the I2C Peripheral
	I2C1->CTLR1 |= I2C_CTLR1_PE;
}

/*** API Functions ***********************************************************/
i2c_err_t i2c_init(uint32_t clk_rate)
{
    //i2c_reset_bus();

	// Toggle the I2C Reset bit to init Registers
	RCC->APB1PRSTR |=  RCC_APB1Periph_I2C1;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_I2C1;

	// Enable the I2C Peripheral Clock
	RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

	// Enable the selected I2C Port, and the Alternate Function enable bit
	RCC->APB2PCENR |= I2C_PORT_RCC | RCC_APB2Periph_AFIO;

	// Reset the AFIO_PCFR1 register, then set it up
	AFIO->PCFR1 &= ~(0x04400002);
	AFIO->PCFR1 |= I2C_AFIO_REG;

	// Clear, then set the GPIO Settings for SCL and SDA, on the selected port
	I2C_PORT->CFGLR &= ~(0x0F << (4 * I2C_PIN_SDA));
	I2C_PORT->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF) << (4 * I2C_PIN_SDA);	
	I2C_PORT->CFGLR &= ~(0x0F << (4 * I2C_PIN_SCL));
	I2C_PORT->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF) << (4 * I2C_PIN_SCL);

	// Set the Prerate frequency
	uint16_t i2c_conf = I2C1->CTLR2 & ~I2C_CTLR2_FREQ;
	i2c_conf |= (FUNCONF_SYSTEM_CORE_CLOCK / I2C_PRERATE) & I2C_CTLR2_FREQ;
	I2C1->CTLR2 = i2c_conf;

	// Set I2C Clock
	if(clk_rate <= 100000)
	{
		i2c_conf = (FUNCONF_SYSTEM_CORE_CLOCK / (2 * clk_rate)) & I2C_CKCFGR_CCR;
	} else {
		// Fast mode. Default to 33% Duty Cycle
		i2c_conf = (FUNCONF_SYSTEM_CORE_CLOCK / (3 * clk_rate)) & I2C_CKCFGR_CCR;
		i2c_conf |= I2C_CKCFGR_FS;
	}
	I2C1->CKCFGR = i2c_conf;

	// Enable the I2C Peripheral
	I2C1->CTLR1 |= I2C_CTLR1_PE;

	//TODO:
	// Check error states
	if(I2C1->STAR1 & I2C_STAR1_BERR) 
	{
		I2C1->STAR1 &= ~(I2C_STAR1_BERR); 
		return I2C_ERR_BERR;
	}

#ifdef I2C_DEBUG
    printf("I2C: wait for idle after init()\n");
#endif
    while (I2C1->STAR2 & I2C_STAR2_BUSY) {}

    _i2c_state = I2C_IDLE;

#ifdef I2C_DEBUG
        printf("I2C: initialized to %ld Hz\n", clk_rate);
#endif

	return I2C_OK;
}

/*
i2c_err_t i2c_ping(const uint8_t addr)
{
	// Check if bus is busy, if it is, return I2C_ERR_BUSY
	if (I2C1->STAR2 & I2C_STAR2_BUSY) { return  I2C_ERR_BUSY; }

    // Send a START Signal and wait for it to assert
    I2C1->CTLR1 |= I2C_CTLR1_START;
    while(!i2c_status(I2C_EVENT_MASTER_MODE_SELECT));

    // Send the Address and wait for it to finish transmitting
    timeout = I2C_TIMEOUT;
    I2C1->DATAR = (addr << 1) & 0xFE;
    // If the device times out, get the error status - if status is okay,
    // return generic I2C_ERR_BUSY Flag
    while(!i2c_status(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        if(--timeout < 0) {i2c_ret = i2c_get_busy_error(); break;}

	// Send the STOP Signal, return i2c status
	I2C1->CTLR1 |= I2C_CTLR1_STOP;
	return i2c_ret;
}
*/

void i2c_scan(void (*callback)(const uint8_t))
{
	// If the callback function is null, exit
	if(callback == NULL) return;

	// Scan through every address, getting a ping() response
	for(uint8_t addr = 0x00; addr < 0x7F; addr++)
	{
		// If the address responds, call the callback function
		if(i2c_ping(addr) == I2C_OK) callback(addr);
	}
}


// Check if state needs transition
i2c_state_t i2c_run() {
    if (_i2c_state == I2C_IDLE || _i2c_state == I2C_DISABLED) { return I2C_IDLE; }
    uint32_t now = millis();
    if (_i2c_timeout && now > _i2c_timeout) {
#ifdef I2C_DEBUG
        printf("I2C: %s timeout\n", i2c_get_state_name(_i2c_state));
#endif
        _i2c_timeout = 0;
        _i2c_state = I2C_IDLE;
        _i2c_status = I2C_ERR_TIMEOUT;
        return _i2c_state;
    }
#ifdef I2C_DEBUG
    i2c_state_t old_state = _i2c_state;
#endif
    switch (_i2c_state) {
        case I2C_SEND_START:
            if (!i2c_status(I2C_EVENT_MASTER_MODE_SELECT)) { break; }
            // Start condition done => Send Address
            if (_i2c_send_register || _i2c_tx_bytes > 0) {
                I2C1->DATAR = (_i2c_address << 1) & 0xFE; // Address + Write Mode
                _i2c_timeout = now + I2C_FLAG_TIMEOUT_ADDR_ACK;
                _i2c_state = I2C_SEND_ADDR_TX;
            } else if (_i2c_rx_bytes > 0) {
                I2C1->DATAR = (_i2c_address << 1) | 0x01;  // Address + Read Mode
                _i2c_timeout = now + I2C_FLAG_TIMEOUT_ADDR_ACK;
                _i2c_state = I2C_SEND_ADDR_RX;
            } else {
                _i2c_state = I2C_IDLE;
                _i2c_timeout = 0;
                _i2c_status = I2C_ERR_ERROR;
            }
            break;

        case I2C_SEND_ADDR_TX:
        case I2C_SEND_ADDR_RX:
            if (_i2c_state == I2C_SEND_ADDR_TX) {
                if (!i2c_status(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) { break; }
            } else {
                if (!i2c_status(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) { break; }
            }
            
            if (_i2c_send_register) {
                // Address sent => Send the Register Byte
                I2C1->DATAR = _i2c_register;
                _i2c_state = I2C_SEND_REGISTER;
                _i2c_timeout = now + I2C_FLAG_TIMEOUT_BYTE_TRANSMITTED;
            } else if (_i2c_tx_bytes > 0) {
                // Register sent => Send data
                I2C1->DATAR = *_i2c_buffer++;
                _i2c_tx_bytes--;
                _i2c_state = I2C_SEND_DATA;
                _i2c_timeout = now + I2C_FLAG_TIMEOUT_BYTE_TRANSMITTED;
            } else if (_i2c_rx_bytes > 0) {
                // If this is the last byte, send the NACK Bit
			    if(_i2c_rx_bytes == 1) { I2C1->CTLR1 &= ~I2C_CTLR1_ACK; }
                _i2c_state = I2C_RECV_DATA;
                _i2c_timeout = now + I2C_FLAG_TIMEOUT_BYTE_RECEIVED;
            }
            break;

        case I2C_SEND_REGISTER:
            if (!(I2C1->STAR1 & I2C_STAR1_TXE))  { break; }
            _i2c_send_register = false;
            if (_i2c_tx_bytes > 0) {
                // Register sent => Send data
                I2C1->DATAR = *_i2c_buffer++;
                _i2c_tx_bytes--;
                _i2c_state = I2C_SEND_DATA;
                _i2c_timeout = now + I2C_FLAG_TIMEOUT_BYTE_TRANSMITTED;
            } else if (_i2c_rx_bytes > 0) {
                // Register sent => Read data
                // If the message is long enough, enable ACK messages
                if(_i2c_rx_bytes > 1) I2C1->CTLR1 |= I2C_CTLR1_ACK;

                // Send a Repeated START Signal and wait for it to assert
                I2C1->CTLR1 |= I2C_CTLR1_START;
                _i2c_state = I2C_SEND_START;
                _i2c_timeout = now + I2C_FLAG_TIMEOUT_START;
            } else {
                // Send Stop
                I2C1->CTLR1 |= I2C_CTLR1_STOP;
                _i2c_state = I2C_SEND_STOP;
                _i2c_timeout = now + I2C_FLAG_TIMEOUT_STOP_BIT_RESET;
            }
            break;


        case I2C_SEND_DATA:
        	if (!(I2C1->STAR1 & I2C_STAR1_TXE)) { break; }
            if (_i2c_tx_bytes > 0) {
                I2C1->DATAR = *_i2c_buffer++;
                _i2c_tx_bytes--;
                _i2c_timeout = now + I2C_FLAG_TIMEOUT_BYTE_TRANSMITTED;
                if((_i2c_status = i2c_error()) != I2C_OK) {
                    _i2c_state = I2C_IDLE;
                    _i2c_timeout = 0;
                }
            } else {
                // Send Stop
                I2C1->CTLR1 |= I2C_CTLR1_STOP;
                _i2c_state = I2C_SEND_STOP;
                _i2c_timeout = now + I2C_FLAG_TIMEOUT_STOP_BIT_RESET;
            }
            break;

        case I2C_RECV_DATA:
            if (!(I2C1->STAR1 & I2C_STAR1_RXNE)) { break; }
            if (_i2c_rx_bytes <= 2) { I2C1->CTLR1 &= ~I2C_CTLR1_ACK; }
            *_i2c_buffer++ = I2C1->DATAR;
            _i2c_rx_bytes--;
            _i2c_timeout = now + I2C_FLAG_TIMEOUT_BYTE_RECEIVED;
            if((_i2c_status = i2c_error()) != I2C_OK) {
                _i2c_state = I2C_IDLE;
                _i2c_timeout = 0;
            }
            else if (_i2c_rx_bytes == 0) {
                I2C1->CTLR1 |= I2C_CTLR1_ACK; 
                I2C1->CTLR1 |= I2C_CTLR1_STOP;
                _i2c_state = I2C_SEND_STOP;
                _i2c_timeout = now + I2C_FLAG_TIMEOUT_STOP_BIT_RESET;
            }
            break;

        case I2C_SEND_STOP:
            if (I2C1->STAR2 & I2C_STAR2_BUSY) { break; }
            _i2c_state = I2C_IDLE;
            _i2c_timeout = 0;
            break;

        default:
            _i2c_state = I2C_IDLE;
            _i2c_status = I2C_ERR_ERROR;
            _i2c_timeout = 0;
            break;
    }
#ifdef I2C_DEBUG
    if (old_state != _i2c_state) {
        printf("I2C: %s => %s | Status: %d\n", i2c_get_state_name(old_state), i2c_get_state_name(_i2c_state), _i2c_status);
    }
#endif
    return _i2c_state;
}
i2c_err_t i2c_read(const uint8_t addr, uint8_t *buf, const uint8_t len) {
	// Check if bus is busy, if it is, return I2C_ERR_BUSY
	if (I2C1->STAR2 & I2C_STAR2_BUSY) { return  I2C_ERR_BUSY; }
	
    // Send a START Signal
    I2C1->CTLR1 |= I2C_CTLR1_START;

    _i2c_address = addr;
    _i2c_register = 0;
    _i2c_send_register = false;
    _i2c_rx_bytes = len;
    _i2c_tx_bytes = 0;
    _i2c_buffer = buf;

    _i2c_state = I2C_SEND_START;

    return I2C_OK;
}

i2c_err_t i2c_read_register(const uint8_t addr,	const uint8_t reg, uint8_t *buf, const uint8_t len) {
	// Check if bus is busy, if it is, return I2C_ERR_BUSY
	if (I2C1->STAR2 & I2C_STAR2_BUSY) { return  I2C_ERR_BUSY; }
	
    i2c_read(addr, buf, len);
    _i2c_register = reg;
    _i2c_send_register = true;

    // printf("I2C: read: %s %d\n", i2c_get_state_name(_i2c_state), _i2c_status);

    return I2C_OK;
}


i2c_err_t i2c_write(const uint8_t addr, const uint8_t *buf, const uint8_t len)
{
	// Check if bus is busy, if it is, return I2C_ERR_BUSY
	if (I2C1->STAR2 & I2C_STAR2_BUSY) { return  I2C_ERR_BUSY; }

    I2C1->CTLR1 |= I2C_CTLR1_START;

    _i2c_address = addr;
    _i2c_register = 0;
    _i2c_send_register = false;
    _i2c_rx_bytes = 0;
    _i2c_tx_bytes = len;
    _i2c_buffer = (uint8_t*) buf;

    _i2c_state = I2C_SEND_START;

	return I2C_OK;
}


i2c_err_t i2c_write_register(const uint8_t addr, const uint8_t reg, const uint8_t *buf, const uint8_t len)
{
	// Check if bus is busy, if it is, return I2C_ERR_BUSY
	if (I2C1->STAR2 & I2C_STAR2_BUSY) { return  I2C_ERR_BUSY; }

    i2c_write(addr, buf, len);
    _i2c_register = reg;
    _i2c_send_register = true;

	return I2C_OK;
}


i2c_err_t i2c_wait_for_idle() {
#ifdef I2C_DEBUG
    printf("I2C: Waiting for idle. State: %s | Status: %d\n", i2c_get_state_name(_i2c_state), _i2c_status);
#endif
    while (_i2c_state != I2C_IDLE) {
        i2c_run();
    }
    return _i2c_status;
}

i2c_err_t i2c_get_status() {
    return _i2c_status;
}

bool i2c_is_idle() {
    return _i2c_state == I2C_IDLE;
}