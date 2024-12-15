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
* 29 Aug 2024	Version 3.3
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
#pragma once

#include "ch32v003fun.h"
#include "stdbool.h"

// TESTED: DEFAULT OK	ALT_1 OK
#define I2C_PINOUT_DEFAULT
//#define I2C_PINOUT_ALT_1
//#define I2C_PINOUT_ALT_2

//#define I2C_DEBUG 1

/*** Hardware Definitions ****************************************************/
// Predefined Clock Speeds
#define I2C_CLK_10KHZ  10000
#define I2C_CLK_50KHZ  50000
#define I2C_CLK_100KHZ 100000
#define I2C_CLK_400KHZ 400000
#define I2C_CLK_500KHZ 500000
#define I2C_CLK_600KHZ 600000
#define I2C_CLK_750KHZ 750000
#define I2C_CLK_1MHZ   1000000

// Hardware CLK Prerate
#define I2C_PRERATE 1000000

#ifndef I2C_FLAG_TIMEOUT
#define I2C_FLAG_TIMEOUT  (50U)
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


// Default Pinout
#ifdef I2C_PINOUT_DEFAULT
	#define I2C_AFIO_REG	((uint32_t)0x00000000)
	#define I2C_PORT_RCC	RCC_APB2Periph_GPIOC
	#define I2C_PORT		GPIOC
	#define I2C_PIN_SCL 	2
	#define I2C_PIN_SDA 	1
#endif

// Alternate 1 Pinout
#ifdef I2C_PINOUT_ALT_1
	#define I2C_AFIO_REG	((uint32_t)0x04000002)
	#define I2C_PORT_RCC	RCC_APB2Periph_GPIOD
	#define I2C_PORT		GPIOD
	#define I2C_PIN_SCL 	1
	#define I2C_PIN_SDA 	0
#endif

// Alternate 2 Pinout
#ifdef I2C_PINOUT_ALT_2
	#define I2C_AFIO_REG	((uint32_t)0x00400002)
	#define I2C_PORT_RCC	RCC_APB2Periph_GPIOC
	#define I2C_PORT		GPIOC
	#define I2C_PIN_SCL 	5
	#define I2C_PIN_SDA 	6
#endif

// Error Code Definitons
typedef enum {
	I2C_OK	  = 0,   // No Error. All OK
	I2C_ERR_BERR,	 // Bus Error
	I2C_ERR_NACK,	 // ACK Bit failed
	I2C_ERR_ARLO,	 // Arbitration Lost
	I2C_ERR_OVR,	 // Overun/underrun condition
	I2C_ERR_BUSY,	 // Bus was busy and timed out
    I2C_ERR_TIMEOUT, // Generic Timeout
    I2C_ERR_ERROR,   // Generic Error
} i2c_err_t;


typedef enum i2c_state {
    I2C_DISABLED,
    I2C_IDLE,
    I2C_SEND_START,
    I2C_SEND_ADDR_TX,
    I2C_SEND_ADDR_RX,
    I2C_SEND_REGISTER,
    I2C_SEND_DATA,
    I2C_RECV_DATA,
    I2C_SEND_STOP
} i2c_state_t;


/*** Functions ***************************************************************/
/// @brief Initialise the I2C Peripheral on the default pins, in Master Mode
/// @param clk_rate that the I2C Bus should use in Hz. Max 400000
/// @return i2c_err_t, I2C_OK On success
i2c_err_t i2c_init(const uint32_t clk_rate);

/// @brief Pings a specific I2C Address, and returns a i2c_err_t status
/// @param addr I2C Device Address, MUST BE 7 Bit
/// @return i2c_err_t, I2C_OK if the device responds
i2c_err_t i2c_ping(const uint8_t addr);

/// @brief Scans through all 7 Bit addresses, prints any that respond
/// @param callback function - returns void, takes uint8_t
/// @return None
void i2c_scan(void (*callback)(const uint8_t));

/// @brief reads [len] bytes from [addr]s [reg] register into [buf]
/// @param addr, address of I2C Device to Read from, MUST BE 7 Bit
/// @param reg, register to read from
/// @param buf, buffer to read to
/// @param len, number of bytes to read
/// @return 12c_err_t. I2C_OK on Success
i2c_err_t i2c_read_register(const uint8_t addr,	const uint8_t reg,
										uint8_t *buf,
										const uint8_t len);

/// @brief writes [len] bytes from [buf], to the [reg] of [addr]
/// @param addr, Address of the I2C Device to Write to, MUST BE 7 Bit
/// @param reg, Register to write to
/// @param buf, Buffer to write from
/// @param len, number of bytes to read
/// @return i2c_err_t. I2C_OK On Success.
i2c_err_t i2c_write_register(const uint8_t addr,	const uint8_t reg,
										const uint8_t *buf,
										const uint8_t len);



/// @brief reads [len] bytes from [addr]s [reg] register into [buf]
/// @param addr, address of I2C Device to Read from, MUST BE 7 Bit
/// @param buf, buffer to read to
/// @param len, number of bytes to read
/// @return 12c_err_t. I2C_OK on Success
i2c_err_t i2c_read(const uint8_t addr,	uint8_t *buf, const uint8_t len);

/// @brief writes [len] bytes from [buf], to the [reg] of [addr]
/// @param addr, Address of the I2C Device to Write to, MUST BE 7 Bit
/// @param buf, Buffer to write from
/// @param len, number of bytes to read
/// @return i2c_err_t. I2C_OK On Success.
i2c_err_t i2c_write(const uint8_t addr, const uint8_t *buf, const uint8_t len);

/// @brief Busy wait until the I2C Bus is idle
/// @return i2c_err_t. I2C_OK On Success.
i2c_err_t i2c_wait_for_idle();

i2c_state_t i2c_run();

i2c_err_t i2c_get_status();

bool i2c_is_idle();

