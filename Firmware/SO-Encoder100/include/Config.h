#pragma once

#include <Arduino.h>

/********************************************\
|* GD32 Pin Definitions
\********************************************/

#ifdef GD32F1
#define LED_PIN PB1
#define SPI_SS_PIN PA4


#ifdef ARDUINO_GENERIC_F130F8PX
// F8 uses different pins for Serial than F4. Use Serial 2 for PA2/PA3
#define COM Serial2
#define LOGGER Serial2
#define COM_UART UART_1
#elif ARDUINO_GENERIC_F130F4PX
#define COM Serial1
#define LOGGER Serial1
#define COM_UART UART_0
#else 
#define COM Serial
#endif

#define COM_TXD PA2
#define COM_RXD PA3  // Not used in half duplex mode

#define MT6701_CS_PIN PA4
#define AS5600_DIR_PIN PA4

#define SERIAL_DEBUG 0

#else

/********************************************\
|* ESP32 Pin Definitions
\********************************************/

/*
https://drive.google.com/file/d/1gbKM7DA7PI7s1-ne_VomcjOrb0bE2TPZ/view
---+------+----+-----+-----+-----------+---------------------------
No.| GPIO | IO | RTC | ADC | Default   | Function
---+------+----+-----+-----+-----------+---------------------------
25 |   0* | IO | R11 | 2_1 | Boot      | 
35 |   1  | IO |     |     | UART0_TXD | USB Programming/Debug
24 |   2* | IO | R12 | 2_2 |           | 
34 |   3  | IO |     |     | UART0_RXD | USB Programming/Debug
26 |   4* | IO | R10 | 2_0 |           | 
29 |   5* | IO |     |     | SPI0_SS   | LED
14 |  12* | IO | R15 | 2_5 |           | 
16 |  13  | IO | R14 | 2_4 |           | 
13 |  14  | IO | R16 | 2_6 |           | MT6701 CS / AS5600 DIR
23 |  15* | IO | R13 | 2_3 |           | 
27 |  16+ | IO |     |     | UART2_RXD | BUS_RXD
28 |  17+ | IO |     |     | UART2_TXD | BUS_TXD 
30 |  18  | IO |     |     | SPI0_SCK  | MT6701 SCK
31 |  19  | IO |     |     | SPI0_MISO | MT6701 MISO
33 |  21  | IO |     |     | I2C0_SDA  | AS5600 SDA
36 |  22  | IO |     |     | I2C0_SCL  | AS5600 SCL
37 |  23  | IO |     |     | SPI0_MOSI | 
10 |  25  | IO | R06 | 2_8 |DAC1/I2S-DT| 
11 |  26  | IO | R07 | 2_9 |DAC2/I2S-WS| 
12 |  27  | IO | R17 | 2_7 | I2S-BCK   | 
8  |  32  | IO | R09 | 1_4 |           | 
9  |  33  | IO | R08 | 1_5 |           | 
6  |  34  | I  | R04 | 1_6 |           | 
7  |  35  | I  | R05 | 1_7 |           | 
4  |  36  | I  | R00 | 1_0 | SENSOR_VP | 
5  |  39  | I  | R03 | 1_3 | SENSOR_VN | 
3  |  EN  | I  |     |     | RESET     | 
---+------+----+-----+-----+-----------+---------------------------
(IO6 to IO11 are used by the internal FLASH and are not useable)
22 x I/O  + 4 x input only = 26 usable pins 
GPIO_34 - GPIO_39 have no internal pullup / pulldown.
- ADC2 can not be used with WIFI/BT (easily)
+ GPIO 16 and 17 are not available on WROVER (PSRAM)
* Strapping pins: IO0, IO2, IO4, IO5 (HIGH), IO12 (LOW), IO15 (HIGH)
*/


#define BUTTON_PIN 0
#define LED_PIN 5

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define SPI_MOSI_PIN 23
#define SPI_SCK_PIN 18
#define SPI_MISO_PIN 19

#define MT6701_CS_PIN 14
#define AS5600_DIR_PIN 14

#define COM Serial2 
#define COM_TXD 16
#define COM_RXD 17

#define SERIAL_DEBUG 1
#define LOGGER Serial

#endif


// Programming and debug

/* ============================================== *\
 * Constants
\* ============================================== */

#define LOOP_DELAY 100

#define __PROJECT_NAME__ "SO-Encoder100";
#define __PROJECT_COMPILE__ "0.2";

#define COM_SPEED 1000000

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
 * Servo Registers
\* ============================================== */

typedef struct __attribute__((packed)) servo_reg_t {
uint8_t firmware_major;  //  0	0x00	Firmware major version number	1	3	EPROM	read only	-1	-1		
uint8_t firmware_minor;  //  1	0x01	Firmware minor version number	1	7	EPROM	read only	-1	-1		
uint8_t res1;            //  2	0x02	Reserved
uint8_t servo_minor;     //  3	0x03	Servo major version number	1	9	EPROM	read only	-1	-1		
uint8_t servo_major;     //  4	0x04	Servo minor version number	1	3	EPROM	read only	-1	-1		
uint8_t id;              //  5	0x05	ID	1	1	EPROM	read/write	0	253	Baud	A unique identification code on the bus, with no duplicate ID numbers allowed on the same bus. ID number 254 (0xFE) is the broadcast ID, and broadcasts do not receive response packets.
uint8_t baudrate;        //  6	0x06	Baudrate	1	0	EPROM	read/write	0	7	None	"0-7 respectively represent baud rates as follows: 1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400"
uint8_t return_delay;    //  7	0x07	Return delay	1	0	EPROM	read/write	0	254	2us	The minimum unit is 2us, and the maximum allowable setting for response delay is 254*2=508us
uint8_t status_level; //  8	0x08	Response status level	1	1	EPROM	read/write	0	1	None	"0: Except for read and PING instructions, other instructions do not return response packets. 1: Return response packets for all instructions"
uint16_t min_angle;      //  9	0x09	Minimum angle	2	0	EPROM	read/write	-32766	--	Step	Set the minimum value limit for the motion range, which should be smaller than the maximum angle limit. When performing multi-turn absolute position control, this value is set to 0.
uint16_t max_angle;      // 11	0x0B	Maximum angle	2	4095	EPROM	read/write	--	32767	Step	Set the maximum value limit for the motion range, which should be greater than the minimum angle limit. When performing multi-turn absolute position control, this value is set to 0.
uint8_t max_temp;        // 13	0x0D	Maximum temperature	1	70	EPROM	read/write	0	100	°C	The maximum operating temperature limit, when set to 70, means the maximum temperature is 70 degrees Celsius, with a precision setting of 1 degree Celsius.
uint8_t max_voltage;     // 14	0x0E	Maximum input voltage	1	80	EPROM	read/write	0	254	0.1V	If the maximum input voltage is set to 80, then the maximum operating voltage limit is 8.0V, with a precision setting of 0.1V.
uint8_t min_voltage;     // 15	0x0F	Minimum input voltage	1	40	EPROM	read/write	0	254	0.1V	If the minimum input voltage is set to 40, then the minimum operating voltage limit is 4.0V, with a precision setting of 0.1V.
uint16_t max_torque;     // 16	0x10	Maximum torque	2	1000	EPROM	read/write	0	1000	0.10%	Set the maximum output torque limit for the servo motor, where 1000 corresponds to 100% of the locked-rotor torque. Assign this value to address 48 upon power-up as the torque limit.
uint8_t phase_1;         // 18	0x12	Phase	1	12	EPROM	read/write	0	254	None	Special function byte, do not modify unless there are specific requirements. Please refer to the special byte bit analysis for further details.
uint8_t unloading_cond;  // 19	0x13	Unloading conditions	1	44	EPROM	read/write	0	254	None	"Bit0  Bit1  Bit2 Bit3 Bit4 Bit5 set corresponding bit to 1 to enable the corresponding protection. Voltage Sensor Temperature Current Angle Overload set corresponding bit to 0 to disable the corresponding protection"
uint8_t led_alarm_cond;  // 20	0x14	LED alarm conditions	1	47	EPROM	read/write	0	254	None	"Bit0  Bit1  Bit2 Bit3 Bit4 Bit5 set the corresponding bit to 1 to enable flashing  LED. Voltage Sensor Temperature Current Angle Overload set corresponding bit to 0 to disable the corresponding protection"
uint8_t position_pid_p;  // 21	0x15	Position loop P (Proportional) coefficient	1	32	EPROM	read/write	0	254	None	Proportional coefficient of control motor
uint8_t position_pid_d;  // 22	0x16	Position loop D (Differential) coefficient	1	32	EPROM	read/write	0	254	None	Differential coefficient of control motor
uint8_t position_pid_i;  // 23	0x17	Position loop I (Integral) coefficient	1	0	EPROM	read/write	0	254	None	Integral coefficient of the control motor
uint16_t max_start_force;// 24	0x18	Minimum starting force	2	16	EPROM	read/write	0	1000	0.1%	Set the minimum output startup torque for the servo, where 1000 corresponds to 100% of the locked-rotor torque.
uint8_t cw_sensitive_zone;   // 26	0x1A	Clockwise insensitive zone	1	1	EPROM	read/write	0	32	Step	The minimum unit is one minimum resolution angle.
uint8_t cc_sensitive_zone;   // 27	0x1B	Anti-clockwise insensitive zone	1	1	EPROM	read/write	0	32	Step	The minimum unit is a minimum resolution angle.
uint16_t protection_current; // 28	0x1C	Protection current	2	500	EPROM	read/write	0	511	6.5mA	The maximum settable current is 500 * 6.5mA= 3250mA.
uint8_t angle_resolution;    // 30	0x1E	Angle resolution	1	1	EPROM	read/write	1	3	None	For the amplification factor of the minimum resolution angle (degree/step) of the sensor, modifying this value can expand the number of control range. When performing the multi-turn control, you need to modify the parameter at address 0x12 by setting BIT4 to 1. This modification will result in the current position feedback value being adjusted to reflect the larger angle feedback.
uint16_t position_correction;// 31	0x1F	Position correction	2	0	EPROM	read/write	-2047	2047	Step	BIT11 is the direction bit, indicating the positive and negative direction, and other bits can indicate the range of 0-2047 steps.
uint8_t operation_mode;      // 33	0x21	Operation mode	1	0	EPROM	read/write	0	3	None	"0: position servo mode 1: motor constant speed mode, controlled by parameter 0x2E running speed parameter, the highest bit BIT15 is direction bit. 2: PWM open-loop speed regulation mode, controlled by parameter 0x2Cthe  running time parameter, BIT10 is direction bit  3: step servo mode, and the target position of parameter 0x2A is used to indicate the number of steps, and the highest bit BIT15 is the direction bit. When working In mode 3, the minimum and maximum angle limits of 0x9 and 0xB must be set to 0. Otherwise, it is impossible to step indefinitely."
uint8_t protection_torque;   // 34	0x22	Protection torque	1	20	EPROM	read/write	0	100	1.0%	Output torque after entering overload protection. If 20 is set, it means 20% of the maximum torque.
uint8_t protection_time;     // 35	0x23	Protection time	1	200	EPROM	read/write	0	254	10ms	The duration for which the current load output exceeds the overload torque and remains is represented by a value, such as 200, which indicates 2 seconds. The maximum value that can be set is 2.5 seconds.
uint8_t overload_torque;     // 36	0x24	Overload torque	1	80	EPROM	read/write	0	100	1.0%	The maximum torque threshold for starting the overload protection time countdown can be represented by a value, such as 80, indicating 80% of the maximum torque.
uint8_t speed_pid_p;         // 37	0x25	Speed closed-loop proportional (P) coefficient	1	10	EPROM	read/write	0	100	None	Proportional coefficient of speed loop in motor constant speed mode (mode 1)
uint8_t over_current_time;   // 38	0x26	Overcurrent protection time	1	200	EPROM	read/write	0	254	10ms	The maximum setting is 254 * 10ms = 2540ms.
uint8_t velocity_pid_i;      // 39	0x27	Velocity closed-loop integral (I) coefficient	1	10	EPROM	read/write	0	254	1/10	In the motor constant speed mode (mode 1), the speed loop integral coefficient (change note: the speed closed loop I integral coefficient is reduced by 10 times compared with version 3.6).
uint8_t torque_switch;       // 40	0x28	Torque switch	1	0	SRAM	read/write	0	128	None	Write 0: disable the torque output; Write 1: enable the torque output; Write 128: Arbitrary current position correction to 2048.
uint8_t acceleration;        // 41	0x29	Acceleration	1	0	SRAM	read/write	0	254	100 step/s^2	If set to 10, it corresponds to an acceleration and deceleration rate of 1000 steps per second squared.
uint16_t target_location;    // 42	0x2A	Target location	2	0	SRAM	read/write	-30719	30719	Step	Each step corresponds to the minimum resolution angle, and it is used in absolute position control mode. The maximum number of steps corresponds to the maximum effective angle.
uint16_t operation_time;     // 44	0x2C	Operation time	2	0	SRAM	read/write	0	1000	0.10%	In the PWM open-loop speed control mode, the value range is from 50 to 1000, and BIT10 serves as the direction bit.
uint16_t operation_speed;    // 46	0x2E	Operation speed	2	0	SRAM	read/write	0	3400	step/s	Number of steps per unit time (per second), 50 steps per second = 0.732 RPM (revolutions per minute)
uint16_t torque_limit;       // 48	0x30	Torque limit	2	1000	SRAM	read/write	0	1000	1.0%	The initial value of power-on will be assigned by the maximum torque (0x10), which can be modified by the user to control the output of the maximum torque.
uint16_t res2;               // 50
uint16_t res3;               // 52
uint8_t  res4;               // 54
uint8_t  lock_flag;          // 55	0x37	Lock flag	1	0	SRAM	read/write	0	1	None	"Writing 0: Disables the write lock, allowing values written to the EPROM address to be saved even after power loss. Writing 1: Enables the write lock, preventing values written to the EPROM address from being saved after power loss."
uint16_t current_location;   // 56	0x38	Current location	2	0	SRAM	read only	-1	-1	Step	Feedback the number of steps in the current position, each step is a minimum resolution angle; Absolute position control mode, the maximum value corresponds to the maximum effective angle.
uint16_t current_speed;      // 58	0x3A	Current speed	2	0	SRAM	read only	-1	-1	step/s	Feedback the current speed of motor rotation and the number of steps in unit time (per second).
uint16_t current_load;       // 60	0x3C	Current load	2	0	SRAM	read only	-1	-1	0.1%	The voltage duty cycle of the current control output driving motor.
uint8_t  current_voltage;    // 62	0x3E	Current voltage	1	0	SRAM	read only	-1	-1	0.1V	Current servo operation voltage
uint8_t  current_temp;       // 63	0x3F	Current temperature	1	0	SRAM	read only	-1	-1	°C	Current servo internal operating temperature
uint8_t  async_write_flag;   // 64	0x40	Asynchronous write flag	1	0	SRAM	read only	-1	-1	None	The flag bit for using asynchronous write instructions
uint8_t  servo_status;       // 65	0x41	Servo status	1	0	SRAM	read only	-1	-1	None	"Bit0  Bit1  Bit2 Bit3 Bit4 Bit5 the corresponding bit is set to 1 to indicate that the corresponding error occurs, Voltage Sensor Temperature Current Angle Overload the corresponding bit is set to 0 to indicate that there is no corresponding error."
uint8_t  move_flag;          // 66	0x42	Move flag	1	0	SRAM	read only	-1	-1	None	The sign of the servo is 1 when it is moving, and 0 when it is stopped.
uint8_t  res5;               // 57
uint8_t  res6;               // 58
uint8_t  current_current;    // 69	0x45	Current current	2	0	SRAM	read only	-1	-1	6.5mA	The maximum measurable current is 500 * 6.5mA= 3250mA.
uint8_t  res7;               // 71
} servo_reg_t;

#define PACKET_ID 2
#define PACKET_LENGTH 3
#define PACKET_INSTRUCTION 4
#define PACKET_ERROR 4
#define PACKET_ADDR 5
#define PACKET_LEN 6
#define PACKET_DATA 6

#define EEPROM_BYTES 40

/* ============================================== *\
 * EEPROM
\* ============================================== */

# define EEPROM_MAGIC 0x4242

typedef struct __attribute__((packed)) eeprom_data_t {
  uint16_t magic;
  union registers {
    servo_reg_t reg;
    uint8_t data[256];
  } registers;
} eeprom_data_t;

#define EEPROM_SIZE (EEPROM_BYTES + sizeof(uint16_t))

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
