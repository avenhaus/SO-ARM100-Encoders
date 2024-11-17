/*
https://emanual.robotis.com/docs/en/dxl/protocol1/
https://www.theremino.com/wp-content/uploads/files/SmartMotors/Feetech_Motors_Programming_ENG.pdf

https://download.kamami.pl/p1181056-ST3215_Servo_User_Manual.pdf
https://www.waveshare.com/w/upload/f/f4/ST3215_Servo_User_Manual.pdf
https://www.waveshare.com/wiki/ST3215_Servo


GD32F130FxP6
  Datasheet: https://www.gigadevice.com.cn/Public/Uploads/uploadfile/files/20241008/GD32F130xxDatasheetRev4.0.pdf
  Manual: https://www.gigadevice.com.cn/Public/Uploads/uploadfile/files/20240407/GD32F1x0_User_Manual_Rev3.8.pdf

  GD32F130F4: UART0 (SERIAL1) TX: PA2, RX: PA3
  GD32F130F8: UART1 (SERIAL2) TX: PA2, RX: PA3

Scan:
999: FF FF FD 04 02 00 09 F3
 33: FF FF 00 04 02 00 09 F0
 29: 00 03 00 00 00 04 45 D8
 31: FF FF 01 04 02 00 09 EF 
   FF FF 01 0B 00 03 09 00 09 03 01 00 00 01 D9 FF FF 02 04 02 00 09 EE
 27: 02 03 00 00 00 04 44 3A
 32: FF FF 03 04 02 00 09 ED

Monitor:
 47: FF FF 01 04 02 38 11 AF FF FF 01 13 00 FC 07 00 00 28 04 45 19 00 00 00 00 08 00 00 00 00 56
 47: FF FF 01 04 02 38 11 AF FF FF 01 13 00 FC 07 00 00 28 04 45 19 00 00 00 00 08 01 00 00 00 55

*/

#include <Arduino.h>

#include "Config.h"
#include <SPI.h>
#ifdef GD32F1
// Somehow EEPROM.h does not find this if not included first.
#include <FlashStorage.h>
#endif
#include <EEPROM.h>

// #include "MT6701.h"

#include "i2c.h"
#include "AS5600_NB.h"


PROGMEM const char EMPTY_STRING[] =  "";
PROGMEM const char NEW_LINE[] =  "\n";

PROGMEM const char PROJECT_NAME[] = __PROJECT_NAME__;
PROGMEM const char PROJECT_VERSION[] = __PROJECT_VERSION__;
PROGMEM const char COMPILE_DATE[] = __DATE__;
PROGMEM const char COMPILE_TIME[] = __TIME__;

PROGMEM const servo_reg_t default_data = {
  3,     // firmware_major
  9,     // firmware_minor
  0,     // res1
  9,     // servo_minor
  3,     // servo_major
  2,     // id
  0,     // baudrate
  0,     // return_delay
  1,     // status_level
  0,     // min_angle
  4096,  // max_angle
  70,    // max_temp
  140,   // max_voltage
  30,    // min_voltage
  1000,  // max_torque
  12,    // phase_1
  44,    // unloading_cond
  47,    // led_alarm_cond
  0,     // position_pid_p
  0,     // position_pid_d
  0,     // position_pid_i
  0,     // max_start_force
  0,     // cw_sensitive_zone
  0,     // cc_sensitive_zone
  310,   // protection_current
  1,     // angle_resolution
  0,     // position_correction
  0,     // operation_mode
  20,    // protection_torque
  200,   // protection_time
  80,    // overload_torque
  0,     // speed_pid_p
  0,     // over_current_time
  0,     // velocity_pid_i
  0,     // torque_switch
  0,     // acceleration
  0,     // target_location
  0,     // operation_time
  0,     // operation_speed
  0,     // torque_limit
  0,     // res2
  0,     // res3
  0,     // res4
  1,     // lock_flag
  2048,  // current_location
  0,     // current_speed
  0,     // current_load
  33,    // current_voltage
  24,    // current_temp
  0,     // async_write_flag
  0,     // servo_status
  0,     // move_flag
  0,     // res5
  0,     // res6
  0,     // current_current
  0,     // res7
};

#if SERIAL_DEBUG
Print* debugStream = &LOGGER;
#endif

eeprom_data_t eeprom_data = {0};

servo_reg_t& reg = eeprom_data.registers.reg;
uint8_t* const reg_data = eeprom_data.registers.data;

uint8_t error = 0;

uint8_t rx_buffer[256];
uint8_t rx_index = 0;
uint32_t last_rx_ts = 0;
uint8_t ignore = 0;  // Ignore this many RX bytes. (What we just sent out.)

// MT6701 mt6701;

I2C i2c0(0, SDA, SCL, 1000000);
AS5600_NB as5600(i2c0, AS5600_DIR_PIN);  

uint16_t old_angle = 0;
int32_t rotations = 0;
int32_t position = 0;

void debug_hex(const uint8_t* data, uint16_t len) {
  for (int i = 0; i < len; i++) {
    DEBUG_print(data[i], HEX);
    DEBUG_print(FST(" "));
  }
}

/***********************************************************\
 * Save Registers to EEPROM
\***********************************************************/
void save_eeprom() {
  eeprom_data.magic = EEPROM_MAGIC;
#ifdef GD32F1
  EEPROM.begin();
  EEPROM.put(0, eeprom_data);
  EEPROM.commit();
#endif
#ifdef ESP32
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.writeBytes(0, &eeprom_data, EEPROM_SIZE);
  EEPROM.commit();
  EEPROM.end();
#endif

  DEBUG_println(FST("# Saved data to EEPROM"));
}


/***********************************************************\
 * Write to registers
\***********************************************************/
void write_cmd(uint8_t addr, uint8_t len, const uint8_t* data) {
  uint8_t header[] = {0xFF, 0xFF, reg.id, 2, error};

  // Save the data to the registers.
  for (size_t i = 0; i < len; i++) {reg_data[addr+i] = data[i];}
  
  // Calculate the CRC.
  uint8_t crc = 0;
  for (int i = PACKET_ID; i < sizeof(header); i++) {crc += header[i]; }
  crc = ~crc; // Invert the bits.

  // Send the response.
#ifdef COM
  COM.write(header, sizeof(header));
  COM.write(crc);
#endif
  ignore = sizeof(header) + 1;

#if SERIAL_DEBUG
  debug_hex(data, len);
  DEBUG_print(FST(" => "));
  debug_hex(header, sizeof(header));
  DEBUG_println(crc, HEX);
#endif

}

/***********************************************************\
 * Read from registers
\***********************************************************/
void read_cmd(uint8_t addr, uint8_t len) {
  uint8_t header[] = {0xFF, 0xFF, reg.id, (uint8_t)(len + 2), error};

  // Calculate the CRC.
  uint8_t crc = 0;
  for (int i = PACKET_ID; i < sizeof(header); i++) {crc += header[i]; }
  for (int i = addr; i < addr+len; i++) {crc += reg_data[i]; }
  crc = ~crc; // Invert the bits.

#ifdef COM
  // Send the response.
  COM.write(header, sizeof(header));
  COM.write(reg_data+addr, len);
  COM.write(crc);
  ignore = sizeof(header) + len + 1;
#endif

#if SERIAL_DEBUG and 0
  debug_hex(header, sizeof(header));
  debug_hex(reg_data+addr, len);
  DEBUG_printf(FST("%02X\n"), crc);
#endif
}

/***********************************************************\
 * Execute a command
\***********************************************************/
void execute_command() {
  uint8_t instruction = rx_buffer[PACKET_INSTRUCTION];
  switch(instruction) {
    case 0x01: // Ping
      DEBUG_print(FST("# Ping\n"));
      break;
    case 0x02: // Read
      {
        uint8_t addr = rx_buffer[PACKET_ADDR]; 
        uint8_t len = rx_buffer[PACKET_LEN];
        // DEBUG_printf(FST("# Read Adr:%d Len:%d => "), addr, len);
        read_cmd(addr, len);
      }
      break;
    case 0x03: // Write
      {
        uint8_t addr = rx_buffer[PACKET_ADDR]; 
        uint8_t len = rx_buffer[PACKET_LENGTH]-3;
        DEBUG_print(FST("# Write Adr:"));
        DEBUG_print(addr);
        DEBUG_print(FST("Len:"));
        DEBUG_println(len);
        bool save = false;
        if (addr == 55 && rx_buffer[PACKET_DATA] == 1 && reg.lock_flag == 0) {save = true;}
        write_cmd(addr, len, rx_buffer + PACKET_DATA);
        if (save) {save_eeprom();}
      }
      break;
    case 0x04: // Reg Write
      DEBUG_print(FST("# Reg Write\n"));
      break;
    case 0x05: // Action
      DEBUG_print(FST("# Action\n"));
      break;
    case 0x06: // Factory Reset
      DEBUG_print(FST("# Factory Reset\n"));
      break;
    case 0x08: // Reboot
      DEBUG_print(FST("# Reboot\n"));
      break;
    case 0x83: // Sync Write
      DEBUG_print(FST("# Sync Write\n"));
      break;
    case 0x92: // Bulk Read
      DEBUG_print(FST("# Bulk Read\n"));
      break;
    default:  
      DEBUG_print(FST("# Unknown instruction: "));
      DEBUG_println(instruction);
      break;
  }
}

/***********************************************************\
 * Parse received data and wait for valid commands
\***********************************************************/
void parse_rx_data(uint8_t c) {
  // Ignore any bytes we sent ourselves.
  if (ignore) {
    ignore--;
    return;
  } 

  // Check for start of packet.
  if (rx_index == 0 && c != 0xFF) {
    return;
  }
  if (rx_index == 1 && c != 0xFF) {
    rx_index = 0;
    return;
  }
  rx_buffer[rx_index++] = c;
  last_rx_ts = millis();

  if (rx_index > 5 && rx_buffer[PACKET_LENGTH]+4 == rx_index) {
    if(rx_buffer[PACKET_ID] != reg.id) {  // TODO: Check for broadcast ID.
      // DEBUG_printf(FST("\n# Wrong ID: %02X, Length: %02X, Instruction: %02X\n"), rx_buffer[PACKET_ID], rx_buffer[PACKET_LENGTH], rx_buffer[PACKET_INSTRUCTION]);
      rx_index = 0;
      return;
    }
    // Checksum calculation: Checksum = ~( ID + Length + Instruction + Parameter1 + … Parameter N )
    uint8_t crc = 0;
    for (int i = PACKET_ID; i < rx_buffer[PACKET_LENGTH]+3; i++) {
      crc += rx_buffer[i];
    }
    crc = ~crc; // Invert the bits.
    if (crc == rx_buffer[rx_buffer[PACKET_LENGTH]+3]) {
      execute_command();
    }
    else { 
      DEBUG_print(FST("\n# Bad CRC:"));
      DEBUG_print(crc);
      DEBUG_print(FST(", received: ")); 
      DEBUG_println(rx_buffer[rx_buffer[PACKET_LENGTH]+3]);
    }
    rx_index = 0;
    return;
  }
}


/***********************************************************\
 * Initialization Code
\***********************************************************/

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

#if SERIAL_DEBUG
  LOGGER.begin(SERIAL_SPEED);
  delay(100);
  DEBUG_println(FST("\n\n# " __PROJECT_NAME__  " " __PROJECT_VERSION__ " " __DATE__ " " __TIME__));
#endif

#ifdef GD32F1
  EEPROM.begin();
  EEPROM.get(0, eeprom_data);
#endif
#ifdef ESP32
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.readBytes(0, &eeprom_data, EEPROM_SIZE);
  EEPROM.end();
#endif
  if (eeprom_data.magic == EEPROM_MAGIC) {
    DEBUG_print(FST("# Loaded EEPROM data. ID: "));
    DEBUG_println(reg.id);
  } else {
#if SERIAL_DEBUG
    DEBUG_print(FST("# Invalid EEPROM magic: "));
    DEBUG_print(eeprom_data.magic);
    DEBUG_println(FST(". Using default values."));
    // DEBUG_printf(FST("# Size %d %d => "), sizeof(eeprom_data), EEPROM_SIZE); 
    debug_hex((uint8_t*)&eeprom_data, sizeof(eeprom_data));
    DEBUG_println();
#endif
    memcpy(&eeprom_data.registers, &default_data, sizeof(default_data));
  }

#ifdef COM
#ifdef GD32F1
  COM.begin(COM_SPEED, SERIAL_8N1);
  usart_halfduplex_enable(COM_UART);
  usart_enable(COM_UART);
#endif
#ifdef ESP32
  COM.begin(COM_SPEED, SERIAL_8N1, COM_RXD, COM_TXD);
#endif
#endif

  i2c0.begin();
  as5600.begin();

  // if (!as5600.isConnected()) {
  //   i2c0.end();
  //   SPI.begin();
  //   mt6701.initializeSSI(MT6701_CS_PIN);
  // }

  DEBUG_println(FST("# Init complete\n"));
}


/***********************************************************\
 * Main Loop
\***********************************************************/

void loop() {
  static uint32_t rx_time = 0;
  static uint32_t s_time = 0;

  uint32_t now = millis();

#ifdef COM
  // Handle Dynamixel communication.
  int r = COM.read();
  while (r != -1) {
    uint8_t c = (uint8_t)r;
    // if (now - rx_time > 20) {
    //   DEBUG_println();
    //   DEBUG_printf(FST("%3d: "), now - rx_time);
    // } 
    // DEBUG_printf(FST("%02X "), c);
    parse_rx_data(c);
    rx_time = now;
    r = COM.read();
  }

  if(rx_index && (millis() - last_rx_ts) > 10) { // RX Timeout
    DEBUG_println(FST("# RX Timeout"));
    rx_index = 0;
  }
#endif

  // Handle sensor
  i2c0.run(now);
  if (as5600.run(now)) {
    // New data available.
    int16_t angle = as5600.getAngle();
    if (angle != old_angle) { 
      int pc = reg.position_correction & 0x07FF;
      if (reg.position_correction & 0x800) { pc = -pc; } // Bit 11 is the sign bit.
      angle = (angle - pc) & 0x0FFF;

      if (angle < 1024 &&  old_angle > 1024*3) {rotations++;}
      else if (angle > 1024 * 3 &&  old_angle < 1024) {rotations--;}
      old_angle = angle;

      position = angle + (rotations << 12);

      // reg.current_location = position;
      // if (reg.min_angle != 0 || reg.max_angle != 0) {
      //   if (reg.current_location < reg.min_angle) {reg.current_location = reg.min_angle;}
      //   if (reg.current_location > reg.max_angle) {reg.current_location = reg.max_angle;}
      // }
    }
  }

  static uint32_t ptime = 0;
  if (now > ptime) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    ptime = now + 100; 
    if (!as5600.isConnected()) { ptime += 400; /* slow blink */}

#if SERIAL_DEBUG
    DEBUG_print(FST("Angle:"));
    DEBUG_print(as5600.getAngle());
    DEBUG_print(FST(" Rotations:"));
    DEBUG_print(as5600.getRotations());
    DEBUG_print(FST(" Position:"));
    DEBUG_print(as5600.getPosition());
    DEBUG_print(FST(" Servo:"));
    DEBUG_println(pos);
#endif
  } 
}


