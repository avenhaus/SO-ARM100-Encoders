/*
https://emanual.robotis.com/docs/en/dxl/protocol1/
https://www.theremino.com/wp-content/uploads/files/SmartMotors/Feetech_Motors_Programming_ENG.pdf

FeeTech Documentation:
https://www.feetechrc.com/letter-of-agreement.html
https://www.feetechrc.com/Data/feetechrc/upload/file/20240702/%E8%88%B5%E6%9C%BA%E5%8D%8F%E8%AE%AE%E6%89%8B%E5%86%8C-%E7%A3%81%E7%BC%96%E7%A0%81%E7%89%88%E6%9C%AC.pdf

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


 STS3215 Position Offset Experiments
======================================
// Position 1310 , Offset 300 => 1010
// Position 1310 , Offset -300 => 1010  !! UI Bug with bit 11 as sign??
// Position 1310 , Offset 300 + 2048 => UI Error: cannot set more than 2048 (UI bug?)
// Position 1311 , Torque Switch 128 => Offset: 2785 (2785 is -737) => 2048 (This value cannot be set from UI)
// Position 2502 , Torque Switch 128 => Offset: 454 => 2048
// Position limits are ignored when moving the motor manually. No negative positions. Wrap at 4095.
*/

#include <Arduino.h>

#include "Config.h"
#ifdef GD32F1
// Somehow EEPROM.h does not find this if not included first.
#include <FlashStorage.h>
#endif
#include <EEPROM.h>

// #include <SPI.h>
// #include "MT6701.h"

#include "i2c.h"
#include "AS5600_NB.h"

#define DXL_BROADCAST_ID 0xFE

#define DXL_PING_CMD 0x01
#define DXL_READ_CMD 0x02
#define DXL_WRITE_CMD 0x03
#define DXL_REG_WRITE_CMD 0x04
#define DXL_ACTION_CMD 0x05
#define DXL_FACTORY_RESET_CMD 0x06
#define DXL_REBOOT_CMD 0x08
#define DXL_SYNC_READ_CMD 0x82
#define DXL_SYNC_WRITE_CMD 0x83
#define DXL_BULK_READ_CMD 0x92


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
  DEFAULT_ID,     // id
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

uint8_t sb_read_after_id = 0;
uint8_t sb_read_addr = 0;
uint8_t sb_read_len = 0;
uint32_t sb_read_timeout = 0;

// MT6701 mt6701;

I2C i2c0(0, SDA, SCL, 1000000);
AS5600_NB as5600(i2c0, AS5600_DIR_PIN);  

int16_t angle = 0;
int16_t old_angle = 0;
int32_t rotations = 0;
int32_t position = 0;
uint32_t last_data_ts = 0;

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
#ifndef ARDUINO_GENERIC_F130F4PX
  EEPROM.begin();
  EEPROM.put(0, eeprom_data);
  EEPROM.commit();
#endif
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
 * Ping
\***********************************************************/
void ping_cmd() {
  uint8_t header[] = {0xFF, 0xFF, reg.id, 2, error};

  // Calculate the CRC.
  uint8_t crc = 0;
  for (int i = PACKET_ID; i < sizeof(header); i++) {crc += header[i]; }
  crc = ~crc; // Invert the bits.

#ifdef COM
  // Send the response.
  COM.write(header, sizeof(header));
  COM.write(crc);
  ignore = sizeof(header) + 1;
#endif

#if SERIAL_DEBUG and 0
  debug_hex(header, sizeof(header));
  debug_hex(reg_data+addr, len);
  DEBUG_printf(FST("%02X\n"), crc);
#endif
}



/***********************************************************\
 * Write to registers
\***********************************************************/
void write_cmd(uint8_t addr, uint8_t len, const uint8_t* data) {
  uint8_t header[] = {0xFF, 0xFF, reg.id, 2, error};

  // Save the data to the registers.
  for (size_t i = 0; i < len; i++) {
    if (addr + i == 40 && data[i] == 128) {
      // Set Center Position
      int16_t pc = angle -2048;
      if (pc < 0) { pc = -pc; pc |= 0x800; }
      reg.position_correction = pc;
      rotations = 0;
    } else {
        reg_data[addr+i] = data[i];
    }
  }

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
 * Sync Read from registers
\***********************************************************/
// TODO: Buffer data so that data from all servos is read at the exact same time?
void sync_read_cmd(uint8_t* packet) {
  uint8_t n = rx_buffer[PACKET_LENGTH] - 4;
  uint8_t* ptr = packet + 7;

  // Check if own ID is first.
  if (ptr[0] == reg.id) {
    DEBUG_println(FST("# Sync Read: Own ID is first."));
    return read_cmd(rx_buffer[PACKET_ADDR], rx_buffer[PACKET_LEN]);
  }

  // Check if own ID is in the rest of the list.
  uint8_t after_id = ptr[0];
  for (uint8_t i = 1; i < n; i++) {
    if (ptr[i] == reg.id) {
      // Wait until after servo with ID after_id responds.
      sb_read_after_id = after_id;
      sb_read_addr = rx_buffer[PACKET_ADDR];
      sb_read_len = rx_buffer[PACKET_LEN];
      sb_read_timeout = millis() + 1000;
      DEBUG_print(FST("# Sync Read: Own ID after: "));
      DEBUG_println(after_id);
      return;
    }
    after_id = ptr[i];
  }
}


/***********************************************************\
 * Bulk Read from registers
\***********************************************************/
void bulk_read_cmd(uint8_t* packet) {
  uint8_t n = rx_buffer[PACKET_LENGTH] - 3;
  uint8_t* ptr = packet + 6;

  // Check if own ID is first.
  if (ptr[1] == reg.id) {
    DEBUG_println(FST("# Bulk Read: Own ID is first."));
    return read_cmd(ptr[2], ptr[0]);
  }

  // Check if own ID is in the rest of the list.
  uint8_t after_id = ptr[1];
  for (uint8_t i = 3; i < n; i+=3) {
    if (ptr[i+1] == reg.id) {
      // Wait until after servo with ID after_id responds.
      sb_read_after_id = after_id;
      sb_read_addr = ptr[i+2];
      sb_read_len = ptr[i];
      sb_read_timeout = millis() + 1000;
      DEBUG_print(FST("# Bulk Read: Own ID after: "));
      DEBUG_println(after_id);
      return;
    }
    after_id = ptr[i+1];
  }

  DEBUG_println(FST("# Bulk Read: Own ID not found."));
}

/***********************************************************\
 * Execute a command
\***********************************************************/
void execute_command() {
  uint8_t instruction = rx_buffer[PACKET_INSTRUCTION];
  switch(instruction) {
    case DXL_PING_CMD: // Ping
      DEBUG_print(FST("# Ping\n"));
      ping_cmd();
      break;

    case DXL_READ_CMD: // Read
      {
        uint8_t addr = rx_buffer[PACKET_ADDR]; 
        uint8_t len = rx_buffer[PACKET_LEN];
        // DEBUG_printf(FST("# Read Adr:%d Len:%d => "), addr, len);
        read_cmd(addr, len);
      }
      break;

    case DXL_WRITE_CMD: // Write
      {
        uint8_t addr = rx_buffer[PACKET_ADDR]; 
        uint8_t len = rx_buffer[PACKET_LENGTH]-3;
        DEBUG_print(FST("# Write Adr:"));
        DEBUG_print(addr);
        DEBUG_print(FST("Len:"));
        DEBUG_println(len);
        bool save = false;
        if (addr == 55 && rx_buffer[PACKET_DATA] == 1 && reg.lock_flag == 0) { save = true; }
        write_cmd(addr, len, rx_buffer + PACKET_DATA);
        if (save) {save_eeprom();}
      }
      break;

    case DXL_REG_WRITE_CMD: // Reg Write
      DEBUG_print(FST("# Reg Write\n"));
      break;

    case DXL_ACTION_CMD: // Action
      DEBUG_print(FST("# Action\n"));
      break;

    case DXL_FACTORY_RESET_CMD: // Factory Reset
      DEBUG_print(FST("# Factory Reset\n"));
      break;

    case DXL_REBOOT_CMD: // Reboot
      DEBUG_print(FST("# Reboot\n"));
      ping_cmd();
#ifdef COM
      COM.flush();
      delay(42);
#endif
      { int a = 0; int b = 42 / a; }  // Crash
      break;

    case DXL_SYNC_READ_CMD: // Sync Read
      DEBUG_print(FST("# Sync Read\n"));
      sync_read_cmd(rx_buffer);
      break;

    case DXL_SYNC_WRITE_CMD: // Sync Write
      DEBUG_print(FST("# Sync Write\n"));
      break;

    case DXL_BULK_READ_CMD: // Bulk Read
      DEBUG_print(FST("# Bulk Read\n"));
      bulk_read_cmd(rx_buffer);
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

  if (rx_index > 5 && rx_buffer[PACKET_LENGTH]+4 == rx_index) { // Full packet received.
    rx_index = 0;
    if(rx_buffer[PACKET_ID] != reg.id 
        && rx_buffer[PACKET_ID] != DXL_BROADCAST_ID 
        && sb_read_after_id == 0) { 
      // DEBUG_printf(FST("\n# Wrong ID: %02X, Length: %02X, Instruction: %02X\n"), rx_buffer[PACKET_ID], rx_buffer[PACKET_LENGTH], rx_buffer[PACKET_INSTRUCTION]);
      return;
    }

    // Checksum calculation: Checksum = ~( ID + Length + Instruction + Parameter1 + … Parameter N )
    uint8_t crc = 0;
    for (int i = PACKET_ID; i < rx_buffer[PACKET_LENGTH]+3; i++) {
      crc += rx_buffer[i];
    }
    crc = ~crc; // Invert the bits.

    if (crc != rx_buffer[rx_buffer[PACKET_LENGTH]+3]) {
      DEBUG_print(FST("\n# Bad CRC:"));
      DEBUG_print(crc);
      DEBUG_print(FST(", received: ")); 
      DEBUG_println(rx_buffer[rx_buffer[PACKET_LENGTH]+3]);
      return;
    }

    if (rx_buffer[PACKET_ID] == DXL_BROADCAST_ID 
          && rx_buffer[PACKET_INSTRUCTION] != DXL_SYNC_READ_CMD 
          && rx_buffer[PACKET_INSTRUCTION] != DXL_BULK_READ_CMD) {
      DEBUG_print(FST("\n# Unsupported broadcast CMD: "));
      DEBUG_println(rx_buffer[PACKET_INSTRUCTION], HEX);
      return;
    }

    if (sb_read_after_id) {
      DEBUG_print(FST("\n# Sync/Bulk Read got response from ID: "));
      DEBUG_println(rx_buffer[PACKET_ID]);
      if (rx_buffer[PACKET_ID] == sb_read_after_id) {
        DEBUG_println(FST("\n# Sync/Bulk Read got response from After ID"));
        read_cmd(sb_read_addr, sb_read_len);
        sb_read_after_id = 0;
        sb_read_addr = 0;
        sb_read_len = 0;
        sb_read_timeout = 0;
      }
      return;
    }

    execute_command();
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

#ifdef ARDUINO_GENERIC_F130F4PX
    memcpy(&eeprom_data.registers, &default_data, sizeof(default_data));
#else

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
#endif

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

  if (sb_read_after_id && now > sb_read_timeout) {
    DEBUG_print(FST("# Sync/Bulk Read: Timeout. After ID: "));
    DEBUG_println(sb_read_after_id);
    sb_read_after_id = 0;
  }
#endif

  // Handle sensor
  i2c0.run(now);
  if (as5600.run(now)) {
    // New data available.
    last_data_ts = now;
    angle = as5600.getAngle();
    if (angle != old_angle) { 
      if (angle < 1024 &&  old_angle > 1024*3) {rotations++;}
      else if (angle > 1024 * 3 &&  old_angle < 1024) {rotations--;}
      old_angle = angle;
    }

    int pc = reg.position_correction & 0x07FF;
    if (reg.position_correction & 0x800) { pc = -pc; } // Bit 11 is the sign bit.

    position = (int32_t)((angle - pc) & 0x0FFF) + (rotations << 12);

    if (reg.max_angle == 0 && reg.min_angle == 0) {
      // Multi Rotations Enabled
      if (position < 0) {position = (-position) | 0x8000;}
    } else {
      position &= 0x0FFF;
      // if (position < reg.min_angle) {position = reg.min_angle;}
      // if (position > reg.max_angle) {position = reg.max_angle;}
    }
    reg.current_location = position;
  }

  static uint32_t ptime = 0;
  if (now > ptime) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    ptime = now + 100; 
    if (!as5600.isConnected() || (now > last_data_ts+100) ) { ptime += 900; /* slow blink */}

#if SERIAL_DEBUG
    DEBUG_print(FST("Angle:"));
    DEBUG_print(as5600.getAngle());
    DEBUG_print(FST(" Rotations:"));
    DEBUG_print(as5600.getRotations());
    DEBUG_print(FST(" Position:"));
    DEBUG_print(as5600.getPosition());
    DEBUG_print(FST(" Servo:"));
    DEBUG_println(reg.current_location);
#endif
  } 
}


