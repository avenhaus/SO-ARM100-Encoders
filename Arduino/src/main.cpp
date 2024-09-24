/*======================================================================*\
 * ESP32-S3 Magnetic Encoder Controller for SO-ARM100 Leader
\*======================================================================*/

/*
 * Read six MT6701 magnetic encoders over SSI (SPI) and send the data to the
 * UART serial port.
 * 
 * The MT6701 has a 14 bit resolution (0-16383).
 * To address six encoders, 10 wires are needed: VCC, GND, Clock, Data, 6 x CS.
 * 
 * https://www.lcsc.com/datasheet/lcsc_datasheet_2110291630_Magn-Tek-MT6701QT-STD_C2913974.pdf
 * 
 * Vcc is 3.3V or 5V, but EEPROM programming needs 5V. ESP32 I/O is not 5V tolerant.
 * 
 */


#include <Arduino.h>
#include "Config.h"
#include <SPI.h>
#include <EEPROM.h>
#include "MT6701.h"

PROGMEM const char EMPTY_STRING[] =  "";
PROGMEM const char NEW_LINE[] =  "\n";

PROGMEM const char PROJECT_NAME[] = __PROJECT_NAME__;
PROGMEM const char PROJECT_VERSION[] = __PROJECT_COMPILE__;
PROGMEM const char COMPILE_DATE[] = __DATE__;
PROGMEM const char COMPILE_TIME[] = __TIME__;

PROGMEM const int ENCODER_PIN[] = {SPI_CS0_PIN, SPI_CS1_PIN, SPI_CS2_PIN, SPI_CS3_PIN, SPI_CS4_PIN, SPI_CS5_PIN};


Print* debugStream = &LOGGER;

MT6701 encoder[ENCODER_COUNT];

typedef struct {
  float angle;
  mt6701_status_t status;
} encoder_data_t;

encoder_data_t data[ENCODER_COUNT] = {0};
float home_angle[ENCODER_COUNT] = {0};

#ifdef CONFIG_IDF_TARGET_ESP32S3
#define HAS_RGB_LED 1
#endif

/***********************************************************\
 * Initialization Code
\***********************************************************/

void setup() {
#ifdef HAS_RGB_LED
  // There is currently a bug in the ESP32-S3 neopixelWrite() code. This will produce the 
  // following errors and the RGB LED will not light up.:
  //  E (19) rmt: rmt_set_gpio(526): RMT GPIO ERROR
  //  E (19) rmt: rmt_config(686): set gpio for RMT driver failed
  //  ==> Remove the #ifdef RGB_BUILTIN logic in the neopixelWrite() code.
  neopixelWrite(RGB_BUILTIN, 0, 10, 10);
#else
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
#endif

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  LOGGER.begin(SERIAL_SPEED);
  delay(1000);
  DEBUG_printf(FST("\n\n# %s %s | %s | %s\n"), PROJECT_NAME, PROJECT_VERSION, COMPILE_DATE, COMPILE_TIME);

  EEPROM.begin(EEPROM_SIZE);
  eeprom_data_t eeprom_data;
  EEPROM.get(0, eeprom_data);
  if (eeprom_data.magic == EEPROM_MAGIC) {
    memcpy(home_angle, eeprom_data.home_angle, sizeof(home_angle));
    DEBUG_print(FST("# Home angles loaded from EEPROM: "));
    for (int i = 0; i < ENCODER_COUNT; i++) {
      DEBUG_printf(FST("%0.2f"), home_angle[i]);
      if (i < ENCODER_COUNT - 1) { DEBUG_print(FST(", ")); }
    }
    DEBUG_println();
#ifdef HAS_RGB_LED
    neopixelWrite(RGB_BUILTIN, 0, 10, 0);
#endif
  } else {
    DEBUG_println(FST("# No valid EEPROM data found"));
#ifdef HAS_RGB_LED
    neopixelWrite(RGB_BUILTIN, 10, 10, 0);
#endif
  }
  EEPROM.end();

  SPI.begin();
  for (int i = 0; i < ENCODER_COUNT; i++) {
    bool ok = encoder[i].initializeSSI(ENCODER_PIN[i]);
    DEBUG_printf(FST("# Init Encoder %d, CS pin %d: %d\n"), i, ENCODER_PIN[i], ok);
  }

  DEBUG_println(FST("# Init complete\n"));
}


/***********************************************************\
 * Main Loop
\***********************************************************/

void loop() {
  static bool last_button = HIGH;
  u32_t start = millis();

  // Read all encoders and check if the data has changed.
  bool change = false;
  for (int i = 0; i < ENCODER_COUNT; i++) {
    mt6701_status_t status = encoder[i].fieldStatusRead();
    float angle = encoder[i].angleRead();
    if (angle != data[i].angle || status != data[i].status) {
      change = true;
      data[i].angle = angle;
      data[i].status = status;
    }
  }

  // Save the home position if the button is pressed.
  bool button = digitalRead(BUTTON_PIN);
  if (button == LOW && last_button == HIGH) {
    for (int i = 0; i < ENCODER_COUNT; i++) {
      home_angle[i] = data[i].angle;
    }
    change = true;
    // Save calibration angles to EEPROM.
    EEPROM.begin(EEPROM_SIZE);
    eeprom_data_t eeprom_data;
    eeprom_data.magic = EEPROM_MAGIC;
    memcpy(eeprom_data.home_angle, home_angle, sizeof(home_angle));
    EEPROM.put(0, eeprom_data);
    EEPROM.end();
    DEBUG_println(FST("# Saved Home Position to EEPROM"));
  }
  last_button = button;

  // Output a record if there was a change.
  if (change || ALWAYS_SEND) {
    DEBUG_printf(FST("%d, "), start);
    for (int i = 0; i < ENCODER_COUNT; i++) {
      float angle = home_angle[i] - data[i].angle;
      if (angle < 0) { angle += 360.0; }
      if (angle > 180.0) { angle -= 360.0; }
      //if (angle < -360.0) { angle += 360.0; }
      //if (angle > 360.0) { angle -= 360.0; }
      DEBUG_printf(FST("%0.2f,%d"), angle , data[i].status);
      if (i < ENCODER_COUNT - 1) { DEBUG_print(FST(", ")); } 
    }

    DEBUG_println();
  }

  // Wait for the next loop.
  delay(LOOP_DELAY - (millis() % LOOP_DELAY));
}



#if 0
//========================= Old I2C code =========================
//========================= Old I2C code =========================
//========================= Old I2C code =========================

void setup() {
#ifdef CONFIG_IDF_TARGET_ESP32S3
  // There is currently a bug in the neopixelWrite() code. This will produce the 
  // following errors and the RGB LED will not light up.:
  //  E (19) rmt: rmt_set_gpio(526): RMT GPIO ERROR
  //  E (19) rmt: rmt_config(686): set gpio for RMT driver failed
  //  ==> Remove the #ifdef RGB_BUILTIN logic in the neopixelWrite() code.
  neopixelWrite(RGB_BUILTIN, 0, 0, 10);
#else
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
#endif

  LOGGER.begin(SERIAL_SPEED);
  delay(1000);
  DEBUG_printf(FST("\n\n%s %s | %s | %s\n"), PROJECT_NAME, PROJECT_VERSION, COMPILE_DATE, COMPILE_TIME);

  Wire.begin(I2C_SDA1_PIN, I2C_SCL_PIN);
  encoder[0].initializeI2C();
  Wire.end();
  Wire.begin(I2C_SDA2_PIN, I2C_SCL_PIN);
  encoder[1].initializeI2C();
  //Wire.end();

}

void printEncoderData(MT6701& encoder){
  float angle = encoder.angleRead();
  mt6701_status_t status = encoder.fieldStatusRead();
  DEBUG_printf(FST("%0.2f, %d | "), angle, status);
}

void loop() {
  Wire.begin(I2C_SDA1_PIN, I2C_SCL_PIN);
  printEncoderData(encoder[0]);
  Wire.end();
  Wire.begin(I2C_SDA2_PIN, I2C_SCL_PIN);
  //delay(10);
  printEncoderData(encoder[1]);
  //delay(10);
  Wire.end();
  DEBUG_println();
  delay(100);
}

#endif