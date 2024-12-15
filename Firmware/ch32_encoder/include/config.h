#pragma once

#define SWIO_PIN PD1 // Program / Debug

#define LED_PIN PD3
#define TXD_PIN PD5
#define RXD_PIN PD6

#define SDA_PIN PC1
#define SCL_PIN PC2

#define OUT_PIN PC3
#define PUSH_PIN PC4

#define SS_PIN PC0
#define SCK_PIN PC5
#define MOSI_PIN PC6
#define MISO_PIN PC7

#define DEBUG 1

//#define millis() (SysTick->CNT / DELAY_MS_TIME)