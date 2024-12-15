#pragma once

#include "stdint.h"

// Number of ticks elapsed per millisecond (48,000 when using 48MHz Clock)
#define SYSTICK_ONE_MILLISECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000)
// Number of ticks elapsed per microsecond (48 when using 48MHz Clock)
#define SYSTICK_ONE_MICROSECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000000)

// Simple macro functions to give a arduino-like functions to call
// millis() reads the incremented systick variable
// micros() reads the raw SysTick Count, and divides it by the number of 
// ticks per microsecond ( WARN: Wraps every 90 seconds!)
#define millis() (systick_millis)
#define micros() (SysTick->CNT / SYSTICK_ONE_MICROSECOND)

// Incremented in the SysTick IRQ - in this example once per millisecond
extern volatile uint32_t systick_millis;

void systime_init(void);
