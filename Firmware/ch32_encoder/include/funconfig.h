#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

#define CH32V003           1

#define FUNCONF_SYSTICK_USE_HCLK 1

#if 0
#define FUNCONF_USE_DEBUGPRINTF 1
#else
#define FUNCONF_USE_DEBUGPRINTF 0
#define FUNCONF_USE_UARTPRINTF  1
#define FUNCONF_UART_PRINTF_BAUD 115200
//#define FUNCONF_DEBUGPRINTF_TIMEOUT (1<<31) // Optionally, wait for a very very long time on every printf.
#endif

#endif

