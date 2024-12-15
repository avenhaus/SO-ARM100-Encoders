#pragma once

#include "ch32v003fun.h"

void uart_init(int baud);
void uart_puts(const char *buf);
int uart_write(const char *buf, int size);
int uart_printf( const char* format, ... );
int uart_putchar(int c);
