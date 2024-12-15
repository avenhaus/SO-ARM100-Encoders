#include "ch32v003fun.h"
#include <stdio.h>
#include <stdarg.h>

#include "uart.h"

void uart_init(int baud) {
#ifdef CH32V003
	// Enable GPIOD and UART.
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1;

	// Push-Pull, 10MHz Output, GPIO D5, with AutoFunction
	GPIOD->CFGLR &= ~(0xf<<(4*5));
	GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*5);
#elif defined(CH32X03x)
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOB | RCC_APB2Periph_USART1;

	// Push-Pull, 10MHz Output, GPIO A9, with AutoFunction
	GPIOB->CFGHR &= ~(0xf<<(4*2));
	GPIOB->CFGHR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*2);
#else
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1;

	// Push-Pull, 10MHz Output, GPIO A9, with AutoFunction
	GPIOA->CFGHR &= ~(0xf<<(4*1));
	GPIOA->CFGHR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*1);
#endif

	// 115200, 8n1.  Note if you don't specify a mode, UART remains off even when UE_Set.
	USART1->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Tx | USART_Mode_Rx;
	USART1->CTLR2 = USART_StopBits_1;
	USART1->CTLR3 = USART_HardwareFlowControl_None;

	USART1->BRR = ((FUNCONF_SYSTEM_CORE_CLOCK) + ((baud)>>1)) / baud;
	USART1->CTLR1 |= CTLR1_UE_Set;
}

void uart_puts(const char *buf)
{
	while(*buf) {
	    while( !(USART1->STATR & USART_FLAG_TC));
	    USART1->DATAR = *buf++;
	}
}

int uart_write(const char *buf, int size)
{
	for(int i = 0; i < size; i++){
	    while( !(USART1->STATR & USART_FLAG_TC));
	    USART1->DATAR = *buf++;
	}
	return size;
}

int uart_putchar(int c)
{
	while( !(USART1->STATR & USART_FLAG_TC));
	USART1->DATAR = (const char)c;
	return 1;
}

int uart_avail() {
    return (USART1->STATR & USART_STATR_RXNE) != 0;
}

uint8_t uart_get() {
    // wait for a character to be received
    while (!(USART1->STATR & USART_STATR_RXNE)) { }
    return USART1->DATAR;
}

static int __uart_puts( char *s, int len, void *buf ) {
	(void)buf;
	uart_write(s, len);
	return len;
}

int mini_vsnprintf( char *buffer, unsigned int buffer_len, const char *fmt, va_list va );
int mini_vpprintf( int (*puts)(char* s, int len, void* buf), void* buf, const char *fmt, va_list va );

int uart_printf( const char* format, ... ) {
	va_list args;
	va_start( args, format );
	int ret_status = mini_vpprintf(__uart_puts, 0, format, args);
	va_end( args );
	return ret_status;
}

