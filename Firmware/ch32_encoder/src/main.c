/* Small example showing how to use the SWIO programming pin to 
   do printf through the debug interface */

// ~\.platformio\packages\tool-minichlink\minichlink.exe -bT

#include "ch32v003fun.h"
#include <stdio.h>

#include "config.h"
#include "systime.h"
#include "uart.h"
#include "i2c.h"
#include "AS5600_NB.h"

as5600_t as5600;

int main()
{
	SystemInit();

	// Enable GPIOs
	funGpioInitAll();
	funPinMode(LED_PIN, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
	funDigitalWrite(LED_PIN, FUN_LOW);

	systime_init();

#if FUNCONF_USE_DEBUGPRINTF
	uart_init(115200); 
#endif

#if DEBUG
	while( !DebugPrintfBufferFree() );
	printf( "CH32 Encoder\n");
#endif

	i2c_init(1000000);
	as5600_init(&as5600, AS5600_SW_DIRECTION_PIN, AS5600_DEFAULT_ADDRESS);


	uint32_t next = millis();
	while(1)
	{
		i2c_run();
		as5600_run(&as5600);
		if (millis() > next)
		{
			next += 500;
			funDigitalWrite(LED_PIN, !funDigitalRead(LED_PIN));
			uint16_t angle = as5600_get_angle(&as5600);
			uint8_t* ptr = (uint8_t*) &angle;
			printf("Hello World %ld Ang:%d %02X %02X Pos:%ld St:%d\n", millis(), angle, ptr[0], ptr[1], as5600_get_position(&as5600), as5600.state);
		}
	}
}

