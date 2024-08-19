#include <stdio.h>
#include "stm32f7xx.h"
#include "uart.h"
#include "led.h"







char ch;

int main(void)
{
    user_leds_init();
	uart3_rxtx_init();

	printf("Ola\r\n");


	while(1)
	{

		ch = uart_read(USART3);

	    switch(ch)
	    {
	    case '0':
	    	all_leds_off();
	    	break;
	    case 'a':
	    	all_leds_on();
	    	break;
	    case 'b':
	    	led_on(BLUE_LED);
	    	break;
	    case 'g':
	    	led_on(GREEN_LED);
	    	break;
	    case 'r':
	    	led_on(RED_LED);
	    	break;
	    default:
	    	break;
	    }

	}


}









