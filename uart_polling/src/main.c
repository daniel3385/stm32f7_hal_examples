#include <stdio.h>
#include "stm32f7xx.h"
#include "uart.h"
#include "led.h"

/* static functions prototype */
static void helper_func(void);

char ch;

/*
	Board leds initialization
	UART3 is initalizaded and used for printf
	printf helper function to user learn how to control the leds using
	keep pooling UART and control user led based on user choice
 */
int main(void)
{
    user_leds_init();
	uart3_rxtx_init();

	helper_func();

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
			helper_func();
	    	break;
	    }
	}
}

/*
	Print in the UART the options available for user
 */
static void helper_func(void)
{
	printf("Select:\r\n"
	"0 - all leds OFF\r\n"
	"a - all leds ON\r\n"
	"b - led blue ON\r\n"
	"g - led green ON\r\n"
	"r - led red ON\r\n"
	);
}









