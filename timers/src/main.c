#include <stdio.h>
#include "stm32f7xx.h"
#include "tim.h"
#include "led.h"
#include "systick.h"

void toogle_led_green(void);

/*
	Board leds initialization
	Initialize timer 1 for 1 second period and using callback function toggle led green
	Initialize systick and each 2 seconds toggle led blue
 */
int main(void)
{
    user_leds_init();

	tim1_1hz_init(&toogle_led_green);

	while(1)
	{
		systickDelayMs(2000);
		led_toggle(LED_BLUE);
	}
}

void toogle_led_green(void)
{
	led_toggle(LED_GREEN);
}








