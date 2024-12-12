#include <stdio.h>
#include "stm32f7xx.h"
#include "led.h"
#include "gpio.h"

void toogle_led_green(void);

/*
	Led and PC13 gpio initialization using toogle_led_green as a callback for the button being pushed
 */
int main(void)
{
    user_leds_init();

	gpio_init(toogle_led_green);

	while(1)
	{
	}
}

void toogle_led_green(void)
{
	led_toggle(LED_GREEN);
}








