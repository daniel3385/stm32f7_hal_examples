#ifndef LED_H_
#define LED_H_

#include <stdint.h>

#define USER_LED1 	(1U<<0)  // Green led
#define USER_LED2 	(1U<<7)  //	Blue led
#define USER_LED3 	(1U<<14) //	Red led


#define LED_GREEN	USER_LED1
#define LED_BLUE	USER_LED2
#define LED_RED		USER_LED3


#define GPIOB_CLK_EN		(1U<<1)

typedef uint32_t ledType;

void user_leds_init(void);
void all_leds_on(void);
void all_leds_off(void);
void all_leds_toggle(void);
void led_toggle(ledType led);
void led_on(ledType led);
void led_off(ledType led);


#endif /* LED_H_ */
