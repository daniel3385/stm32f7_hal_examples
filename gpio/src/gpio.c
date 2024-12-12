#include "stm32f7xx.h"

static void pc13_exti_init(void (*callback)(void));

void (*pin13_callback)(void);

void gpio_init(void (*callback)(void))
{
    pc13_exti_init(callback);
}

static void pc13_exti_init(void (*callback)(void))
{
	/*Enable clock access to PORTC*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	/*Set PC13 to input*/
	GPIOC->MODER &= ~(1U<<26);
	GPIOC->MODER &= ~(1U<<27);

	/*Enable clock access to SYSCFG module*/
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/*Clear port selection for EXTI13*/
	SYSCFG->EXTICR[3] &= ~(1U<<4);
	SYSCFG->EXTICR[3] &= ~(1U<<5);
	SYSCFG->EXTICR[3] &= ~(1U<<6);
	SYSCFG->EXTICR[3] &= ~(1U<<7);

	/*Select PORTC for EXTI13*/
	SYSCFG->EXTICR[3] |= (1U<<5);

	/*Unmask EXTI13*/
	EXTI->IMR |= (1U<<13);

	/*Select falling edge trigger*/
	EXTI->FTSR |= (1U<<13);

    pin13_callback = callback;

	/*Enable EXTI13 in NVIC*/
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
	if((EXTI->PR & (1 << 13)) != 0)
	{
		/*Clear PR flag*/
		EXTI->PR |= (1 << 13);

		pin13_callback();
	}
}