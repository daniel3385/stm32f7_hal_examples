/*
 * rcc.c
 *
 *  Created on: Jan 6, 2024
 *      Author: Daniel
 */

#include "stm32f7xx.h"
#include "rcc.h"

void set_ahb1_periph_clock(uint32_t periph)
{
	SET_BIT(RCC->AHB1ENR, periph);
}

void set_ahb2_periph_clock(uint32_t periph)
{
	SET_BIT(RCC->AHB2ENR, periph);
}

void set_apb1_periph_clock(uint32_t periph)
{
	SET_BIT(RCC->APB1ENR, periph);
}

void set_apb2_periph_clock(uint32_t periph)
{
	SET_BIT(RCC->APB2ENR, periph);
}
