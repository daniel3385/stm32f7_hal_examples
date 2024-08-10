/*
 * rcc.h
 *
 *  Created on: Jan 6, 2024
 *      Author: Daniel
 */

#ifndef RCC_H_
#define RCC_H_

#include <stdint.h>

void set_ahb1_periph_clock(uint32_t periph);
void set_apb1_periph_clock(uint32_t periph);

#endif /* RCC_H_ */
