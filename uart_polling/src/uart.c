/*
 * uart.c
 *
 *  Created on: Jan 6, 2024
 *      Author: Daniel
 */

#include <stdint.h>
#include "stm32f7xx.h"
#include "uart.h"
#include "rcc.h"


#define GPIOD_EN		(1U << 3)
#define UART3_TX		(1U << 8)
#define GPIO_ALT_MODE	(0x02)
#define UART3_EN		(1U << 18)

#define UART_DATAWIDTH_8B 	(0x00U)
#define UART_PARITY_NONE 	(0x00U)
#define UART_STOPBIT_1 	(0x00U)

#define SYSTEM_CLOCK 	16000000
#define UART_BAUDRATE	115200

void config_uart_parameters(USART_TypeDef *USARTx, uint32_t data_width, uint32_t parity, uint32_t stop_bits);
uint16_t compute_uart_div(uint32_t periph_clock, uint32_t baudrate);
void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t periph_clock, uint32_t baudrate);
void uart_enable(USART_TypeDef *USARTx);
void set_pin_mode(GPIO_TypeDef *GPIOx, uint32_t pin, uint32_t mode);

void uart_write(USART_TypeDef *USARTx, uint8_t value);

void uart3_tx_init()
{
	// 1. Enable clock for GPIOD
	set_ahb1_periph_clock(GPIOD_EN);
	// 2. Set PD8 to AF mode
	set_pin_mode(GPIOD, UART3_TX, GPIO_ALT_MODE);

	// 3. Set AF to AF7 UART3 see datasheet pag 94
	GPIOD->AFR[1] |= (1U << 0);
	GPIOD->AFR[1] |= (1U << 1);
	GPIOD->AFR[1] |= (1U << 2);
	GPIOD->AFR[1] &= ~(1U << 3);

	// Enable clock to UART
	set_apb1_periph_clock(UART3_EN);

	// Configure UART
	config_uart_parameters(USART3, UART_DATAWIDTH_8B, UART_PARITY_NONE, UART_STOPBIT_1);

	// Set baudrate
	uart_set_baudrate(USART3, SYSTEM_CLOCK, UART_BAUDRATE);

	// Enable TX
	USART3->CR1 |= USART_CR1_TE;

	// Enable UART
	uart_enable(USART3);

}

void uart_write(USART_TypeDef *USARTx, uint8_t value)
{
	// Make sure transmit register is empty
	while(!(USARTx->ISR & USART_ISR_TXE)) {}

	// Write value into transmitt register
	USARTx->TDR = value;
}

void uart_enable(USART_TypeDef *USARTx)
{
	USARTx->CR1 |= USART_CR1_UE;
}

void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t periph_clock, uint32_t baudrate)
{
	USARTx->BRR = compute_uart_div(periph_clock, baudrate);
}

uint16_t compute_uart_div(uint32_t periph_clock, uint32_t baudrate)
{
	return ((periph_clock + baudrate/2U) / baudrate);
}
void config_uart_parameters(USART_TypeDef *USARTx, uint32_t data_width, uint32_t parity, uint32_t stop_bits)
{
	MODIFY_REG(USARTx->CR1, USART_CR1_PS | USART_CR1_PCE | USART_CR1_M, parity | data_width);
	MODIFY_REG(USARTx->CR2, USART_CR2_STOP, stop_bits);
}

void set_pin_mode(GPIO_TypeDef *GPIOx, uint32_t pin, uint32_t mode)
{
	/*
	 * Example: Alternate mode for pin 8:
	 * first steap we clearmask (1U << 16) | (1U << 17)
	 * we can use 0x03 << 16
	 * then we set mode << 16
	 */
	MODIFY_REG(GPIOx->MODER, 0x03 << (POSITION_VAL(pin) * 2U), mode << (POSITION_VAL(pin) * 2U));
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    //__io_putchar(*ptr++);
	uart_write(USART3, *ptr++);
  }
  return len;
}

int __io_putchar(int ch)
{
	uart_write(USART3, ch);
	return ch;
}
