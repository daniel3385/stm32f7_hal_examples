#include <stdint.h>
#include <stdio.h>
#include "stm32f7xx.h"
#include "uart.h"




int main()
{
	uart3_tx_init();

	while(1)
	{
		printf("Hello from stm32f7\r\n");

		// Delay
		for(int i=0; i<1000000; i++) {}
	}
}










