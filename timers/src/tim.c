#include "stm32f7xx.h"
#include "tim.h"


#define 	TIM1EN		(1U<<0)
#define     CR1_CEN		(1U<<0)
#define		DIER_UIF	(1U<<0)
#define		EGR_UG		(1U<<0)
#define 	SR_UIF		(1U<<0)


void (*timer1_cb)(void);

void tim1_1hz_init(void (*f)(void))
{
	/*Enable clock access to TIM1*/
	RCC->APB2ENR |= TIM1EN;

	/*Set the prescaler*/
	TIM1->PSC = 1600 - 1; // 16 000 000 / 1600 = 10 000

	/*Set auto-reload value*/
	TIM1->ARR = 10000 - 1; //10 000 /10 000 = 1

	/*Enable TIM1 update interrupt */
	TIM1->DIER |= DIER_UIF;

	/*Enable TIM1 update interrupt in NVIC*/
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	/*Assign callback*/
	timer1_cb = f;

	/*Enable timer*/
	TIM1->CR1 |= CR1_CEN;

	/*Force update generation*/
	TIM1->EGR |= EGR_UG;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	/*Check whether update interrupt is pending*/
	if((TIM1->SR & SR_UIF)!=0)
	{
		/*Clear the update interrupt flat*/
		TIM1->SR &=~SR_UIF;

		timer1_cb();
	}

}
