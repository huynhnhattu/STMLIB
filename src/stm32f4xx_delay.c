#include "stm32f4xx_delay.h"

void TIM_Delay_US(uint16_t period)
{
		/** @Enable @RCC @Clock **/
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
		TIM4->PSC = 49;    // STM32F411 has 16 bits TIM4 50MHz APB1 bus 50*10^6 / 50 = 1MHz => 1us
		TIM4->ARR = period - 1;   // agr 16 bits unsigned int
		TIM4->CNT = 0;
		TIM4->EGR = 1;		//update register;
		
		TIM4->SR = 0;			//clear overflow flag
		TIM4->CR1 = 1;    //Enable Timer
		while(!TIM4->SR);
		TIM4->CR1 = 0;		//Stop Timer
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,DISABLE);
}

void TIM_Delay_MS(uint16_t period)
{
		/** @Enable @RCC @Clock **/
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
		TIM4->PSC = 4999;    // STM32F411 has 16 bits TIM4 50MHz APB1 bus 50*10^6 / 50 = 1MHz => 1us
		TIM4->ARR = period*20 - 1;   // agr 16 bits unsigned int
		TIM4->CNT = 0;
		TIM4->EGR = 1;		//update register;
		
		TIM4->SR = 0;			//clear overflow flag
		TIM4->CR1 = 1;    //Enable Timer
		while(!TIM4->SR);
		TIM4->CR1 = 0;		//Stop Timer
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,DISABLE);
}

void Core_Delay(uint32_t count)
{
		while(count--);
}
	
	
	

	
	
	
	

















