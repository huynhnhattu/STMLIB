#include "Srf05.h"

GPIO_InitTypeDef						Srf05_GPIO_Struct;
TIM_TimeBaseInitTypeDef			Srf05_TIM_TimeBaseStruct;
TIM_ICInitTypeDef						Srf05_TIM_ICStruct;
NVIC_InitTypeDef						Srf05_NVIC_Struct;
uint16_t counter = 0;
Srf05												Srf05_Sensor;
/** @Brief: Enable TIMx peripheral clock
**	@Args : TIMx
**	@Ret	: None
**/
void EnableTIMPeriphClock(TIM_TypeDef *Srf05_TIMx)
{
		if(Srf05_TIMx == TIM1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	else if(Srf05_TIMx == TIM2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	else if(Srf05_TIMx == TIM3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	else if(Srf05_TIMx == TIM4)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	else if(Srf05_TIMx == TIM5)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	else if(Srf05_TIMx == TIM6)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	else if(Srf05_TIMx == TIM7)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	else if(Srf05_TIMx == TIM8)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	else if(Srf05_TIMx == TIM9)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
	else if(Srf05_TIMx == TIM10)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);
	else if(Srf05_TIMx == TIM11)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);
	else if(Srf05_TIMx == TIM12)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);
	else if(Srf05_TIMx == TIM13)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13,ENABLE);
}

/** @Brief: Disable TIMx peripheral clock
**	@Args : TIMx
**	@Ret	: None
**/
void	DisableTIMPeriphClock(TIM_TypeDef *Srf05_TIMx)
{
		if(Srf05_TIMx == TIM1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,DISABLE);
	else if(Srf05_TIMx == TIM2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,DISABLE);
	else if(Srf05_TIMx == TIM3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,DISABLE);
	else if(Srf05_TIMx == TIM4)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,DISABLE);
	else if(Srf05_TIMx == TIM5)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,DISABLE);
	else if(Srf05_TIMx == TIM6)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,DISABLE);
	else if(Srf05_TIMx == TIM7)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,DISABLE);
	else if(Srf05_TIMx == TIM8)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,DISABLE);
	else if(Srf05_TIMx == TIM9)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,DISABLE);
	else if(Srf05_TIMx == TIM10)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,DISABLE);
	else if(Srf05_TIMx == TIM11)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,DISABLE);
	else if(Srf05_TIMx == TIM12)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,DISABLE);
	else if(Srf05_TIMx == TIM13)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13,DISABLE);
}

/** @Brief: Enable GPIOx peripheral clock
**	@Args : GPIOx
**	@Ret	: None
**/
void	EnableGPIOxClock(GPIO_TypeDef *GPIOx)
{
	if(GPIOx == GPIOA)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	else if(GPIOx == GPIOB)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	else if(GPIOx == GPIOC)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	else if(GPIOx == GPIOD)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	else if(GPIOx == GPIOE)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	else if(GPIOx == GPIOF)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	else if(GPIOx == GPIOG)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	else if(GPIOx == GPIOH)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	else if(GPIOx == GPIOI)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	else if(GPIOx == GPIOJ)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOJ,ENABLE);
}

/** @Brief: TIMER 16 bits delay in us
**	@Args : TIMx and period
**	@Ret	: None
**/
void	TIM16_Delay_US(TIM_TypeDef *TIMx, uint16_t period)
{
		/** @Enable @RCC @Clock **/
		EnableTIMPeriphClock(TIMx);
		TIMx->PSC = 49;    // STM32F411 has 16 bits TIM4 50MHz APB1 bus 50*10^6 / 50 = 1MHz => 1us
		TIMx->ARR = period - 1;   // agr 16 bits unsigned int
		TIMx->CNT = 0;
		TIMx->EGR = 1;		//update register;
		
		TIMx->SR = 0;			//clear overflow flag
		TIMx->CR1 = 1;    //Enable Timer
		while(!TIMx->SR);
		TIMx->CR1 = 0;		//Stop Timer
		DisableTIMPeriphClock(TIMx);
}

/** @Brief: Parameters init for sensor
**	@Args : Sensor variables structure, GPIOx, TIMx, Trigger input pin to start the sensor and Output pin trigger of sensor
**	@Ret	: None
**/
void	Srf05_ParametersInitial(Srf05 *psrf05, GPIO_TypeDef *GPIOx, TIM_TypeDef *TIMx, TIM_TypeDef *TIMx_Delay, uint16_t Srf05_TriggerInput_Pin, uint16_t Srf05_TriggerOutput_Pin)
{
	psrf05->GPIOx											= GPIOx;
	psrf05->TIMx											= TIMx;
	psrf05->Srf05_TriggerInput_Pin 		= Srf05_TriggerInput_Pin;
	psrf05->Srf05_TriggerOutput_Pin		= Srf05_TriggerOutput_Pin;
	psrf05->TIMx_Delay								= TIMx_Delay;
	psrf05->Timeout										= SAI;
	EnableGPIOxClock(GPIOx);
	/* Config GPIO pin */
	Srf05_GPIO_Struct.GPIO_Mode									= GPIO_Mode_AF;
	Srf05_GPIO_Struct.GPIO_Pin									= Srf05_TriggerOutput_Pin;
	Srf05_GPIO_Struct.GPIO_Speed								= GPIO_Speed_50MHz;
	GPIO_Init(GPIOx,&Srf05_GPIO_Struct);
	
	GPIO_PinAFConfig(psrf05->GPIOx,GPIO_PinSource8,GPIO_AF_TIM10);
	
	Srf05_GPIO_Struct.GPIO_Mode									= GPIO_Mode_OUT;
	Srf05_GPIO_Struct.GPIO_Pin									= Srf05_TriggerInput_Pin;
	Srf05_GPIO_Struct.GPIO_OType								= GPIO_OType_PP;
	Srf05_GPIO_Struct.GPIO_PuPd									= GPIO_PuPd_NOPULL;
	Srf05_GPIO_Struct.GPIO_Speed								= GPIO_Speed_50MHz;
	GPIO_Init(GPIOx,&Srf05_GPIO_Struct);
	
	EnableTIMPeriphClock(TIMx);
	/* Config time base struct */
	Srf05_TIM_TimeBaseStruct.TIM_Prescaler			= (SystemCoreClock / 1000000) - 1; //us
	Srf05_TIM_TimeBaseStruct.TIM_Period					= 0xFFFF;
	Srf05_TIM_TimeBaseStruct.TIM_ClockDivision 	= TIM_CKD_DIV1;
	Srf05_TIM_TimeBaseStruct.TIM_CounterMode 	  = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMx,&Srf05_TIM_TimeBaseStruct);
	Srf05_TIM_ICStruct.TIM_Channel							= TIM_Channel_1;
	Srf05_TIM_ICStruct.TIM_ICPolarity						= TIM_ICPolarity_BothEdge;
	Srf05_TIM_ICStruct.TIM_ICPrescaler					= TIM_ICPSC_DIV1;
	Srf05_TIM_ICStruct.TIM_ICSelection					= TIM_ICSelection_DirectTI;
	TIM_ICInit(psrf05->TIMx,&Srf05_TIM_ICStruct);
	
	Srf05_NVIC_Struct.NVIC_IRQChannel						= TIM1_UP_TIM10_IRQn;
	Srf05_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
	Srf05_NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
	Srf05_NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&Srf05_NVIC_Struct);
	
	TIM_ITConfig(TIMx,TIM_IT_CC1,ENABLE);
	TIM_Cmd(TIMx,ENABLE);
}

/** @Brief: Get data function
**	@Args : srf05
**	@Ret	: None
**/
void	Srf05_GetData(Srf05 *psrf05)
{
	GPIO_ResetBits(psrf05->GPIOx,psrf05->Srf05_TriggerInput_Pin);
	TIM16_Delay_US(psrf05->TIMx_Delay,5);
	GPIO_SetBits(psrf05->GPIOx,psrf05->Srf05_TriggerInput_Pin);
	TIM16_Delay_US(psrf05->TIMx_Delay,10);
	GPIO_ResetBits(psrf05->GPIOx,psrf05->Srf05_TriggerInput_Pin);
	for(uint32_t timeout = 0; timeout < 1000000; timeout++)
	{
		if(psrf05->Timeout == DUNG)
		{
			psrf05->Timeout = SAI;
			psrf05->Data_Out = ((double)((double)counter / Srf05_Const))/2;
			break;
		}
	}
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	TIM_ClearITPendingBit(Srf05_Sensor.TIMx,TIM_IT_CC1);
	if(GPIO_ReadInputDataBit(Srf05_Sensor.GPIOx,Srf05_Sensor.Srf05_TriggerOutput_Pin))
	{
		TIM_SetCounter(Srf05_Sensor.TIMx,0);
	}
	else
	{
		counter = TIM_GetCapture1(Srf05_Sensor.TIMx);
		Srf05_Sensor.Timeout  = DUNG;
	}
}


