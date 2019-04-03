#include "stm32f4xx.h"

/* ------------- How to use the driver ------------------*/
/* ------------------------------------------------------*/
/***** Followwing these steps to config the sensor ******/
/*     Step 1:
					(+)		Initial Srf05 struct variables
					Ex: Srf05 Srf05_Sensor;
					
			 Step 2:
					(+)		Call the Srf05_ParametersInitial() (based on STM32F4xx datasheet)
					Ex: Srf05_ParametersInitial(&Srf05_Sensor,GPIOA,TIM4,GPIO_Pin_0,GPIO_Pin_1)
			 Step 3:
					(+)		Call function Srf05_GetData(&Srf05_Sensor) to get data
*/
/* ------------ Device parameters	--------------------------*/
#define					Srf05_Const											(double)29.412

typedef enum{
	SAI = 0,
	DUNG,
}LOGIC;

typedef struct Srf05{
	GPIO_TypeDef	*GPIOx;
	TIM_TypeDef		*TIMx;
	TIM_TypeDef		*TIMx_Delay;
	uint16_t			Srf05_TriggerInput_Pin;
	uint16_t			Srf05_TriggerOutput_Pin;
	LOGIC					Timeout;
	double				Data_Out;
}Srf05;

extern Srf05 Srf05_Sensor;

/* Export function */
void 			TIM1_UP_TIM10_IRQHandler(void);
void 			TIM16_Delay_US(TIM_TypeDef *TIMx, uint16_t period);
void			Srf05_ParametersInitial(Srf05 *psrf05, GPIO_TypeDef *GPIOx, TIM_TypeDef *TIMx,TIM_TypeDef *TIMx_Delay, uint16_t Srf05_TriggerInput_Pin, uint16_t Srf05_TriggerOutput_Pin);
void			Srf05_GetData(Srf05 *psrf05);



