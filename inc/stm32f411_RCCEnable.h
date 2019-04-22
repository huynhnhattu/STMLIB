#include "stm32f4xx.h"

/* Export functions */
/* GPIO Config section */
void 				RCC_GPIOxClockCmd(GPIO_TypeDef *GPIOx, FunctionalState NewState);
uint8_t 		GetPinSourceFromGPIOPin(uint16_t GPIO_Pin);
/* TIM Config section */
void 				RCC_TIMxClockCmd(TIM_TypeDef *TIMx, FunctionalState NewState);
uint8_t			GetAFFromTIM(TIM_TypeDef *TIMx);
IRQn_Type		GetIRQHandlerFromTIMxCC(TIM_TypeDef *TIMx);
/* DMA Config section */
void 				RCC_DMAxClockCmd(DMA_TypeDef *DMAx, FunctionalState NewState);
/* USART Config section */
void		  	RCC_USARTxClockCmd(USART_TypeDef *USARTx, FunctionalState NewState);
/* I2C Config section */
void 				RCC_I2CxClockCmd(I2C_TypeDef *I2Cx, FunctionalState NewState);






















