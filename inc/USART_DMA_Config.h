#include "stm32f4xx.h"
/* Define */
#define  						TxSize												500
#define							RxSize												500
/*-------- Hardware config USART1 (can change) ---------*/
#define 						U1_GPIOx													GPIOA
#define             U1_GPIO_Pin_Tx        						GPIO_Pin_9
#define             U1_GPIO_Pin_Rx										GPIO_Pin_10
#define							U1_GPIO_PinSourceTx								GPIO_PinSource9
#define							U1_GPIO_PinSourceRx								GPIO_PinSource10
#define   					U1_RCC_AHB1Periph_GPIOx						RCC_AHB1Periph_GPIOA
/*-------- Hardware config USART2 (can change) ---------*/
//#define 					U2_GPIOx													GPIOD
//#define           U2_GPIO_Pin_Tx        						GPIO_Pin_5
//#define           U2_GPIO_Pin_Rx										GPIO_Pin_6
//#define						U2_GPIO_PinSourceTx								GPIO_PinSource5
//#define						U2_GPIO_PinSourceRx								GPIO_PinSource6
//#define   				U2_RCC_AHB1Periph_GPIOx						RCC_AHB1Periph_GPIOD
/*-------- Hardware config USART2 Rx -------------------*/
#define							U2Rx_GPIOx												GPIOD
#define							U2Rx_GPIO_Pin_Rx									GPIO_Pin_6
#define							U2Rx_GPIO_PinSourceRx							GPIO_PinSource6
#define							U2Rx_RCC_AHB1Periph_GPIOx					RCC_AHB1Periph_GPIOD
/*-------- Hardware config USART6 (can change) ---------*/
#define 						U6_GPIOx													GPIOC
#define             U6_GPIO_Pin_Tx        						GPIO_Pin_6
#define             U6_GPIO_Pin_Rx										GPIO_Pin_7
#define							U6_GPIO_PinSourceTx								GPIO_PinSource6
#define							U6_GPIO_PinSourceRx								GPIO_PinSource7
#define   					U6_RCC_AHB1Periph_GPIOx						RCC_AHB1Periph_GPIOC
/* Types */

/* Export variables */
extern    uint8_t 						U1_TxBuffer[TxSize], U1_RxBuffer[RxSize];
extern		uint8_t							U2_TxBuffer[TxSize], U2_RxBuffer[RxSize];
extern    uint8_t 						U6_TxBuffer[TxSize], U6_RxBuffer[RxSize];
/* Export Function */
void 						USART1_Config(uint32_t  BaudRate);
void					  USART2_Config(uint32_t  BaudRate);
void						USART2_Rx_Config(uint32_t BaudRate);
void 						USART6_Config(uint32_t  BaudRate);
void 						U1_SendData(uint16_t NbOfByte);
void 						U2_SendData(uint16_t NbOfByte);
void 						U6_SendData(uint16_t NbOfByte);
unsigned int	 	ToStringData(char string[], uint8_t *pBuffer);
unsigned int 		SendNewLine(uint8_t *pBuffer);

















































