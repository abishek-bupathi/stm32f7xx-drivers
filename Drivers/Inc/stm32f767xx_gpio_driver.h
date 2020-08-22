/*
 * stm32f767xx_gpio_driver.h
 *
 *  Created on: 21 Aug 2020
 *      Author: abishek_bupathi
 */

#ifndef INC_STM32F767XX_GPIO_DRIVER_H_
#define INC_STM32F767XX_GPIO_DRIVER_H_

#include "stm32f767xx.h"


typedef struct{

	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;					/*<Available modes: @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinSpeed;					/*<Available modes: @GPIO_PIN_PUPD_CONFIG>*/
	uint8_t GPIO_PinPuPdControl;			/*<Available modes: @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinOPType;					/*<Available modes: @GPIO_PIN_OP_TYPES>*/
	uint8_t GPIO_PinAltFunMode;				/*<Available modes: @GPIO_PIN_PUPD_CONFIG>*/


}GPIO_PinConfig_t;


typedef struct{

	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_N0_0 		  0
#define GPIO_PIN_N0_1 		  1
#define GPIO_PIN_N0_2 		  2
#define GPIO_PIN_N0_3 		  3
#define GPIO_PIN_N0_4 		  4
#define GPIO_PIN_N0_5 		  5
#define GPIO_PIN_N0_6 		  6
#define GPIO_PIN_N0_7 		  7
#define GPIO_PIN_N0_8 		  8
#define GPIO_PIN_N0_9 		  9
#define GPIO_PIN_N0_10 		  10
#define GPIO_PIN_N0_11		  11
#define GPIO_PIN_N0_12		  12
#define GPIO_PIN_N0_13		  13
#define GPIO_PIN_N0_14		  14
#define GPIO_PIN_N0_15		  15

/*
 * @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_MODE_IN  	 	  0
#define GPIO_MODE_OUT	 	  1
#define GPIO_MODE_ALTFN  	  2
#define GPIO_MODE_ANALOG	  3
#define GPIO_MODE_IT_FT		  4
#define GPIO_MODE_IT_RT       5
#define GPIO_MODE_IT_RFT	  6

/*
 * @GPIO_PIN_OP_TYPES
 * GPIO pin output types
 */
#define GPIO_OP_TYPE_PP		  0
#define GPIO_OP_TYPE_OD		  1

/*
 * @GPIO_PIN_OP_SPEED
 * GPIO pin output speed
 */
#define GPIO_SPEED_LOW		  0
#define GPIO_SPEED_MEDIUM	  1
#define GPIO_SPEED_FAST		  2
#define GPIO_SPEED_HIGH		  3

/*
 * @GPIO_PIN_PUPD_CONFIG
 * GPIO pin pull up and pull down configurations
 */
#define GPIO_NO_PUPD		  0
#define GPIO_PIN_PU			  1
#define GPIO_PIN_PD			  2


/*******************************************************************************
 * 						APIs supported  by this driver
 *******************************************************************************/

// Clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// Init and DeInit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ config and handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t IRQEnorDi);
void GPIO_IRQHandle(uint8_t PinNumber);

#endif /* INC_STM32F767XX_GPIO_DRIVER_H_ */
