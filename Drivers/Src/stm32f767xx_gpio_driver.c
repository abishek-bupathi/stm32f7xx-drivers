/*
 * stm32f767xx_gpio_driver.c
 *
 *  Created on: 21 Aug 2020
 *      Author: abishek_bupathi
 */

#include "stm32f767xx_gpio_driver.h"


/*****************************************************************
 * @function				- GPIO_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the given GPIO port
 *
 * @params[in]				- Base address of the GPIO peripheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){

		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}else if(pGPIOx == GPIOJ){
			GPIOJ_PCLK_EN();
		}else if(pGPIOx == GPIOK){
			GPIOK_PCLK_EN();
		}


	}else{

		if(pGPIOx == GPIOA){
					GPIOA_PCLK_DI();
				}else if(pGPIOx == GPIOB){
					GPIOB_PCLK_DI();
				}else if(pGPIOx == GPIOC){
					GPIOC_PCLK_DI();
				}else if(pGPIOx == GPIOD){
					GPIOD_PCLK_DI();
				}else if(pGPIOx == GPIOE){
					GPIOE_PCLK_DI();
				}else if(pGPIOx == GPIOF){
					GPIOF_PCLK_DI();
				}else if(pGPIOx == GPIOG){
					GPIOG_PCLK_DI();
				}else if(pGPIOx == GPIOH){
					GPIOH_PCLK_DI();
				}else if(pGPIOx == GPIOI){
					GPIOI_PCLK_DI();
				}else if(pGPIOx == GPIOJ){
					GPIOJ_PCLK_DI();
				}else if(pGPIOx == GPIOK){
					GPIOK_PCLK_DI();
				}

	}

}


/*****************************************************************
 * @function				- GPIO_Init
 *
 * @description				- The function initializes the GPIO Port
 *
 * @params[in]				- Base address of the GPIO peripheral
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0;

	// 1. Configure the Mode of the GPIO pin
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		// Non-Interrupt Mode

		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
		pGPIOHandle->pGPIOx -> MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // setting
		pGPIOHandle->pGPIOx -> MODER |= temp;

	}else{
		// Interrupt Mode

	}

	temp = 0;

	// 2. Configure the Speed
	temp = pGPIOHandle-> GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx -> OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx -> OSPEEDR |= temp;

	temp = 0;

	// 3. Configure the PuPd settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx -> PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx -> PUPDR |= temp;

	temp = 0;

	// 4. Configure the OPType
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx -> OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx -> OTYPER |= temp;

	temp = 0;

	// 5. Configure the ALT Functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << 4 * temp2);
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << 4 * temp2;
	}

}


/*****************************************************************
 * @function				- GPIO_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the
 *
 * @params[in]				- Base address of the GPIO peripheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){



}



/*****************************************************************
 * @function				- GPIO_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the
 *
 * @params[in]				- Base address of the GPIO peripheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){



}



/*****************************************************************
 * @function				- GPIO_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the
 *
 * @params[in]				- Base address of the GPIO peripheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){



}



/*****************************************************************
 * @function				- GPIO_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the
 *
 * @params[in]				- Base address of the GPIO peripheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){



}



/*****************************************************************
 * @function				- GPIO_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the
 *
 * @params[in]				- Base address of the GPIO peripheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){



}



/*****************************************************************
 * @function				- GPIO_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the
 *
 * @params[in]				- Base address of the GPIO peripheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){



}




/*****************************************************************
 * @function				- GPIO_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the
 *
 * @params[in]				- Base address of the GPIO peripheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t IRQEnorDi){



}



/*****************************************************************
 * @function				- GPIO_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the
 *
 * @params[in]				- Base address of the GPIO peripheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void GPIO_IRQHandle(uint8_t PinNumber){



}
