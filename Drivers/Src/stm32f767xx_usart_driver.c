/*
 * stm32f767xx_usart_driver.c
 *
 *  Created on: 21 Sep 2020
 *      Author: Abishek Bupathi
 */

#include "stm32f767xx_usart_driver.h"


/*****************************************************************
 * @function				- USART_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the given USART port
 *
 * @params[in]				- Base address of the USART peripheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){

			if(pUSARTx == USART1){
				USART1_PCLK_EN();
			}else if(pUSARTx == USART2){
				USART2_PCLK_EN();
			}else if(pUSARTx == USART3){
				USART3_PCLK_EN();
			}else if(pUSARTx == UART4){
				UART4_PCLK_EN();
			}else if(pUSARTx == UART5){
				UART5_PCLK_EN();
			}else if(pUSARTx == USART6){
				USART6_PCLK_EN();
			}else if(pUSARTx == UART7){
				UART7_PCLK_EN();
			}else if(pUSARTx == UART8){
				UART8_PCLK_EN();
			}

	}else{

			if(pUSARTx == USART1){
				USART1_PCLK_DI();
			}else if(pUSARTx == USART2){
				USART2_PCLK_DI();
			}else if(pUSARTx == USART3){
				USART3_PCLK_DI();
			}else if(pUSARTx == UART4){
				UART4_PCLK_DI();
			}else if(pUSARTx == UART5){
				UART5_PCLK_DI();
			}else if(pUSARTx == USART6){
				USART6_PCLK_DI();
			}else if(pUSARTx == UART7){
				UART7_PCLK_DI();
			}else if(pUSARTx == UART8){
				UART8_PCLK_DI();
			}

	}


}

/*****************************************************************
 * @function				- USART_Init
 *
 * @description				- The function Initializer the given USART/UART Peripheral
 *
 * @params[in]				- Base address of the USART peripheral
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void USART_Init(USART_Handle_t *pUSARTHandle){



}

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
void USART_DeInit(USART_Handle_t *pUSARTHandle);


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
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);

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
void  USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);

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
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);

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
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);

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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

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
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

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
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);

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
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

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
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

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
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

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
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t ApEv);
