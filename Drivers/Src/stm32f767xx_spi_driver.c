/*
 * stm32f767xx_spi_driver.c
 *
 *  Created on: 3 Sep 2020
 *      Author: abishek_bupathi
 */

#include"stm32f767xx_spi_driver.h"

/*****************************************************************
 * @function				- SPI_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the SPI peripheral
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if(EnorDi == ENABLE){

			if(pSPIx == SPI1){
				SPI1_PCLK_EN();
			}else if(pSPIx == SPI2){
				SPI2_PCLK_EN();
			}else if(pSPIx == SPI3){
				SPI3_PCLK_EN();
			}else if(pSPIx == SPI4){
				SPI4_PCLK_EN();
			}else if(pSPIx == SPI5){
				SPI5_PCLK_EN();
			}else if(pSPIx == SPI6){
				SPI6_PCLK_EN();
			}


		}else{

			if(pSPIx == SPI1){
				SPI1_PCLK_DI();
			}else if(pSPIx == SPI2){
				SPI2_PCLK_DI();
			}else if(pSPIx == SPI3){
				SPI3_PCLK_DI();
			}else if(pSPIx == SPI4){
				SPI4_PCLK_DI();
			}else if(pSPIx == SPI5){
				SPI5_PCLK_DI();
			}else if(pSPIx == SPI6){
				SPI6_PCLK_DI();
			}

		}

}

/*****************************************************************
 * @function				- SPI_Init
 *
 * @description				- The function initializes SPI peripheral
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

	// Configure CR1 Register

	uint32_t tempreg = 0;

	// 1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//bidi mode cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_DD){
		// bidi mode set
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_RXONLY){
		// bidi mode cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// RXOnly bit set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure SPI serial clock speed (Baud Rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_CRCL;

	// 5. Configure CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 4. Configure CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;

}


/*****************************************************************
 * @function				- SPI_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the SPI peripheral
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){

}


/*****************************************************************
 * @function				- SPI_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the SPI peripheral
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

}


/*****************************************************************
 * @function				- SPI_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the SPI peripheral
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

}


/*****************************************************************
 * @function				- SPI_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the SPI peripheral
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQEnorDi){

}


/*****************************************************************
 * @function				- SPI_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the SPI peripheral
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

}


/*****************************************************************
 * @function				- SPI_PeriClockControl
 *
 * @description				- The function enables or disables peripheral clock for the SPI peripheral
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_IRQHandle(SPI_Handle_t *pHandle){

}
