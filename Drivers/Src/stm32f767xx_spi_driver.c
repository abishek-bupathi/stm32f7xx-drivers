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

// Init and DeInit
void SPI_Init(SPI_Handle_t *pSPIHandle){

}
void SPI_DeInit(SPI_RegDef_t *pSPIx){

}

// Data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

}
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

}

// IRQ Configurationa and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQEnorDi){

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

}
void SPI_IRQHandle(SPI_Handle_t *pHandle){

}
