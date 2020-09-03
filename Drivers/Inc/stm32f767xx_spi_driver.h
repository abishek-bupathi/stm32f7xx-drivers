/*
 * stm32f767xx_spi_driver.h
 *
 *  Created on: 3 Sep 2020
 *      Author: abishek_bupathi
 */

#ifndef INC_STM32F767XX_SPI_DRIVER_H_
#define INC_STM32F767XX_SPI_DRIVER_H_

#include "stm32f767xx.h"


// Configuration structure for SPI Periheral
typedef struct{

	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;


// Handle structure for SPI peripheral
typedef struct{

	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;

}SPI_Handle_t;


/*********************************************************************************************
 *
 * 								APIs supported by this Driver
 *
 ********************************************************************************************/

// Peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// Init and DeInit
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// IRQ Configurationa and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQEnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandle(SPI_Handle_t *pHandle);

#endif /* INC_STM32F767XX_SPI_DRIVER_H_ */
