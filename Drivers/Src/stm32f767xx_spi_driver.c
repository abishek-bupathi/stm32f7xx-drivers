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

	// Enable SPI Peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Configure CR1 Register

	uint32_t tempreg = 0;

	// 1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//bidi mode cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// bidi mode set
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// bidi mode cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// RXOnly bit set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure SPI serial clock speed (Baud Rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_CRCL << SPI_CR1_CRCL;

	// 5. Configure CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

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


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){

	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;

}


/*****************************************************************
 * @function				- SPI_SendData
 *
 * @description				- The function sends the data
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- TX Buffer pointer (Data)
 * @params[in]				- Length
 * @return					- None
 *
 * @note					- This is a blocking call
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0){

		// 1. Wait till TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check DFF
		if(pSPIx->CR1 & (1 << SPI_CR1_CRCL)){

			// 16bit DFF
			// 1. Load the data into DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;

		}else{
			// 8bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;

		}

	}
}


/*****************************************************************
 * @function				- SPI_RecieveData
 *
 * @description				- The function recieves data
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- RxBuffer
 * @params[in]				- Length
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

	while(Len > 0){

		// 1. Wait till RXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. Check DFF
		if(pSPIx->CR1 & (1 << SPI_CR1_CRCL)){

			// 16bit DFF
			// 1. Load the data from DR to RXBuffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;

		}else{
			// 8bit DFF
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;

		}
	}
}



/*****************************************************************
 * @function				-SPI_IRQConfig
 *
 * @description				- The function enables or IRQ
 *
 * @params[in]				- IRQ Number
 * @params[in]				- IRQ Priority
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQEnorDi){

	if(IRQEnorDi == ENABLE){

		if(IRQNumber <= 31){
			// Program ISER0 Regsiter

			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64){
			// Program ISER1 Regsiter

			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96){
			// Program ISER2 Regsiter

			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}
	} else{

		if(IRQNumber <= 31){
			// Program ICER0 Regsiter

			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64){
			// Program ICER1 Regsiter

			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96){
			// Program ICER2 Regsiter

			*NVIC_ICER2 |= (1 << (IRQNumber % 64));

		}

	}

}



/*****************************************************************
 * @function				- SPI_IRQPriorityConfig
 *
 * @description				- The function sets the Priority register bit
 *
 * @params[in]				- IRQ Number
 * @params[in]				- IRQ Priority
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriotity){

	// 1. Finding the IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amt = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx*4)) |= (IRQPriotity << shift_amt);

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


/*****************************************************************
 * @function				- SPI_SendDataIT
 *
 * @description				- The function sends data through SPI in Interupt mode
 *
 * @params[in]				- SPI peripheral handle
 * @params[in]				- Data to be sent
 * @params[in]				- Length of data
 *
 * @return					- None
 *
 * @note					- None
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){

		// 1. Save the Tx buffer and Len in global variables
		pSPIHandle -> pTxBuffer = pTxBuffer;
		pSPIHandle -> TxLen  	= Len;

		// 2. Set SPI state as busy in transmission
		pSPIHandle -> TxState 	= SPI_BUSY_IN_TX;

		// 3. Enable TXIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle -> pSPIx -> CR2 |= (1 << SPI_CR2_TXEIE);

	}

	return state;
}


/*****************************************************************
 * @function				- SPI_RecieveDataIT
 *
 * @description				- The function recieves data through SPI in interrupt mode
 *
 * @params[in]				- SPI peripheral handle
 * @params[in]				- Data to be recieved
 * @params[in]				- Length of data
 *
 * @return					- None
 *
 * @note					- None
 *
 */
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->RxState;

		if(state != SPI_BUSY_IN_RX){

			// 1. Save the Tx buffer and Len in global variables
			pSPIHandle -> pRxBuffer = pRxBuffer;
			pSPIHandle -> RxLen  	= Len;

			// 2. Set SPI state as busy in transmission
			pSPIHandle -> RxState 	= SPI_BUSY_IN_RX;

			// 3. Enable TXIE control bit to get interrupt whenever TXE flag is set in SR
			pSPIHandle -> pSPIx -> CR2 |= (1 << SPI_CR2_RXNEIE);

		}

		return state;

}


/*****************************************************************
 * @function				- SPI_PeripheralControl
 *
 * @description				- The function enables or disables the SPI peripheral
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){

		pSPIx->CR1 |= (1 << SPI_CR1_SPE);

	}else{

		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);

	}

}


/*****************************************************************
 * @function				- SPI_SSIConfig
 *
 * @description				- The function enables or disables the SPI SSI Register
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){

		pSPIx->CR1 |= (1 << SPI_CR1_SSI);

	}else{

		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);

	}

}


/*****************************************************************
 * @function				- SPI_SSOEConfig
 *
 * @description				- The function enables or disables the SPI SSOE Register
 *
 * @params[in]				- Base address of the SPIperipheral
 * @params[in]				- ENABLE or DISABLE Macros
 *
 * @return					- None
 *
 * @note					- None
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){

		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);

	}else{

		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);

	}

}
