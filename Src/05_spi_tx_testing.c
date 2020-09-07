/*
 * 05_spi_tx_testing.c
 *
 *  Created on: 5 Sep 2020
 *      Author: Abishek Bupathi
 *
 ************************************
 *
 *      Pin Configurations:
 *
 *      PB15 --> SPI2_MOSI
 *      PB14 --> SPI2_MISO
 *      PB13 --> SPI2_SCLK
 *      PB12 --> SPI2_NSS
 *
 *      ALT Function mode: 5
 *
 *
 *	Only Master is used ( No slave attached )
 */


#include "string.h"
#include "stm32f767xx.h"



/*****************************************************************
 * @function				- SPI_GPIOInit
 *
 * @description				- The function initializes the Alternate function of GPIO Pins for SPI2
 *
 * @params[in]				- None
 *
 * @return					- None
 *
 * @note					- None
 *
 */

void SPI2_GPIOInits(void){


	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
}



void SPI2_Inits(void){

	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2Handle);

}


int main(void){

	char user_data[] = "Hello World";

	SPI2_GPIOInits();

	SPI2_Inits();

	// This makes NSS signal internally HIGH
	SPI_SSIConfig(SPI2, ENABLE);

	// Enable SPI2 Peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	// Closing the communication
	SPI_PeripheralControl(SPI2, DISABLE);
	while(1);

	return 0;
}
