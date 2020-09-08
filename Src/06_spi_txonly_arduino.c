/*
 * 06_spi_txonly_arduino.c
 *
 *  Created on: 8 Sep 2020
 *  Author: Abishek Bupathi
 *
 *  ************************************
 *
 *  Pin Configurations:
 *
 *   PB15 --> SPI2_MOSI
 *   PB14 --> SPI2_MISO
 *   PB13 --> SPI2_SCLK
 *   PB12 --> SPI2_NSS
 *
 *   ALT Function mode: 5
 *
 *   Master: STM32
 *   Slave: Arduino Uno
 *
 */



#include "string.h"
#include "stm32f767xx.h"


void delay(void){
	for(uint32_t i = 0; i < 500000/2; i++ );
}

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
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

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
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management

	SPI_Init(&SPI2Handle);

}

void GPIO_ButtonInit(void){

	GPIO_Handle_t GpioBtn;

	// Initialising Button
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);

}


int main(void){


	char user_data[] = "Hi, this is a test for sending string data from STM32 MCU to Arduino UNO via SPI Communication interface";

	GPIO_ButtonInit();

	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){

		while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		// Enable SPI2 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// Send Length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		// Send data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// Conforming SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// Closing the communication
		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}
