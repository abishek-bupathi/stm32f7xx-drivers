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

// Command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT   		0x53
#define COMMAND_ID_READ 		0x54

#define LED_ON					0
#define LED_OFF					1

// Arduino Analog Pins
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4
#define ANALOG_PIN5				5

// Ardiuno LED pin
#define LED_PIN					9


void delay(void){
	for(uint32_t i = 0; i < 500000; i++ );
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
		SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

		//SCLK
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		GPIO_Init(&SPIPins);

		//MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
		GPIO_Init(&SPIPins);

		//MISO
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
		GPIO_Init(&SPIPins);


		//NSS
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GPIO_Init(&SPIPins);
}



void SPI2_Inits(void){

	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV16;
	SPI2Handle.SPIConfig.SPI_CRCL = SPI_DFF_8BITS;
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
	GPIO_Init(&GpioBtn);

}


uint8_t SPI_VerifyResponse(uint8_t ackbyte){

	if(ackbyte == 0xF5){
		//ack
		return 1;

	}else{
		return 0;
	}

}


int main(void){


	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	GPIO_ButtonInit();

	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){

		while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		// Enable SPI2 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// 1. CMD_LED_CTRL		<pin no(1)>		<value(1)>

		uint8_t cmdcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		// send command
		SPI_SendData(SPI2, &cmdcode, 1);

		// dummy read to clear off RXNE
		SPI_RecieveData(SPI2, &dummy_read, 1);

		// send dummy data to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		SPI_RecieveData(SPI2, &ackbyter,1);

		if(SPI_VerifyResponse(ackbyte)){

			// Send arguments
			arg[0]  = LED_PIN;
			args[1] = LED_ON

			SPI_SendData(SPI2, &args, 1);
		}


		// Conforming SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );


		// Closing the communication
		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}
