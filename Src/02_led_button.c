/*
 * 02_led_button.c
 *
 *  Created on: 25 Aug 2020
 *      Author: Abishek Bupathi
 */


#include <stm32f767xx.h>

#define HIGH 1
#define LOW 0

#define BUTTON_PRESSED HIGH


void delay(void){
	for(uint32_t i = 0; i < 500000/2; i++ );
}

int main(void){

	GPIO_Handle_t GpioLed, GpioBtn;

	// Initialising Led
	GpioLed.pGPIOx = GPIOB;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);

	// Initialising Button
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);


	while(1){

		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BUTTON_PRESSED){
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, HIGH);
		}else{
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, LOW);
		}
	}

	return 0;

}

/*
  if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BUTTON_PRESSED){
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, HIGH);
		}else{
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, LOW);
		}
 */
