/*
 * 01_led_toggle.c
 *
 *  Created on: 25 Aug 2020
 *      Author: abishek_bupathi
 */


#include <stm32f767xx.h>

void delay(void){
	for(uint32_t i = 0; i < 500000; i++ );
}

int main(void){

	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOB;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);


	while(1){

		GPIO_TogglePin(GPIOB, GPIO_PIN_NO_7);
		delay();

	}
	return 0;

}

