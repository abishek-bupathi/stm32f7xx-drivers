/*
 * 04_button_interrupt.c
 *
 *  Created on: 31 Aug 2020
 *      Author: abishek_bupathi
 */



#include <stm32f767xx.h>
#include <string.h>

#define HIGH 1
#define LOW 0

#define BUTTON_PRESSED LOW


void delay(void){
	for(uint32_t i = 0; i < 500000/2; i++ );
}

int main(void){

	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	// Initialising Led
	GpioLed.pGPIOx = GPIOB;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);

	// Initialising interrupt Button
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);

	// IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_PRIORITY_15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	return 0;

}

void EXTI15_10_IRQHandler(void){
	delay();
	GPIO_IRQHandle(GPIO_PIN_NO_13);
	GPIO_TogglePin(GPIOB, GPIO_PIN_NO_7);

}
