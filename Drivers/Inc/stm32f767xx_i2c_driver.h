/*
 * stm32f767xx_i2c_driver.h
 *
 *  Created on: 17 Sep 2020
 *      Author: abishek_bupathi
 */

#ifndef INC_STM32F767XX_I2C_DRIVER_H_
#define INC_STM32F767XX_I2C_DRIVER_H_

#include"stm32f767xx.h"

// Configuration structure for I2C
typedef struct{

	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;

}I2C_Config_t;


// Handle structure for I2C
typedef struct{

	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;

}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM2K		200000
#define I2C_SCL_SPEED_FM4K		400000

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_EN
#define I2C_ACK_DI

/*
 * @I2C_FMDutyCycle
 */
#define

#endif /* INC_STM32F767XX_I2C_DRIVER_H_ */
