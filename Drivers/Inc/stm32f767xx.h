/*
 * stm32f767xx.h
 *
 *  Created on: Aug 18, 2020
 *      Author: abishek_bupathi
 */

#ifndef INC_STM32F767XX_H_
#define INC_STM32F767XX_H_


/*
 * Dase addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x0800 0000U	// setting base address macro for flash memory
#define SRAM1_BASEADDR			0x2002 0000U	// setting base address macro for SRAM1 memory
#define SRAM1_BASEADDR			0x2007 C000U	// setting base address macro for SRAM2 memory
#define ROM						0x1FF0 0000U	// setting base address macro for ROM memory
#define SRAM 					SRAM1_BASEADDR	// setting default SRAM1 base address as default SRAM base address


/*
 * Dase addresses of APBx and AHBx Peripheral
 */

#define PERIPH_BASEADDR				0X4000 0000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x4001 0000U
#define AHB1PERIPH_BASEADDR			0x4002 0000U
#define AHB2PERIPH_BASEADDR			0x5000 0000U
#define AHB3PERIPH_BASEADDR			0x6000 0000U

/*
 * Base addresses of peripherals on AHB1 bus
 */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0xOOOO)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0xO4OO)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0xO8OO)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0xOCOO)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x10OO)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0xO4OO)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0xO8OO)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1COO)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2OOO)
#define GPIOJ_BASEADDR				(AHB1PERIPH_BASEADDR + 0x24OO)
#define GPIOK_BASEADDR				(AHB1PERIPH_BASEADDR + 0x28OO)

/*
 * Base addresses of peripherals on APB1 bus
 */

#define I2C1_BASEADDR
#endif /* INC_STM32F767XX_H_ */
