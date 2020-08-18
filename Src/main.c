/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Abishek Bupathi
 * @brief          : Main program body
 ******************************************************************************
 */

#include "stm32f767xx.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
    /* Loop forever */
	for(;;);
}
