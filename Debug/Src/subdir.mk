################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/04_button_interrupt.c 

OBJS += \
./Src/04_button_interrupt.o 

C_DEPS += \
./Src/04_button_interrupt.d 


# Each subdirectory must supply rules for building sources it contributes
Src/04_button_interrupt.o: ../Src/04_button_interrupt.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DNUCLEO_F767ZI -DSTM32F7 -DDEBUG -DSTM32F767ZITx -c -I../Inc -I"/run/media/abishek_bupathi/Productivity/CODING/STM32/STM32F7XX_DRIVERS/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/04_button_interrupt.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

