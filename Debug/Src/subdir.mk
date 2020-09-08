################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/06_spi_txonly_arduino.c 

OBJS += \
./Src/06_spi_txonly_arduino.o 

C_DEPS += \
./Src/06_spi_txonly_arduino.d 


# Each subdirectory must supply rules for building sources it contributes
Src/06_spi_txonly_arduino.o: ../Src/06_spi_txonly_arduino.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32 -DNUCLEO_F767ZI -DSTM32F7 -DDEBUG -DSTM32F767ZITx -c -I../Inc -I"D:/CODING/STM32/STM32F7XX_DRIVERS/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/06_spi_txonly_arduino.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

