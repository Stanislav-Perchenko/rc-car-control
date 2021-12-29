################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c 

OBJS += \
./Drivers/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.o 

C_DEPS += \
./Drivers/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F10x_StdPeriph_Driver/src/%.o: ../Drivers/STM32F10x_StdPeriph_Driver/src/%.c Drivers/STM32F10x_StdPeriph_Driver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=500 -DLSE_VALUE=32768 -DHSI_VALUE=8000000 -DLSI_VALUE=40000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I"C:/ST/projects/rc-car-control/STM32CubeIDE/Drivers/STM32F10x_StdPeriph_Driver" -I"C:/ST/projects/rc-car-control/STM32CubeIDE/Drivers/STM32F10x_StdPeriph_Driver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

