################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32_MAX30102_HAL/max30102_for_stm32_hal.c 

OBJS += \
./Drivers/STM32_MAX30102_HAL/max30102_for_stm32_hal.o 

C_DEPS += \
./Drivers/STM32_MAX30102_HAL/max30102_for_stm32_hal.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32_MAX30102_HAL/%.o Drivers/STM32_MAX30102_HAL/%.su Drivers/STM32_MAX30102_HAL/%.cyclo: ../Drivers/STM32_MAX30102_HAL/%.c Drivers/STM32_MAX30102_HAL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I"C:/Users/sethb/OneDrive/Documents/Workspaces/STM/Smart_Watch/Drivers/OLED" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32_MAX30102_HAL -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32_MAX30102_HAL

clean-Drivers-2f-STM32_MAX30102_HAL:
	-$(RM) ./Drivers/STM32_MAX30102_HAL/max30102_for_stm32_hal.cyclo ./Drivers/STM32_MAX30102_HAL/max30102_for_stm32_hal.d ./Drivers/STM32_MAX30102_HAL/max30102_for_stm32_hal.o ./Drivers/STM32_MAX30102_HAL/max30102_for_stm32_hal.su

.PHONY: clean-Drivers-2f-STM32_MAX30102_HAL

