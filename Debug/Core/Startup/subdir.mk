################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l476rgtx.s 

OBJS += \
./Core/Startup/startup_stm32l476rgtx.o 

S_DEPS += \
./Core/Startup/startup_stm32l476rgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/Everett/Documents/GitHub/Smart_Watch_Project_ECEN_361_BYU-I/Drivers/OLED" -I"C:/Users/Everett/Documents/GitHub/Smart_Watch_Project_ECEN_361_BYU-I/Drivers/ADXL345_Driver" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32l476rgtx.d ./Core/Startup/startup_stm32l476rgtx.o

.PHONY: clean-Core-2f-Startup

