################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/SSD1306/ssd1306.c \
../Core/Inc/SSD1306/ssd1306_fonts.c \
../Core/Inc/SSD1306/ssd1306_tests.c 

OBJS += \
./Core/Inc/SSD1306/ssd1306.o \
./Core/Inc/SSD1306/ssd1306_fonts.o \
./Core/Inc/SSD1306/ssd1306_tests.o 

C_DEPS += \
./Core/Inc/SSD1306/ssd1306.d \
./Core/Inc/SSD1306/ssd1306_fonts.d \
./Core/Inc/SSD1306/ssd1306_tests.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/SSD1306/%.o: ../Core/Inc/SSD1306/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/kenmo/OneDrive/Documents/Snow-weAR/Core/Inc" -I"C:/Users/kenmo/OneDrive/Documents/Snow-weAR/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/kenmo/OneDrive/Documents/Snow-weAR/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kenmo/OneDrive/Documents/Snow-weAR/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/kenmo/OneDrive/Documents/Snow-weAR/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


