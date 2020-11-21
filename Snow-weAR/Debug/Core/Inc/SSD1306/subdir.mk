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
Core/Inc/SSD1306/ssd1306.o: ../Core/Inc/SSD1306/ssd1306.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Core/Inc/SSD1306/ssd1306.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Inc/SSD1306/ssd1306_fonts.o: ../Core/Inc/SSD1306/ssd1306_fonts.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Core/Inc/SSD1306/ssd1306_fonts.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Inc/SSD1306/ssd1306_tests.o: ../Core/Inc/SSD1306/ssd1306_tests.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Core/Inc/SSD1306/ssd1306_tests.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

