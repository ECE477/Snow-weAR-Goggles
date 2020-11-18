################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BB.c \
../Core/Src/BQ27441.c \
../Core/Src/imu.c \
../Core/Src/main.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/BB.o \
./Core/Src/BQ27441.o \
./Core/Src/imu.o \
./Core/Src/main.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/BB.d \
./Core/Src/BQ27441.d \
./Core/Src/imu.d \
./Core/Src/main.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32L476xx -I"C:/Users/mjcre/Documents/Snow-weAR-Goggles/Snow-weAR/Core/Inc" -I"C:/Users/mjcre/Documents/Snow-weAR-Goggles/Snow-weAR/Drivers/STM32L4xx_HAL_Driver/Inc" -I"C:/Users/mjcre/Documents/Snow-weAR-Goggles/Snow-weAR/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/mjcre/Documents/Snow-weAR-Goggles/Snow-weAR/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"C:/Users/mjcre/Documents/Snow-weAR-Goggles/Snow-weAR/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


