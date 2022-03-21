################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MS5837-30BA.c \
../Core/Src/MadgwickAHRS.c \
../Core/Src/Thruster.c \
../Core/Src/imu9dof.c \
../Core/Src/l3gd20.c \
../Core/Src/lsm303dlhc.c \
../Core/Src/main.c \
../Core/Src/max7456.c \
../Core/Src/osdWidgets.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f3xx.c 

OBJS += \
./Core/Src/MS5837-30BA.o \
./Core/Src/MadgwickAHRS.o \
./Core/Src/Thruster.o \
./Core/Src/imu9dof.o \
./Core/Src/l3gd20.o \
./Core/Src/lsm303dlhc.o \
./Core/Src/main.o \
./Core/Src/max7456.o \
./Core/Src/osdWidgets.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f3xx.o 

C_DEPS += \
./Core/Src/MS5837-30BA.d \
./Core/Src/MadgwickAHRS.d \
./Core/Src/Thruster.d \
./Core/Src/imu9dof.d \
./Core/Src/l3gd20.d \
./Core/Src/lsm303dlhc.d \
./Core/Src/main.d \
./Core/Src/max7456.d \
./Core/Src/osdWidgets.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MS5837-30BA.d ./Core/Src/MS5837-30BA.o ./Core/Src/MadgwickAHRS.d ./Core/Src/MadgwickAHRS.o ./Core/Src/Thruster.d ./Core/Src/Thruster.o ./Core/Src/imu9dof.d ./Core/Src/imu9dof.o ./Core/Src/l3gd20.d ./Core/Src/l3gd20.o ./Core/Src/lsm303dlhc.d ./Core/Src/lsm303dlhc.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/max7456.d ./Core/Src/max7456.o ./Core/Src/osdWidgets.d ./Core/Src/osdWidgets.o ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o

.PHONY: clean-Core-2f-Src

