################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DDSM115.c \
../Core/Src/StartupStrategy.c \
../Core/Src/StateEstimator.c \
../Core/Src/controler.c \
../Core/Src/cybergear.c \
../Core/Src/cybergear_angle_protection.c \
../Core/Src/high_level_functions.c \
../Core/Src/joystick.c \
../Core/Src/kinematics.c \
../Core/Src/main.c \
../Core/Src/spring_damp_system.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_init.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/telemetry.c \
../Core/Src/watchdog.c 

OBJS += \
./Core/Src/DDSM115.o \
./Core/Src/StartupStrategy.o \
./Core/Src/StateEstimator.o \
./Core/Src/controler.o \
./Core/Src/cybergear.o \
./Core/Src/cybergear_angle_protection.o \
./Core/Src/high_level_functions.o \
./Core/Src/joystick.o \
./Core/Src/kinematics.o \
./Core/Src/main.o \
./Core/Src/spring_damp_system.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_init.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/telemetry.o \
./Core/Src/watchdog.o 

C_DEPS += \
./Core/Src/DDSM115.d \
./Core/Src/StartupStrategy.d \
./Core/Src/StateEstimator.d \
./Core/Src/controler.d \
./Core/Src/cybergear.d \
./Core/Src/cybergear_angle_protection.d \
./Core/Src/high_level_functions.d \
./Core/Src/joystick.d \
./Core/Src/kinematics.d \
./Core/Src/main.d \
./Core/Src/spring_damp_system.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_init.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/telemetry.d \
./Core/Src/watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc -gdwarf-4 "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/DDSM115.cyclo ./Core/Src/DDSM115.d ./Core/Src/DDSM115.o ./Core/Src/DDSM115.su ./Core/Src/StartupStrategy.cyclo ./Core/Src/StartupStrategy.d ./Core/Src/StartupStrategy.o ./Core/Src/StartupStrategy.su ./Core/Src/StateEstimator.cyclo ./Core/Src/StateEstimator.d ./Core/Src/StateEstimator.o ./Core/Src/StateEstimator.su ./Core/Src/controler.cyclo ./Core/Src/controler.d ./Core/Src/controler.o ./Core/Src/controler.su ./Core/Src/cybergear.cyclo ./Core/Src/cybergear.d ./Core/Src/cybergear.o ./Core/Src/cybergear.su ./Core/Src/cybergear_angle_protection.cyclo ./Core/Src/cybergear_angle_protection.d ./Core/Src/cybergear_angle_protection.o ./Core/Src/cybergear_angle_protection.su ./Core/Src/high_level_functions.cyclo ./Core/Src/high_level_functions.d ./Core/Src/high_level_functions.o ./Core/Src/high_level_functions.su ./Core/Src/joystick.cyclo ./Core/Src/joystick.d ./Core/Src/joystick.o ./Core/Src/joystick.su ./Core/Src/kinematics.cyclo ./Core/Src/kinematics.d ./Core/Src/kinematics.o ./Core/Src/kinematics.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/spring_damp_system.cyclo ./Core/Src/spring_damp_system.d ./Core/Src/spring_damp_system.o ./Core/Src/spring_damp_system.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_init.cyclo ./Core/Src/system_init.d ./Core/Src/system_init.o ./Core/Src/system_init.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/telemetry.cyclo ./Core/Src/telemetry.d ./Core/Src/telemetry.o ./Core/Src/telemetry.su ./Core/Src/watchdog.cyclo ./Core/Src/watchdog.d ./Core/Src/watchdog.o ./Core/Src/watchdog.su

.PHONY: clean-Core-2f-Src

