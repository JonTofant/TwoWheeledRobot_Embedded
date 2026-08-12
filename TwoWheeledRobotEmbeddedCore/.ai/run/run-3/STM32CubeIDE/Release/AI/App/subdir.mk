################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/jon/OneDrive\ -\ Univerza\ v\ Mariboru/Dokumenti/GitHub/BipedRobot/STM32_Diablo_Robot_Source/.ai/run/run-3/AI/App/app_x-cube-ai.c \
C:/Users/jon/OneDrive\ -\ Univerza\ v\ Mariboru/Dokumenti/GitHub/BipedRobot/STM32_Diablo_Robot_Source/.ai/run/run-3/AI/App/network.c \
C:/Users/jon/OneDrive\ -\ Univerza\ v\ Mariboru/Dokumenti/GitHub/BipedRobot/STM32_Diablo_Robot_Source/.ai/run/run-3/AI/App/network_data.c \
C:/Users/jon/OneDrive\ -\ Univerza\ v\ Mariboru/Dokumenti/GitHub/BipedRobot/STM32_Diablo_Robot_Source/.ai/run/run-3/AI/App/network_weights.c \
C:/Users/jon/OneDrive\ -\ Univerza\ v\ Mariboru/Dokumenti/GitHub/BipedRobot/STM32_Diablo_Robot_Source/.ai/run/run-3/AI/App/user_init.c 

OBJS += \
./AI/App/app_x-cube-ai.o \
./AI/App/network.o \
./AI/App/network_data.o \
./AI/App/network_weights.o \
./AI/App/user_init.o 

C_DEPS += \
./AI/App/app_x-cube-ai.d \
./AI/App/network.d \
./AI/App/network_data.d \
./AI/App/network_weights.d \
./AI/App/user_init.d 


# Each subdirectory must supply rules for building sources it contributes
AI/App/app_x-cube-ai.o: C:/Users/jon/OneDrive\ -\ Univerza\ v\ Mariboru/Dokumenti/GitHub/BipedRobot/STM32_Diablo_Robot_Source/.ai/run/run-3/AI/App/app_x-cube-ai.c AI/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -DHAVE_NETWORK_INFO -c -I../../Core/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/AI/Inc -I../../Middlewares/ST/AI/Misc/Inc -I../../Middlewares/ST/AI/Validation/Inc -I../../Middlewares/ST/AI/Misc/Src -I../../Middlewares/ST/AI/Validation/Src -I../../AI/App -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"AI/App/app_x-cube-ai.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
AI/App/network.o: C:/Users/jon/OneDrive\ -\ Univerza\ v\ Mariboru/Dokumenti/GitHub/BipedRobot/STM32_Diablo_Robot_Source/.ai/run/run-3/AI/App/network.c AI/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -DHAVE_NETWORK_INFO -c -I../../Core/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/AI/Inc -I../../Middlewares/ST/AI/Misc/Inc -I../../Middlewares/ST/AI/Validation/Inc -I../../Middlewares/ST/AI/Misc/Src -I../../Middlewares/ST/AI/Validation/Src -I../../AI/App -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"AI/App/network.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
AI/App/network_data.o: C:/Users/jon/OneDrive\ -\ Univerza\ v\ Mariboru/Dokumenti/GitHub/BipedRobot/STM32_Diablo_Robot_Source/.ai/run/run-3/AI/App/network_data.c AI/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -DHAVE_NETWORK_INFO -c -I../../Core/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/AI/Inc -I../../Middlewares/ST/AI/Misc/Inc -I../../Middlewares/ST/AI/Validation/Inc -I../../Middlewares/ST/AI/Misc/Src -I../../Middlewares/ST/AI/Validation/Src -I../../AI/App -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"AI/App/network_data.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
AI/App/network_weights.o: C:/Users/jon/OneDrive\ -\ Univerza\ v\ Mariboru/Dokumenti/GitHub/BipedRobot/STM32_Diablo_Robot_Source/.ai/run/run-3/AI/App/network_weights.c AI/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -DHAVE_NETWORK_INFO -c -I../../Core/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/AI/Inc -I../../Middlewares/ST/AI/Misc/Inc -I../../Middlewares/ST/AI/Validation/Inc -I../../Middlewares/ST/AI/Misc/Src -I../../Middlewares/ST/AI/Validation/Src -I../../AI/App -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"AI/App/network_weights.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
AI/App/user_init.o: C:/Users/jon/OneDrive\ -\ Univerza\ v\ Mariboru/Dokumenti/GitHub/BipedRobot/STM32_Diablo_Robot_Source/.ai/run/run-3/AI/App/user_init.c AI/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -DHAVE_NETWORK_INFO -c -I../../Core/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/AI/Inc -I../../Middlewares/ST/AI/Misc/Inc -I../../Middlewares/ST/AI/Validation/Inc -I../../Middlewares/ST/AI/Misc/Src -I../../Middlewares/ST/AI/Validation/Src -I../../AI/App -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"AI/App/user_init.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-AI-2f-App

clean-AI-2f-App:
	-$(RM) ./AI/App/app_x-cube-ai.cyclo ./AI/App/app_x-cube-ai.d ./AI/App/app_x-cube-ai.o ./AI/App/app_x-cube-ai.su ./AI/App/network.cyclo ./AI/App/network.d ./AI/App/network.o ./AI/App/network.su ./AI/App/network_data.cyclo ./AI/App/network_data.d ./AI/App/network_data.o ./AI/App/network_data.su ./AI/App/network_weights.cyclo ./AI/App/network_weights.d ./AI/App/network_weights.o ./AI/App/network_weights.su ./AI/App/user_init.cyclo ./AI/App/user_init.d ./AI/App/user_init.o ./AI/App/user_init.su

.PHONY: clean-AI-2f-App

