################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../.ai/run/run-3/Core/Startup/startup_stm32f446retx.s 

OBJS += \
./.ai/run/run-3/Core/Startup/startup_stm32f446retx.o 

S_DEPS += \
./.ai/run/run-3/Core/Startup/startup_stm32f446retx.d 


# Each subdirectory must supply rules for building sources it contributes
.ai/run/run-3/Core/Startup/%.o: ../.ai/run/run-3/Core/Startup/%.s .ai/run/run-3/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean--2e-ai-2f-run-2f-run-2d-3-2f-Core-2f-Startup

clean--2e-ai-2f-run-2f-run-2d-3-2f-Core-2f-Startup:
	-$(RM) ./.ai/run/run-3/Core/Startup/startup_stm32f446retx.d ./.ai/run/run-3/Core/Startup/startup_stm32f446retx.o

.PHONY: clean--2e-ai-2f-run-2f-run-2d-3-2f-Core-2f-Startup

