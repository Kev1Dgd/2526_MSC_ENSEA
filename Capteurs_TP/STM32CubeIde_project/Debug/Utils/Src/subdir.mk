################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utils/Src/compass.c \
../Utils/Src/functions.c \
../Utils/Src/tools.c 

OBJS += \
./Utils/Src/compass.o \
./Utils/Src/functions.o \
./Utils/Src/tools.o 

C_DEPS += \
./Utils/Src/compass.d \
./Utils/Src/functions.d \
./Utils/Src/tools.d 


# Each subdirectory must supply rules for building sources it contributes
Utils/Src/%.o Utils/Src/%.su Utils/Src/%.cyclo: ../Utils/Src/%.c Utils/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kevin/STM32CubeIDE/workspace_1.19.0/G431-MSC-Sensors/Utils/Inc" -I"C:/Users/kevin/STM32CubeIDE/workspace_1.19.0/G431-MSC-Sensors/Utils/Inc" -O0 -ffunction-sections -fdata-sections -Wall -u _printf_float -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Utils-2f-Src

clean-Utils-2f-Src:
	-$(RM) ./Utils/Src/compass.cyclo ./Utils/Src/compass.d ./Utils/Src/compass.o ./Utils/Src/compass.su ./Utils/Src/functions.cyclo ./Utils/Src/functions.d ./Utils/Src/functions.o ./Utils/Src/functions.su ./Utils/Src/tools.cyclo ./Utils/Src/tools.d ./Utils/Src/tools.o ./Utils/Src/tools.su

.PHONY: clean-Utils-2f-Src

