################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../adxl345/drv_spi.c 

OBJS += \
./adxl345/drv_spi.o 

C_DEPS += \
./adxl345/drv_spi.d 


# Each subdirectory must supply rules for building sources it contributes
adxl345/%.o adxl345/%.su adxl345/%.cyclo: ../adxl345/%.c adxl345/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/eliof/STM32CubeIDE/workspace_1.15.1/2526_MSC_RTOS_TP2/shell" -I"C:/Users/eliof/STM32CubeIDE/workspace_1.15.1/2526_MSC_RTOS_TP2/adxl345" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-adxl345

clean-adxl345:
	-$(RM) ./adxl345/drv_spi.cyclo ./adxl345/drv_spi.d ./adxl345/drv_spi.o ./adxl345/drv_spi.su

.PHONY: clean-adxl345

