################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/B-L475E-IOT01A2/b_l475e_iot01a2_bus.c 

OBJS += \
./Drivers/BSP/B-L475E-IOT01A2/b_l475e_iot01a2_bus.o 

C_DEPS += \
./Drivers/BSP/B-L475E-IOT01A2/b_l475e_iot01a2_bus.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/B-L475E-IOT01A2/%.o Drivers/BSP/B-L475E-IOT01A2/%.su Drivers/BSP/B-L475E-IOT01A2/%.cyclo: ../Drivers/BSP/B-L475E-IOT01A2/%.c Drivers/BSP/B-L475E-IOT01A2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/B-L475E-IOT01A2 -I../Drivers/BSP/Components/lsm6dsl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-B-2d-L475E-2d-IOT01A2

clean-Drivers-2f-BSP-2f-B-2d-L475E-2d-IOT01A2:
	-$(RM) ./Drivers/BSP/B-L475E-IOT01A2/b_l475e_iot01a2_bus.cyclo ./Drivers/BSP/B-L475E-IOT01A2/b_l475e_iot01a2_bus.d ./Drivers/BSP/B-L475E-IOT01A2/b_l475e_iot01a2_bus.o ./Drivers/BSP/B-L475E-IOT01A2/b_l475e_iot01a2_bus.su

.PHONY: clean-Drivers-2f-BSP-2f-B-2d-L475E-2d-IOT01A2

