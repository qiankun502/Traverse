################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lsm6dso/lsm6dso.c \
../Drivers/BSP/Components/lsm6dso/lsm6dso_reg.c 

OBJS += \
./Drivers/BSP/Components/lsm6dso/lsm6dso.o \
./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lsm6dso/lsm6dso.d \
./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lsm6dso/%.o Drivers/BSP/Components/lsm6dso/%.su Drivers/BSP/Components/lsm6dso/%.cyclo: ../Drivers/BSP/Components/lsm6dso/%.c Drivers/BSP/Components/lsm6dso/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_PWR_LDO_SUPPLY -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"D:/Traverse_Door/Drivers/BSP/Components/lsm6dso" -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/lsm6dso -I../Core/Src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dso

clean-Drivers-2f-BSP-2f-Components-2f-lsm6dso:
	-$(RM) ./Drivers/BSP/Components/lsm6dso/lsm6dso.cyclo ./Drivers/BSP/Components/lsm6dso/lsm6dso.d ./Drivers/BSP/Components/lsm6dso/lsm6dso.o ./Drivers/BSP/Components/lsm6dso/lsm6dso.su ./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.cyclo ./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.d ./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.o ./Drivers/BSP/Components/lsm6dso/lsm6dso_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dso

