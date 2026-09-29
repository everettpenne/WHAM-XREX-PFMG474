################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/app/control/pfm.c \
../src/app/control/pid.c \
../src/app/control/state_machine.c

OBJS += \
./src/app/control/pfm.o \
./src/app/control/pid.o \
./src/app/control/state_machine.o

C_DEPS += \
./src/app/control/pfm.d \
./src/app/control/pid.d \
./src/app/control/state_machine.d


# Each subdirectory must supply rules for building sources it contributes
src/app/control/%.o src/app/control/%.su src/app/control/%.cyclo: ../src/app/control/%.c src/app/control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../src/app -I../src/app/commands -I../src/app/control -I../src/app/protection -I../src/app/sim -I../src/bsp/stm32g4 -I../src/config -I../src/drivers -I../src/middleware/scpi -I../build/generated -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-app-2f-control

clean-src-2f-app-2f-control:
	-$(RM) ./src/app/control/pfm.cyclo ./src/app/control/pfm.d ./src/app/control/pfm.o ./src/app/control/pfm.su ./src/app/control/pid.cyclo ./src/app/control/pid.d ./src/app/control/pid.o ./src/app/control/pid.su ./src/app/control/state_machine.cyclo ./src/app/control/state_machine.d ./src/app/control/state_machine.o ./src/app/control/state_machine.su

.PHONY: clean-src-2f-app-2f-control

