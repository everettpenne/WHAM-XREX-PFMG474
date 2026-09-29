################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/app/app.c \
../src/app/task_faults.c \
../src/app/task_scpi.c \
../src/app/telemetry.c

OBJS += \
./src/app/app.o \
./src/app/task_faults.o \
./src/app/task_scpi.o \
./src/app/telemetry.o

C_DEPS += \
./src/app/app.d \
./src/app/task_faults.d \
./src/app/task_scpi.d \
./src/app/telemetry.d


# Each subdirectory must supply rules for building sources it contributes
src/app/%.o src/app/%.su src/app/%.cyclo: ../src/app/%.c src/app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../src/app -I../src/app/commands -I../src/app/control -I../src/app/protection -I../src/app/sim -I../src/bsp/stm32g4 -I../src/config -I../src/drivers -I../src/middleware/scpi -I../build/generated -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-app

clean-src-2f-app:
	-$(RM) ./src/app/app.cyclo ./src/app/app.d ./src/app/app.o ./src/app/app.su ./src/app/task_faults.cyclo ./src/app/task_faults.d ./src/app/task_faults.o ./src/app/task_faults.su ./src/app/task_scpi.cyclo ./src/app/task_scpi.d ./src/app/task_scpi.o ./src/app/task_scpi.su ./src/app/telemetry.cyclo ./src/app/telemetry.d ./src/app/telemetry.o ./src/app/telemetry.su

.PHONY: clean-src-2f-app

