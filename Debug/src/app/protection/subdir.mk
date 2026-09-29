################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/app/protection/gate_driver.c \
../src/app/protection/xrex_io.c

OBJS += \
./src/app/protection/gate_driver.o \
./src/app/protection/xrex_io.o

C_DEPS += \
./src/app/protection/gate_driver.d \
./src/app/protection/xrex_io.d


# Each subdirectory must supply rules for building sources it contributes
src/app/protection/%.o src/app/protection/%.su src/app/protection/%.cyclo: ../src/app/protection/%.c src/app/protection/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../src/app -I../src/app/commands -I../src/app/control -I../src/app/protection -I../src/app/sim -I../src/bsp/stm32g4 -I../src/config -I../src/drivers -I../src/middleware/scpi -I../build/generated -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-app-2f-protection

clean-src-2f-app-2f-protection:
	-$(RM) ./src/app/protection/gate_driver.cyclo ./src/app/protection/gate_driver.d ./src/app/protection/gate_driver.o ./src/app/protection/gate_driver.su ./src/app/protection/xrex_io.cyclo ./src/app/protection/xrex_io.d ./src/app/protection/xrex_io.o ./src/app/protection/xrex_io.su

.PHONY: clean-src-2f-app-2f-protection

