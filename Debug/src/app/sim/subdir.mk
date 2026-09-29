################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/app/sim/sim_transrex.c

OBJS += \
./src/app/sim/sim_transrex.o

C_DEPS += \
./src/app/sim/sim_transrex.d


# Each subdirectory must supply rules for building sources it contributes
src/app/sim/%.o src/app/sim/%.su src/app/sim/%.cyclo: ../src/app/sim/%.c src/app/sim/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../src/app -I../src/app/commands -I../src/app/control -I../src/app/protection -I../src/app/sim -I../src/bsp/stm32g4 -I../src/config -I../src/drivers -I../src/middleware/scpi -I../build/generated -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-app-2f-sim

clean-src-2f-app-2f-sim:
	-$(RM) ./src/app/sim/sim_transrex.cyclo ./src/app/sim/sim_transrex.d ./src/app/sim/sim_transrex.o ./src/app/sim/sim_transrex.su

.PHONY: clean-src-2f-app-2f-sim

