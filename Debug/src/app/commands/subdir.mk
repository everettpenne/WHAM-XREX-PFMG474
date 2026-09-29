################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/app/commands/cmd_common.c \
../src/app/commands/cmd_config.c \
../src/app/commands/cmd_control.c \
../src/app/commands/cmd_fwupdate.c \
../src/app/commands/cmd_io.c \
../src/app/commands/cmd_pfmin.c \
../src/app/commands/cmd_shot.c \
../src/app/commands/cmd_sim.c \
../src/app/commands/cmd_state.c \
../src/app/commands/cmd_system.c \
../src/app/commands/cmd_table.c \
../src/app/commands/command_table.c

OBJS += \
./src/app/commands/cmd_common.o \
./src/app/commands/cmd_config.o \
./src/app/commands/cmd_control.o \
./src/app/commands/cmd_fwupdate.o \
./src/app/commands/cmd_io.o \
./src/app/commands/cmd_pfmin.o \
./src/app/commands/cmd_shot.o \
./src/app/commands/cmd_sim.o \
./src/app/commands/cmd_state.o \
./src/app/commands/cmd_system.o \
./src/app/commands/cmd_table.o \
./src/app/commands/command_table.o

C_DEPS += \
./src/app/commands/cmd_common.d \
./src/app/commands/cmd_config.d \
./src/app/commands/cmd_control.d \
./src/app/commands/cmd_fwupdate.d \
./src/app/commands/cmd_io.d \
./src/app/commands/cmd_pfmin.d \
./src/app/commands/cmd_shot.d \
./src/app/commands/cmd_sim.d \
./src/app/commands/cmd_state.d \
./src/app/commands/cmd_system.d \
./src/app/commands/cmd_table.d \
./src/app/commands/command_table.d


# Each subdirectory must supply rules for building sources it contributes
src/app/commands/%.o src/app/commands/%.su src/app/commands/%.cyclo: ../src/app/commands/%.c src/app/commands/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../src/app -I../src/app/commands -I../src/app/control -I../src/app/protection -I../src/app/sim -I../src/bsp/stm32g4 -I../src/config -I../src/drivers -I../src/middleware/scpi -I../build/generated -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-app-2f-commands

clean-src-2f-app-2f-commands:
	-$(RM) ./src/app/commands/cmd_common.cyclo ./src/app/commands/cmd_common.d ./src/app/commands/cmd_common.o ./src/app/commands/cmd_common.su ./src/app/commands/cmd_config.cyclo ./src/app/commands/cmd_config.d ./src/app/commands/cmd_config.o ./src/app/commands/cmd_config.su ./src/app/commands/cmd_control.cyclo ./src/app/commands/cmd_control.d ./src/app/commands/cmd_control.o ./src/app/commands/cmd_control.su ./src/app/commands/cmd_fwupdate.cyclo ./src/app/commands/cmd_fwupdate.d ./src/app/commands/cmd_fwupdate.o ./src/app/commands/cmd_fwupdate.su ./src/app/commands/cmd_io.cyclo ./src/app/commands/cmd_io.d ./src/app/commands/cmd_io.o ./src/app/commands/cmd_io.su ./src/app/commands/cmd_pfmin.cyclo ./src/app/commands/cmd_pfmin.d ./src/app/commands/cmd_pfmin.o ./src/app/commands/cmd_pfmin.su ./src/app/commands/cmd_shot.cyclo ./src/app/commands/cmd_shot.d ./src/app/commands/cmd_shot.o ./src/app/commands/cmd_shot.su ./src/app/commands/cmd_sim.cyclo ./src/app/commands/cmd_sim.d ./src/app/commands/cmd_sim.o ./src/app/commands/cmd_sim.su ./src/app/commands/cmd_state.cyclo ./src/app/commands/cmd_state.d ./src/app/commands/cmd_state.o ./src/app/commands/cmd_state.su ./src/app/commands/cmd_system.cyclo ./src/app/commands/cmd_system.d ./src/app/commands/cmd_system.o ./src/app/commands/cmd_system.su ./src/app/commands/cmd_table.cyclo ./src/app/commands/cmd_table.d ./src/app/commands/cmd_table.o ./src/app/commands/cmd_table.su ./src/app/commands/command_table.cyclo ./src/app/commands/command_table.d ./src/app/commands/command_table.o ./src/app/commands/command_table.su

.PHONY: clean-src-2f-app-2f-commands

