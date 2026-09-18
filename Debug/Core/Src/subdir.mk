################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/boot_jump.c \
../Core/Src/cmd_parser.c \
../Core/Src/commands.c \
../Core/Src/gate_driver.c \
../Core/Src/hrtim.c \
../Core/Src/main.c \
../Core/Src/pfm.c \
../Core/Src/pfm_input.c \
../Core/Src/pid.c \
../Core/Src/qspi_test.c \
../Core/Src/sim_transrex.c \
../Core/Src/state_machine.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/uart.c \
../Core/Src/xrex_io.c

OBJS += \
./Core/Src/boot_jump.o \
./Core/Src/cmd_parser.o \
./Core/Src/commands.o \
./Core/Src/gate_driver.o \
./Core/Src/hrtim.o \
./Core/Src/main.o \
./Core/Src/pfm.o \
./Core/Src/pfm_input.o \
./Core/Src/pid.o \
./Core/Src/qspi_test.o \
./Core/Src/sim_transrex.o \
./Core/Src/state_machine.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/uart.o \
./Core/Src/xrex_io.o

C_DEPS += \
./Core/Src/boot_jump.d \
./Core/Src/cmd_parser.d \
./Core/Src/commands.d \
./Core/Src/gate_driver.d \
./Core/Src/hrtim.d \
./Core/Src/main.d \
./Core/Src/pfm.d \
./Core/Src/pfm_input.d \
./Core/Src/pid.d \
./Core/Src/qspi_test.d \
./Core/Src/sim_transrex.d \
./Core/Src/state_machine.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/uart.d \
./Core/Src/xrex_io.d


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/boot_jump.cyclo ./Core/Src/boot_jump.d ./Core/Src/boot_jump.o ./Core/Src/boot_jump.su ./Core/Src/cmd_parser.cyclo ./Core/Src/cmd_parser.d ./Core/Src/cmd_parser.o ./Core/Src/cmd_parser.su ./Core/Src/commands.cyclo ./Core/Src/commands.d ./Core/Src/commands.o ./Core/Src/commands.su ./Core/Src/gate_driver.cyclo ./Core/Src/gate_driver.d ./Core/Src/gate_driver.o ./Core/Src/gate_driver.su ./Core/Src/hrtim.cyclo ./Core/Src/hrtim.d ./Core/Src/hrtim.o ./Core/Src/hrtim.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pfm.cyclo ./Core/Src/pfm.d ./Core/Src/pfm.o ./Core/Src/pfm.su ./Core/Src/pfm_input.cyclo ./Core/Src/pfm_input.d ./Core/Src/pfm_input.o ./Core/Src/pfm_input.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/qspi_test.cyclo ./Core/Src/qspi_test.d ./Core/Src/qspi_test.o ./Core/Src/qspi_test.su ./Core/Src/sim_transrex.cyclo ./Core/Src/sim_transrex.d ./Core/Src/sim_transrex.o ./Core/Src/sim_transrex.su ./Core/Src/state_machine.cyclo ./Core/Src/state_machine.d ./Core/Src/state_machine.o ./Core/Src/state_machine.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/uart.cyclo ./Core/Src/uart.d ./Core/Src/uart.o ./Core/Src/uart.su ./Core/Src/xrex_io.cyclo ./Core/Src/xrex_io.d ./Core/Src/xrex_io.o ./Core/Src/xrex_io.su

.PHONY: clean-Core-2f-Src

