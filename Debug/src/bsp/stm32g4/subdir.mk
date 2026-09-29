################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/bsp/stm32g4/board_io.c \
../src/bsp/stm32g4/boot_diag.c \
../src/bsp/stm32g4/boot_jump.c \
../src/bsp/stm32g4/flash_bank.c \
../src/bsp/stm32g4/hrtim.c \
../src/bsp/stm32g4/mcu.c \
../src/bsp/stm32g4/pfm_input.c \
../src/bsp/stm32g4/qspi_test.c \
../src/bsp/stm32g4/uart.c

OBJS += \
./src/bsp/stm32g4/board_io.o \
./src/bsp/stm32g4/boot_diag.o \
./src/bsp/stm32g4/boot_jump.o \
./src/bsp/stm32g4/flash_bank.o \
./src/bsp/stm32g4/hrtim.o \
./src/bsp/stm32g4/mcu.o \
./src/bsp/stm32g4/pfm_input.o \
./src/bsp/stm32g4/qspi_test.o \
./src/bsp/stm32g4/uart.o

C_DEPS += \
./src/bsp/stm32g4/board_io.d \
./src/bsp/stm32g4/boot_diag.d \
./src/bsp/stm32g4/boot_jump.d \
./src/bsp/stm32g4/flash_bank.d \
./src/bsp/stm32g4/hrtim.d \
./src/bsp/stm32g4/mcu.d \
./src/bsp/stm32g4/pfm_input.d \
./src/bsp/stm32g4/qspi_test.d \
./src/bsp/stm32g4/uart.d


# Each subdirectory must supply rules for building sources it contributes
src/bsp/stm32g4/%.o src/bsp/stm32g4/%.su src/bsp/stm32g4/%.cyclo: ../src/bsp/stm32g4/%.c src/bsp/stm32g4/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../src/app -I../src/app/commands -I../src/app/control -I../src/app/protection -I../src/app/sim -I../src/bsp/stm32g4 -I../src/config -I../src/drivers -I../src/middleware/scpi -I../build/generated -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-bsp-2f-stm32g4

clean-src-2f-bsp-2f-stm32g4:
	-$(RM) ./src/bsp/stm32g4/board_io.cyclo ./src/bsp/stm32g4/board_io.d ./src/bsp/stm32g4/board_io.o ./src/bsp/stm32g4/board_io.su ./src/bsp/stm32g4/boot_diag.cyclo ./src/bsp/stm32g4/boot_diag.d ./src/bsp/stm32g4/boot_diag.o ./src/bsp/stm32g4/boot_diag.su ./src/bsp/stm32g4/boot_jump.cyclo ./src/bsp/stm32g4/boot_jump.d ./src/bsp/stm32g4/boot_jump.o ./src/bsp/stm32g4/boot_jump.su ./src/bsp/stm32g4/flash_bank.cyclo ./src/bsp/stm32g4/flash_bank.d ./src/bsp/stm32g4/flash_bank.o ./src/bsp/stm32g4/flash_bank.su ./src/bsp/stm32g4/hrtim.cyclo ./src/bsp/stm32g4/hrtim.d ./src/bsp/stm32g4/hrtim.o ./src/bsp/stm32g4/hrtim.su ./src/bsp/stm32g4/mcu.cyclo ./src/bsp/stm32g4/mcu.d ./src/bsp/stm32g4/mcu.o ./src/bsp/stm32g4/mcu.su ./src/bsp/stm32g4/pfm_input.cyclo ./src/bsp/stm32g4/pfm_input.d ./src/bsp/stm32g4/pfm_input.o ./src/bsp/stm32g4/pfm_input.su ./src/bsp/stm32g4/qspi_test.cyclo ./src/bsp/stm32g4/qspi_test.d ./src/bsp/stm32g4/qspi_test.o ./src/bsp/stm32g4/qspi_test.su ./src/bsp/stm32g4/uart.cyclo ./src/bsp/stm32g4/uart.d ./src/bsp/stm32g4/uart.o ./src/bsp/stm32g4/uart.su

.PHONY: clean-src-2f-bsp-2f-stm32g4

