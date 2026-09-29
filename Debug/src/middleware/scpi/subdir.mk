################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/middleware/scpi/scpi_parser.c

OBJS += \
./src/middleware/scpi/scpi_parser.o

C_DEPS += \
./src/middleware/scpi/scpi_parser.d


# Each subdirectory must supply rules for building sources it contributes
src/middleware/scpi/%.o src/middleware/scpi/%.su src/middleware/scpi/%.cyclo: ../src/middleware/scpi/%.c src/middleware/scpi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../src/app -I../src/app/commands -I../src/app/control -I../src/app/protection -I../src/app/sim -I../src/bsp/stm32g4 -I../src/config -I../src/drivers -I../src/middleware/scpi -I../build/generated -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-src-2f-middleware-2f-scpi

clean-src-2f-middleware-2f-scpi:
	-$(RM) ./src/middleware/scpi/scpi_parser.cyclo ./src/middleware/scpi/scpi_parser.d ./src/middleware/scpi/scpi_parser.o ./src/middleware/scpi/scpi_parser.su

.PHONY: clean-src-2f-middleware-2f-scpi

