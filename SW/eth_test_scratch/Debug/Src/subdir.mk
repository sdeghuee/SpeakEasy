################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ethernetif.c \
../Src/lwip.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c \
../Src/udp_scratch.c 

OBJS += \
./Src/ethernetif.o \
./Src/lwip.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o \
./Src/udp_scratch.o 

C_DEPS += \
./Src/ethernetif.d \
./Src/lwip.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d \
./Src/udp_scratch.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"U:/Personal/eclipse-workspace/eth_test_scratch/Inc" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/system" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Drivers/STM32F4xx_HAL_Driver/Inc" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/lwip" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/netif" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/posix" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/system/arch" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


