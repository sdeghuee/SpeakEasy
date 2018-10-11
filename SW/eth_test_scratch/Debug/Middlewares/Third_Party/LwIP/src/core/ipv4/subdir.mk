################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/LwIP/src/core/ipv4/autoip.c \
../Middlewares/Third_Party/LwIP/src/core/ipv4/dhcp.c \
../Middlewares/Third_Party/LwIP/src/core/ipv4/etharp.c \
../Middlewares/Third_Party/LwIP/src/core/ipv4/icmp.c \
../Middlewares/Third_Party/LwIP/src/core/ipv4/igmp.c \
../Middlewares/Third_Party/LwIP/src/core/ipv4/ip4.c \
../Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_addr.c \
../Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_frag.c 

OBJS += \
./Middlewares/Third_Party/LwIP/src/core/ipv4/autoip.o \
./Middlewares/Third_Party/LwIP/src/core/ipv4/dhcp.o \
./Middlewares/Third_Party/LwIP/src/core/ipv4/etharp.o \
./Middlewares/Third_Party/LwIP/src/core/ipv4/icmp.o \
./Middlewares/Third_Party/LwIP/src/core/ipv4/igmp.o \
./Middlewares/Third_Party/LwIP/src/core/ipv4/ip4.o \
./Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_addr.o \
./Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_frag.o 

C_DEPS += \
./Middlewares/Third_Party/LwIP/src/core/ipv4/autoip.d \
./Middlewares/Third_Party/LwIP/src/core/ipv4/dhcp.d \
./Middlewares/Third_Party/LwIP/src/core/ipv4/etharp.d \
./Middlewares/Third_Party/LwIP/src/core/ipv4/icmp.d \
./Middlewares/Third_Party/LwIP/src/core/ipv4/igmp.d \
./Middlewares/Third_Party/LwIP/src/core/ipv4/ip4.d \
./Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_addr.d \
./Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_frag.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/LwIP/src/core/ipv4/%.o: ../Middlewares/Third_Party/LwIP/src/core/ipv4/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"U:/Personal/eclipse-workspace/eth_test_scratch/Inc" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/system" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Drivers/STM32F4xx_HAL_Driver/Inc" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/lwip" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/netif" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/posix" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Middlewares/Third_Party/LwIP/system/arch" -I"U:/Personal/eclipse-workspace/eth_test_scratch/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


