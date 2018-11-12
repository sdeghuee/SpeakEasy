################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/LwIP/src/core/ipv6/dhcp6.c \
../Middlewares/Third_Party/LwIP/src/core/ipv6/ethip6.c \
../Middlewares/Third_Party/LwIP/src/core/ipv6/icmp6.c \
../Middlewares/Third_Party/LwIP/src/core/ipv6/inet6.c \
../Middlewares/Third_Party/LwIP/src/core/ipv6/ip6.c \
../Middlewares/Third_Party/LwIP/src/core/ipv6/ip6_addr.c \
../Middlewares/Third_Party/LwIP/src/core/ipv6/ip6_frag.c \
../Middlewares/Third_Party/LwIP/src/core/ipv6/mld6.c \
../Middlewares/Third_Party/LwIP/src/core/ipv6/nd6.c 

OBJS += \
./Middlewares/Third_Party/LwIP/src/core/ipv6/dhcp6.o \
./Middlewares/Third_Party/LwIP/src/core/ipv6/ethip6.o \
./Middlewares/Third_Party/LwIP/src/core/ipv6/icmp6.o \
./Middlewares/Third_Party/LwIP/src/core/ipv6/inet6.o \
./Middlewares/Third_Party/LwIP/src/core/ipv6/ip6.o \
./Middlewares/Third_Party/LwIP/src/core/ipv6/ip6_addr.o \
./Middlewares/Third_Party/LwIP/src/core/ipv6/ip6_frag.o \
./Middlewares/Third_Party/LwIP/src/core/ipv6/mld6.o \
./Middlewares/Third_Party/LwIP/src/core/ipv6/nd6.o 

C_DEPS += \
./Middlewares/Third_Party/LwIP/src/core/ipv6/dhcp6.d \
./Middlewares/Third_Party/LwIP/src/core/ipv6/ethip6.d \
./Middlewares/Third_Party/LwIP/src/core/ipv6/icmp6.d \
./Middlewares/Third_Party/LwIP/src/core/ipv6/inet6.d \
./Middlewares/Third_Party/LwIP/src/core/ipv6/ip6.d \
./Middlewares/Third_Party/LwIP/src/core/ipv6/ip6_addr.d \
./Middlewares/Third_Party/LwIP/src/core/ipv6/ip6_frag.d \
./Middlewares/Third_Party/LwIP/src/core/ipv6/mld6.d \
./Middlewares/Third_Party/LwIP/src/core/ipv6/nd6.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/LwIP/src/core/ipv6/%.o: ../Middlewares/Third_Party/LwIP/src/core/ipv6/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F407xx '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Inc" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/system" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/lwip" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/netif" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/posix" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/system/arch" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


