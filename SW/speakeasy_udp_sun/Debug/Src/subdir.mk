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
../Src/udp_sun.c 

OBJS += \
./Src/ethernetif.o \
./Src/lwip.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o \
./Src/udp_sun.o 

C_DEPS += \
./Src/ethernetif.d \
./Src/lwip.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d \
./Src/udp_sun.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F407xx '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Inc" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/system" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/lwip" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/netif" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/posix" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/src/include/posix/sys" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Middlewares/Third_Party/LwIP/system/arch" -I"/home/david/Documents/eclipse-workspace/speakeasy_udp_sun/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


