################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' -DDCU_LOGIC_TRACE -DTARGET_DEPENDENT_CODE -DHW_DEPENDENT_CODE '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F756xx -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Inc" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Drivers/CMSIS/Include" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/modules"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


