################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/freertos.c \
../Src/main.c \
../Src/ring_buffer.c \
../Src/stm32f7xx_hal_msp.c \
../Src/stm32f7xx_hal_timebase_TIM.c \
../Src/stm32f7xx_it.c \
../Src/system_stm32f7xx.c 

OBJS += \
./Src/freertos.o \
./Src/main.o \
./Src/ring_buffer.o \
./Src/stm32f7xx_hal_msp.o \
./Src/stm32f7xx_hal_timebase_TIM.o \
./Src/stm32f7xx_it.o \
./Src/system_stm32f7xx.o 

C_DEPS += \
./Src/freertos.d \
./Src/main.d \
./Src/ring_buffer.d \
./Src/stm32f7xx_hal_msp.d \
./Src/stm32f7xx_hal_timebase_TIM.d \
./Src/stm32f7xx_it.d \
./Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' -DDCU_LOGIC_TRACE -DTARGET_DEPENDENT_CODE -DHW_DEPENDENT_CODE '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F756xx -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Inc" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Drivers/CMSIS/Include" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/modules"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


