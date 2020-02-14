################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modules/mDataManage.c \
../modules/mLibrary.c \
../modules/mMotorOut.c \
../modules/mtCommunication.c \
../modules/mtDecisionControl.c \
../modules/mtInputProcessing.c \
../modules/mtMonitoring.c \
../modules/mtMotorFeedback.c \
../modules/mtObstacleDetect.c 

OBJS += \
./modules/mDataManage.o \
./modules/mLibrary.o \
./modules/mMotorOut.o \
./modules/mtCommunication.o \
./modules/mtDecisionControl.o \
./modules/mtInputProcessing.o \
./modules/mtMonitoring.o \
./modules/mtMotorFeedback.o \
./modules/mtObstacleDetect.o 

C_DEPS += \
./modules/mDataManage.d \
./modules/mLibrary.d \
./modules/mMotorOut.d \
./modules/mtCommunication.d \
./modules/mtDecisionControl.d \
./modules/mtInputProcessing.d \
./modules/mtMonitoring.d \
./modules/mtMotorFeedback.d \
./modules/mtObstacleDetect.d 


# Each subdirectory must supply rules for building sources it contributes
modules/%.o: ../modules/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' -DDCU_LOGIC_TRACE -DTARGET_DEPENDENT_CODE -DHW_DEPENDENT_CODE '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F756xx -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Inc" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Drivers/CMSIS/Include" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/Administrator/Desktop/Wjjang/DCU Source Conv/[현차 F10 수정]korail128_sidedoor_v3_191014/korail128_sidedoor_v3_191014/korail128_sidedoor_v3/modules"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


