################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/hal/src/context_switch.s 

C_SRCS += \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/hal/src/clock.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/hal/src/gp_timer.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/hal/src/miscutil.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/hal/src/osal.c 

OBJS += \
./HAL/clock.o \
./HAL/context_switch.o \
./HAL/gp_timer.o \
./HAL/miscutil.o \
./HAL/osal.o 

S_DEPS += \
./HAL/context_switch.d 

C_DEPS += \
./HAL/clock.d \
./HAL/gp_timer.d \
./HAL/miscutil.d \
./HAL/osal.d 


# Each subdirectory must supply rules for building sources it contributes
HAL/clock.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/hal/src/clock.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"HAL/clock.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

HAL/context_switch.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/hal/src/context_switch.s
	@echo 'Building file: $<'
	@echo 'Invoking: GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -x assembler-with-cpp -MMD -MP -MF"HAL/context_switch.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

HAL/gp_timer.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/hal/src/gp_timer.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"HAL/gp_timer.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

HAL/miscutil.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/hal/src/miscutil.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"HAL/miscutil.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

HAL/osal.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/hal/src/osal.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"HAL/osal.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


