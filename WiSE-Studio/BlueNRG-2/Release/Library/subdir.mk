################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_adc.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_dma.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_flash.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_gpio.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_i2c.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_mft.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_pka.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_rtc.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_spi.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_sysCtrl.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_uart.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_wdg.c \
D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/misc.c 

OBJS += \
./Library/BlueNRG1_adc.o \
./Library/BlueNRG1_dma.o \
./Library/BlueNRG1_flash.o \
./Library/BlueNRG1_gpio.o \
./Library/BlueNRG1_i2c.o \
./Library/BlueNRG1_mft.o \
./Library/BlueNRG1_pka.o \
./Library/BlueNRG1_rtc.o \
./Library/BlueNRG1_spi.o \
./Library/BlueNRG1_sysCtrl.o \
./Library/BlueNRG1_uart.o \
./Library/BlueNRG1_wdg.o \
./Library/misc.o 

C_DEPS += \
./Library/BlueNRG1_adc.d \
./Library/BlueNRG1_dma.d \
./Library/BlueNRG1_flash.d \
./Library/BlueNRG1_gpio.d \
./Library/BlueNRG1_i2c.d \
./Library/BlueNRG1_mft.d \
./Library/BlueNRG1_pka.d \
./Library/BlueNRG1_rtc.d \
./Library/BlueNRG1_spi.d \
./Library/BlueNRG1_sysCtrl.d \
./Library/BlueNRG1_uart.d \
./Library/BlueNRG1_wdg.d \
./Library/misc.d 


# Each subdirectory must supply rules for building sources it contributes
Library/BlueNRG1_adc.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_adc.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/BlueNRG1_adc.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/BlueNRG1_dma.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_dma.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/BlueNRG1_dma.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/BlueNRG1_flash.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_flash.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/BlueNRG1_flash.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/BlueNRG1_gpio.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_gpio.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/BlueNRG1_gpio.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/BlueNRG1_i2c.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_i2c.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/BlueNRG1_i2c.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/BlueNRG1_mft.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_mft.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/BlueNRG1_mft.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/BlueNRG1_pka.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_pka.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/BlueNRG1_pka.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/BlueNRG1_rtc.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_rtc.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/BlueNRG1_rtc.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/BlueNRG1_spi.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_spi.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/BlueNRG1_spi.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/BlueNRG1_sysCtrl.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_sysCtrl.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/BlueNRG1_sysCtrl.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/BlueNRG1_uart.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_uart.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/BlueNRG1_uart.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/BlueNRG1_wdg.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/BlueNRG1_wdg.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/BlueNRG1_wdg.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Library/misc.o: D:/ST_wise_studio/BlueNRG-1_2\ DK\ 3.2.3_Slave/Library/BlueNRG1_Periph_Driver/src/misc.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -O0 -ffunction-sections -fdata-sections -g3 -std=c99 -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION -DBLUENRG2_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ -DUSER_BUTTON=BUTTON_1 -I../../../inc -I../../../../../../Library/hal/inc -I../../../../../../Library/BlueNRG1_Periph_Driver/inc -I../../../../../../Library/CMSIS/Device/ST/BlueNRG1/Include -I../../../../../../Library/CMSIS/Include -I../../../../../../Library/Bluetooth_LE/inc -I../../../../../../Library/BLE_Application/layers_inc -I../../../../../../Library/BLE_Application/Utils/inc -I../../../../../../Library/SDK_Eval_BlueNRG1/inc -MMD -MP -MF"Library/misc.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


