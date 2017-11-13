################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
ST7735.obj: ../ST7735.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP432 Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/driverlib" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/inc" --include_path="/Users/ianhamilton/ResourceLibrary/326/Final Project/326HSP" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/inc" --include_path="/Users/ianhamilton/ResourceLibrary/326/Final Project/326HSP/FinalProject" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/third_party/CMSIS/Include" --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=__MSP432P401R__ --define=DeviceFamily_MSP432P401x -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="ST7735.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Splash.obj: ../Splash.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP432 Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/driverlib" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/inc" --include_path="/Users/ianhamilton/ResourceLibrary/326/Final Project/326HSP" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/inc" --include_path="/Users/ianhamilton/ResourceLibrary/326/Final Project/326HSP/FinalProject" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/third_party/CMSIS/Include" --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=__MSP432P401R__ --define=DeviceFamily_MSP432P401x -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="Splash.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

main.obj: ../main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP432 Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/driverlib" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/inc" --include_path="/Users/ianhamilton/ResourceLibrary/326/Final Project/326HSP" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/inc" --include_path="/Users/ianhamilton/ResourceLibrary/326/Final Project/326HSP/FinalProject" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/third_party/CMSIS/Include" --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=__MSP432P401R__ --define=DeviceFamily_MSP432P401x -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="main.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

system_msp432p401r.obj: ../system_msp432p401r.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP432 Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/driverlib" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/inc" --include_path="/Users/ianhamilton/ResourceLibrary/326/Final Project/326HSP" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/inc" --include_path="/Users/ianhamilton/ResourceLibrary/326/Final Project/326HSP/FinalProject" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source" --include_path="/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/third_party/CMSIS/Include" --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --define=__MSP432P401R__ --define=DeviceFamily_MSP432P401x -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="system_msp432p401r.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


