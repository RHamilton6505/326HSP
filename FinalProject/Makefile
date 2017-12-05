#===============================================================
#Makefile for building MSP Code Examples in command line
#environement using the GCC Open Source Compiler for MSP432
#===============================================================

# Require DEVICE to be specified
ifndef DEVICE
	DEVICE=MSP432P401R
endif

NAME := main

################################ FILE LOCATIONS ################################
OBJ_DIR			:= output
OS 					:= $(shell uname)
INSTALL_DIR			:= /Applications/ti/msp432_gcc
GCC_MSP_INC_DIR 	?= $(INSTALL_DIR)/arm/include
GCC_CMSIS_INC_DIR 	?= $(GCC_MSP_INC_DIR)/CMSIS
LDDIR				:= $(GCC_MSP_INC_DIR)/$(shell echo $(DEVICE) | tr A-Z a-z)
RM					:= rm -rf
SIMPLELINK_MSP432_SDK_INSTALL_DIR := /Applications/ti/simplelink_msp432p4_sdk_1_50_00_12

################################### BINARIES ###################################
GCC_BIN_DIR		?= $(INSTALL_DIR)/arm_compiler/bin/
GCC_INC_DIR		?= $(INSTALL_DIR)/arm_compiler/arm-none-eabi/include
CC              := $(GCC_BIN_DIR)arm-none-eabi-gcc
GDB			    := $(GCC_BIN_DIR)arm-none-eabi-gdb

############################## COMPILER SETTINGS ###############################
INCLUDES		:= "-I$(SIMPLELINK_MSP432_SDK_INSTALL_DIR)/source" "-I$(SIMPLELINK_MSP432_SDK_INSTALL_DIR)/source/ti/devices/msp432p4xx/driverlib" "-I$(SIMPLELINK_MSP432_SDK_INSTALL_DIR)/source/third_party/CMSIS/Include" "-I$(GCC_CMSIS_INC_DIR)" "-I$(GCC_MSP_INC_DIR)" "-I$(GCC_INC_DIR)"  "/Users/ianhamilton/ResourceLibrary/326/Final Project/326HSP/FinalProject" "/Applications/ti/simplelink_msp432p4_sdk_1_50_00_12/source/ti/devices/msp432p4xx/inc" 

CFLAGS			:= -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -D__$(DEVICE)__ -DTARGET_IS_MSP432P4XX -Dgcc -g -gstrict-dwarf -Wall -ffunction-sections -fdata-sections -MD -std=c99 -fno-builtin

############################### LINKER SETTINGS ################################
LDFLAGS			:= -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -D__$(DEVICE)__ -DTARGET_IS_MSP432P4XX -Dgcc -g -gstrict-dwarf -Wall -T$(LDDIR).lds -l'c' -l'gcc' -l'nosys' -specs='nosys.specs'

LFLAGS = -T$(LDDIR).lds "-Wl,-Map,$(NAME).map" "-L$(SIMPLELINK_MSP432_SDK_INSTALL_DIR)/source/ti/display/lib" -l:display.am4fg "-L$(SIMPLELINK_MSP432_SDK_INSTALL_DIR)/source/ti/drivers/lib" -l:drivers_msp432p401x.am4fg "-L$(SIMPLELINK_MSP432_SDK_INSTALL_DIR)/source/third_party/fatfs/lib" -l:fatfs.am4fg "-L$(SIMPLELINK_MSP432_SDK_INSTALL_DIR)/source/ti/devices/msp432p4xx/driverlib/gcc" -l:msp432p4xx_driverlib.a -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -static -Wl,--gc-sections "-L$(GCC_ARMCOMPILER)/arm-none-eabi/lib/thumb/v7e-m/fpv4-sp/hard" -lgcc -lc -lm -lnosys --specs=nano.specs

############################## DEBUGGER SETTINGS ###############################
GDBFLAGS        := -ex "target remote :55000"

OBJECTS			:= $(patsubst %.c,%.o,$(wildcard *.c))

all: $(NAME).out

%.o: %.c
	@echo ============================================
	@echo Generating $@
	$(CC) -c $< -o $@ $(CFLAGS) $(INCLUDES)

$(NAME).out: $(OBJECTS)
	@echo ============================================
	@echo Linking objects and generating output binary
	$(CC) $(OBJECTS) -o $@ $(LFLAGS) $(INCLUDES)

debug: all
	$(GDB) $(GDBFLAGS) $(NAME).out

clean:
	$(RM) *.o *.d $(NAME).out $(NAME).map
