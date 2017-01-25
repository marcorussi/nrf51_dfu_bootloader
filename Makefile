# Copyright (c) 2016 BlueMaestro. All Rights Reserved.
#
# The information contained herein is property of BlueMaestro Ltd.
# Not to be used, copied or distributed without the permission of 
# Blue Maestro Limited.
#
# The above copyright notice and this permission notice shall be 
# included in all copies or substantial portions of the Software.

#ATTENTON: comment the following define for optimized release firmware; not suitable for debug
#DEBUG := 1

#ATTENTON: modify following names and paths as required
PROJECT_NAME := nrf51_dfu
NRFJPROG_PATH := ./tools/
SDK_PATH := ../nrf51_sdk_dependencies
LINKER_SCRIPT := gcc_nrf51_dfu.ld
GNU_INSTALL_ROOT := /home/marco/ARMToolchain/gcc-arm-none-eabi-4_9-2015q2
GNU_VERSION := 4.9.3
GNU_PREFIX := arm-none-eabi

export OUTPUT_FILENAME

SDK_COMPONENTS_PATH = $(SDK_PATH)/components
TEMPLATE_PATH = $(SDK_COMPONENTS_PATH)/toolchain/gcc
OUTPUT_FILENAME = $(PROJECT_NAME)_s110_pca10028

MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE    		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"
GDB       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gdb"

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(abspath ../../../dfu_ble_svc.c) \
$(abspath ../../../main.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/util/app_error.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/util/app_error_weak.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/scheduler/app_scheduler.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/timer/app_timer.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/timer/app_timer_appsh.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/util/app_util_platform.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/bootloader_dfu/bootloader.c) \
$(abspath $(SDK_COMPONENTS_PATH)s/libraries/bootloader_dfu/bootloader_settings.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/bootloader_dfu/bootloader_util.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/crc16/crc16.c) \
$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/wdt/nrf_drv_wdt.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/bootloader_dfu/dfu_dual_bank.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/bootloader_dfu/dfu_init_template.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/bootloader_dfu/dfu_transport_ble.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/hci/hci_mem_pool.c) \
$(abspath $(SDK_COMPONENTS_PATH)/libraries/util/nrf_assert.c) \
$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/delay/nrf_delay.c) \
$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/common/nrf_drv_common.c) \
$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/pstorage/pstorage_raw.c) \
$(abspath $(SDK_COMPONENTS_PATH)/ble/common/ble_advdata.c) \
$(abspath $(SDK_COMPONENTS_PATH)/ble/common/ble_conn_params.c) \
$(abspath $(SDK_COMPONENTS_PATH)/ble/ble_services/ble_dfu/ble_dfu.c) \
$(abspath $(SDK_COMPONENTS_PATH)/ble/common/ble_srv_common.c) \
$(abspath $(SDK_COMPONENTS_PATH)/toolchain/system_nrf51.c) \
$(abspath $(SDK_COMPONENTS_PATH)/softdevice/common/softdevice_handler/softdevice_handler.c) \
$(abspath $(SDK_COMPONENTS_PATH)/softdevice/common/softdevice_handler/softdevice_handler_appsh.c) \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath $(SDK_COMPONENTS_PATH)/toolchain/gcc/gcc_startup_nrf51.s)

#includes common to all targets
INC_PATHS += -I$(abspath config)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/util)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/toolchain/gcc)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/toolchain)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/ble/common)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/scheduler)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/softdevice/s130/headers)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/pstorage)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/bootloader_dfu/ble_transport)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/softdevice/common/softdevice_handler)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/device)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/wdt)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/hci)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/toolchain/CMSIS/Include)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/delay)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/crc16)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/common)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/ble/ble_services/ble_dfu)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/timer)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/drivers_nrf/hal)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/softdevice/s130/headers/nrf51)
INC_PATHS += -I$(abspath $(SDK_COMPONENTS_PATH)/libraries/bootloader_dfu)

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets  
CFLAGS  = -DSWI_DISABLE0
CFLAGS += -DBOARD_PCA10028
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS130
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -Os
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums
# build for debugging if needed
ifdef DEBUG
CFLAGS += -g -O0
endif

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags   
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS130
ASMFLAGS += -D__HEAP_SIZE=0
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DBSP_DEFINES_ONLY
#default target - first one defined
default: clean nrf51422_preset

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_preset

#target for printing all targets
help:
	@echo - following targets are available:
	@echo 	nrf51422_xxac_s110
	@echo 	flash_softdevice
	@echo - following main commands are available:
	@echo 	clean: clean _build directory
	@echo 	cleanobj: clean object files
	@echo 	flash: download application firmware into device
	@echo 	erase: erase all flash memory device
	@echo 	debug_init: init debug and start JLink GDB server
	@echo 	debug_start: start GDB client and launch debug with gdb_cmd command file
	@echo 	memwr "add=<address_hex>" "val=<value_hex_4bytes>": write 4 bytes to a flash memory address
	@echo 	flash_softdevice: download s110 softdevice firmware into device


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51422_preset: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genhex echosize

## ATTENTION: genbin call removed from finalize step. The resulting .bin file was too big...
genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
	$(NRFJPROG_PATH)/nrfjprog.sh --flash $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

erase: $(MAKECMDGOALS)
	@echo Erasing device...
	$(NRFJPROG_PATH)/nrfjprog.sh --erase-all

debug_init: $(MAKECMDGOALS)
	@echo Debugging init: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NRFJPROG_PATH)/nrfjprog.sh --debug $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out

debug_start: $(MAKECMDGOALS)
	@echo Debugging start: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo "symbol-file _build/$(OUTPUT_FILENAME).out" > gdb_cmd
	@echo "target remote localhost:2331" >> gdb_cmd
	@echo "break main.c:179" >> gdb_cmd
	@echo "continue" >> gdb_cmd
	@echo "p /x adv_data" >> gdb_cmd
	$(GDB) -x gdb_cmd

## ATTENTION: erase all before writing to a memory register such as a UICR register
memwr: $(MAKECMDGOALS)
	@echo Writing memory...
	$(NRFJPROG_PATH)/nrfjprog.sh --write $(add) $(val)

## Flash softdevice
flash_softdevice: 
	@echo Flashing: s110_nrf51_2.0.0_softdevice.hex
	$(NRFJPROG_PATH)/nrfjprog.sh --flash-softdevice $(SDK_COMPONENTS_PATH)/softdevice/s130/hex/s130_nrf51_2.0.0_softdevice.hex
