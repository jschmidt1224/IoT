#############################################################################
# Single board / simple build system
#############################################################################

# 1. All .c and .s files bellow SOURCE_DIR are compiled

#############################################################################
# Project information
#############################################################################
PROJECT_NAME=flowControl

#############################################################################
# Configuration variables
#############################################################################
OUTPUT_DIR=build
SOURCE_DIR=src
INCLUDE_DIR=include
TOOLS_DIR=tools

#############################################################################
# Compiler flags
#############################################################################
CFLAGS = -ggdb3 -O0 -Wall -W -Wextra
CFLAGS += -I$(INCLUDE_DIR)
CFLAGS += -mcpu=cortex-m4 -mthumb -ffunction-sections -fdata-sections
CFLAGS += -mlittle-endian -DSTM32F407xx
CFLAGS += -Werror -Wendif-labels

LINKER_SCRIPT = linker_script.ld
LDFLAGS = -T $(LINKER_SCRIPT)
LDFLAGS += -Wl,--gc-sections

MAKEFLAGS += --no-builtin-rules

#############################################################################
# Toolchain
#############################################################################
ARCH=arm-none-eabi-

CC=$(ARCH)gcc
CPP=$(ARCH)cpp
GDB=$(ARCH)gdb
OBJDUMP=$(ARCH)objdump
OBJCOPY=$(ARCH)objcopy
SIZE=$(ARCH)size

#############################################################################
#
#############################################################################

SOURCE_FILES_C = $(shell find $(SOURCE_DIR) -name "*.c")
SOURCE_FILES_ASM = $(shell find $(SOURCE_DIR) -name "*.s")

OBJS_C = $(addprefix $(OUTPUT_DIR)/, $(SOURCE_FILES_C:%.c=%.o))
OBJS_ASM = $(addprefix $(OUTPUT_DIR)/, $(SOURCE_FILES_ASM:%.s=%.o))
OBJS := $(OBJS_C) $(OBJS_ASM)
AUTODEP_FILES = $(OBJS:%.o=%.d)

#############################################################################
# Rules
#############################################################################

all: bin hex md5

bin: $(OUTPUT_DIR)/$(PROJECT_NAME).bin

hex: $(OUTPUT_DIR)/$(PROJECT_NAME).hex

elf: $(OUTPUT_DIR)/$(PROJECT_NAME).elf

md5: $(OUTPUT_DIR)/$(PROJECT_NAME).bin
	@md5sum $<

$(OUTPUT_DIR)/$(PROJECT_NAME).elf: $(OBJS) $(LINKER_SCRIPT)
	@echo [LD] $(PROJECT_NAME)
	@$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) \
		-Wl,-Map,$(OUTPUT_DIR)/$(PROJECT_NAME).map -o $@ $(OBJS)
	@$(OBJDUMP) -S -d $@ > $(OUTPUT_DIR)/$(PROJECT_NAME).lst
	@$(SIZE) $@

$(OUTPUT_DIR)/$(PROJECT_NAME).bin: $(OUTPUT_DIR)/$(PROJECT_NAME).elf
	@$(OBJCOPY) -O binary $< $@

$(OUTPUT_DIR)/$(PROJECT_NAME).hex: $(OUTPUT_DIR)/$(PROJECT_NAME).elf
	@$(OBJCOPY) -O ihex $< $@


.PHONY: debug
debug: $(OUTPUT_DIR)/$(PROJECT_NAME).elf
	@$(GDB) $< -x $(TOOLS_DIR)/load_halt.gdb

.PHONY: flash
flash: $(OUTPUT_DIR)/$(PROJECT_NAME).elf
	@$(GDB) $< -x $(TOOLS_DIR)/flash.gdb -batch

.PHONY: clean
clean:
	@echo [CLEAN]
	@rm -rf $(OUTPUT_DIR)

#############################################################################
# Auto dependency rules
#############################################################################

-include $(AUTODEP_FILES)

#############################################################################
# Suffix Rules
#############################################################################

$(OBJS_C): $(OUTPUT_DIR)/%.o : %.c
	@echo [CC] $<
	@mkdir -p $(dir $@)
	@$(CC) -MMD $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

$(OBJS_ASM): $(OUTPUT_DIR)/%.o : %.s
	@echo [CC] $<
	@mkdir -p $(dir $@)
	@$(CC) -MMD $(CFLAGS) $(CPPFLAGS) -c -o $@ $<
