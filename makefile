################################################################################
# chillpill-firmware — single-file GNU Make build
# Works with your current layout (Core/Inc/... and Drivers/...)
# NOTE: Paths with spaces are handled; keeping names without spaces is cleaner.
################################################################################

# Project
PROJECT := PLU

# Toolchain
CC      := arm-none-eabi-gcc
AS      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size
RM      := rm -rf
MKDIR   := mkdir -p

# MCU / common flags
MCU     := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
DEFS    := -DUSE_HAL_DRIVER -DSTM32F103xB
OPT     := -O2 -g3
WARN    := -Wall -Wextra -Werror=implicit-function-declaration

CSTD    := -std=gnu11

CFLAGS  := $(MCU) $(OPT) $(WARN) $(DEFS) $(CSTD) -ffunction-sections -fdata-sections -MMD -MP
ASFLAGS := $(MCU) -x assembler-with-cpp

# Linker
LD_SCRIPT := Core/STM32F103C8TX_FLASH.ld
LDFLAGS   := $(MCU) -T"$(LD_SCRIPT)" --specs=nosys.specs --specs=nano.specs -static \
             -Wl,-Map="$(PROJECT).map",--gc-sections -u _printf_float -u _scanf_float
LDLIBS    := -Wl,--start-group -lc -lm -Wl,--end-group

# Output
OUT_ELF := $(PROJECT).elf
OUT_HEX := $(PROJECT).hex
OUT_BIN := $(PROJECT).bin
OUT_LST := $(PROJECT).list

# Build dir (mirrors source tree)
BUILD := build

# ---------------------------------------------------------------------------
# Source & include paths (quoted to tolerate spaces)
# If you rename folders to not have spaces, you can simplify this list.
INC_DIRS := \
 "Core/Inc" \
 "Core/Inc/Core Files" \
 "Core/Inc/Control FSM Files" \
 "Core/Inc/Foundation Files" \
 "Drivers/STM32F1xx_HAL_Driver/Inc" \
 "Drivers/CMSIS/Include" \
 "Drivers/CMSIS/Device/ST/STM32F1xx/Include"

# Add -I for each include dir
CFLAGS += $(addprefix -I, $(INC_DIRS))

# Directories to scan for sources
SRC_DIRS := \
 "Core/Src" \
 "Core/Src/Core Files" \
 "Core/Src/Control FSM Files" \
 "Core/Src/Foundation Files" \
 "Core/Startup" \
 "Drivers/STM32F1xx_HAL_Driver/Src"

# Find all C and ASM sources
# (We quote dirs so 'find' copes with spaces.)
SRC_C := $(shell find $(SRC_DIRS) -type f -name '*.c' 2>/dev/null)
SRC_S := $(shell find $(SRC_DIRS) -type f -name '*.s' -o -name '*.S' 2>/dev/null)

# Convert "path/to/file.c" -> "build/path/to/file.o"
OBJ_C := $(patsubst %.c, $(BUILD)/%.o, $(SRC_C))
OBJ_S := $(patsubst %.s, $(BUILD)/%.o, $(patsubst %.S, $(BUILD)/%.o, $(SRC_S)))
OBJS  := $(OBJ_C) $(OBJ_S)

DEPS  := $(OBJS:.o=.d)

# ---------------------------------------------------------------------------
# Top-level targets
.PHONY: all clean size list hex bin

all: $(OUT_ELF) hex bin list size

$(OUT_ELF): $(OBJS) $(LD_SCRIPT)
	@echo "Linking $(OUT_ELF)"
	$(CC) -o $@ $(OBJS) $(LDFLAGS) $(LDLIBS)

hex: $(OUT_HEX)
bin: $(OUT_BIN)
list: $(OUT_LST)
size: $(OUT_ELF)
	$(SIZE) $(OUT_ELF)
	@echo "Finished building: default.size.stdout"

$(OUT_HEX): $(OUT_ELF)
	$(OBJCOPY) -O ihex    $< $@
	@echo "Finished building: $@"

$(OUT_BIN): $(OUT_ELF)
	$(OBJCOPY) -O binary  $< $@
	@echo "Finished building: $@"

$(OUT_LST): $(OUT_ELF)
	$(OBJDUMP) -h -S $< > $@
	@echo "Finished building: $@"

# ---------------------------------------------------------------------------
# Compile rules (auto-create out dirs)
$(BUILD)/%.o: %.c
	@$(MKDIR) $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@$(MKDIR) $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD)/%.o: %.S
	@$(MKDIR) $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

# ---------------------------------------------------------------------------
# Housekeeping
clean:
	$(RM) "$(BUILD)" "$(OUT_ELF)" "$(OUT_HEX)" "$(OUT_BIN)" "$(OUT_LST)" "$(PROJECT).map"

# Include dependency files if they exist
-include $(DEPS)
