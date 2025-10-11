################################################################################
# STM32F103C8 "Chillpill" — Make-based build
# - Builds Core/Src/Core_Files + Core/Src/Foundation_Files (+ HAL + startup)
# - EXCLUDES Control FSM folders (placeholders) by default
# - Space-safe handling for directories and files
################################################################################

# ---- Toolchain ---------------------------------------------------------------
CC      := arm-none-eabi-gcc
AS      := arm-none-eabi-gcc
LD      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size
RM      := rm -rf
MKDIR   := mkdir -p

# ---- Artifact names ----------------------------------------------------------
OUT     := PLU
OUT_DIR := build
ELF     := $(OUT).elf
HEX     := $(OUT).hex
BIN     := $(OUT).bin
MAP     := $(OUT).map
LST     := $(OUT).list

# ---- MCU / link --------------------------------------------------------------
MCU     := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
LDSCRIPT:= Core/STM32F103C8TX_FLASH.ld

# ---- Feature flags -----------------------------------------------------------
# Leave the FSM disabled until its modules are implemented.
DEFS_COMMON := -DUSE_HAL_DRIVER -DSTM32F103xB -DENABLE_CONTROL_FSM=0

# ---- Optimization / warnings -------------------------------------------------
CSTD   := -std=c11
OPT    := -O2
DBG    := -g3
WARN   := -Wall -Wextra -Werror=implicit-function-declaration
SEC    := -ffunction-sections -fdata-sections
CFLAGS := $(MCU) $(OPT) $(DBG) $(CSTD) $(WARN) $(SEC) $(DEFS_COMMON)
ASFLAGS:= $(MCU) $(DBG) -x assembler-with-cpp
LDFLAGS:= $(MCU) --specs=nosys.specs --specs=nano.specs -static \
          -Wl,-Map="$(MAP)",--gc-sections -Wl,--start-group -lc -lm -Wl,--end-group \
          -T"$(LDSCRIPT)" -u _printf_float -u _scanf_float

# ---- Include paths -----------------------------------------------------------
INCDIRS := \
  Core/Inc \
  Core/Inc/Core_Files \
  Core/Inc/Foundation_Files \
  Core/Inc/Control_FSM_Files \
  Drivers/STM32F1xx_HAL_Driver/Inc \
  Drivers/CMSIS/Include \
  Drivers/CMSIS/Device/ST/STM32F1xx/Include

CPPFLAGS := $(foreach d,$(INCDIRS),-I"$(d)")

# ---- Source folders (explicitly exclude Control FSM) -------------------------
SRC_DIRS := \
  Core/Src/Core_Files \
  Core/Src/Foundation_Files \
  Drivers/STM32F1xx_HAL_Driver/Src \
  Core/Startup

# C sources
SRC_C := $(foreach d,$(SRC_DIRS),$(wildcard $(d)/*.c))
# Assembly startup
SRC_S := $(wildcard Core/Startup/*.s)

# ---- Objects in build/… (space-safe via quoting in rules) --------------------
OBJ_C := $(patsubst %.c,$(OUT_DIR)/%.o,$(SRC_C))
OBJ_S := $(patsubst %.s,$(OUT_DIR)/%.o,$(SRC_S))
OBJS  := $(OBJ_C) $(OBJ_S)

# Make sure the build subfolders exist
DIRS  := $(sort $(dir $(OBJS)))

# ---- Default target ----------------------------------------------------------
.PHONY: all
all: $(ELF) secondary-outputs

# ---- Build rules -------------------------------------------------------------
# Create needed directories
$(OUT_DIR)/%/.dir:
	@$(MKDIR) "$(@D)"

# Pattern rule for C
$(OUT_DIR)/%.o: %.c | $(DIRS:%=%/.dir)
	@$(MKDIR) "$(dir $@)"
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c "$<" -o "$@"

# Pattern rule for ASM
$(OUT_DIR)/%.o: %.s | $(DIRS:%=%/.dir)
	@$(MKDIR) "$(dir $@)"
	$(AS) $(ASFLAGS) -c "$<" -o "$@"

# Link
$(ELF): $(OBJS)
	@echo "Linking $(ELF)"
	$(LD) -o "$(ELF)" $(OBJS) $(LDFLAGS)

# ---- Secondary outputs -------------------------------------------------------
.PHONY: secondary-outputs
secondary-outputs: $(HEX) $(BIN) $(LST) size

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex  "$(ELF)" "$(HEX)"

$(BIN): $(ELF)
	$(OBJCOPY) -O binary "$(ELF)" "$(BIN)"

$(LST): $(ELF)
	$(OBJDUMP) -h -S "$(ELF)" > "$(LST)"

.PHONY: size
size: $(ELF)
	$(SIZE) "$(ELF)"

# ---- Convenience targets -----------------------------------------------------
.PHONY: clean flash rebuild
clean:
	-$(RM) "$(OUT_DIR)" "$(ELF)" "$(HEX)" "$(BIN)" "$(MAP)" "$(LST)" *.d

rebuild: clean all

# ---- Auto-include dependency files ------------------------------------------
-include $(OBJS:.o=.d)
