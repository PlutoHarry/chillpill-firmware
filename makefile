################################################################################
# Portable makefile for STM32F103C8 (works without CubeIDE generating outputs)
################################################################################

RM := rm -rf

# Auto-generated CubeIDE fragments (these define OBJS/C_DEPS/S_DEPS and rules)
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

# Pull in dependency files if they exist
ifneq ($(MAKECMDGOALS),clean)
  ifneq ($(strip $(S_DEPS)),)
    -include $(S_DEPS)
  endif
  ifneq ($(strip $(S_UPPER_DEPS)),)
    -include $(S_UPPER_DEPS)
  endif
  ifneq ($(strip $(C_DEPS)),)
    -include $(C_DEPS)
  endif
endif

# ------------------------------------------------------------------------------
# Build artifact
# ------------------------------------------------------------------------------
BUILD_ARTIFACT_NAME := PLU
ELF := $(BUILD_ARTIFACT_NAME).elf
MAP := $(BUILD_ARTIFACT_NAME).map
HEX := $(BUILD_ARTIFACT_NAME).hex
BIN := $(BUILD_ARTIFACT_NAME).bin
LST := $(BUILD_ARTIFACT_NAME).list
SIZE_STDOUT := default.size.stdout

# Linker script (keep this **relative** to repo)
LINKER_SCRIPT := Core/STM32F103C8TX_FLASH.ld

# Toolchain
CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size

# MCU flags (cortex-m3, no FPU)
CPUFLAGS := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft

# Common link flags
LDFLAGS := $(CPUFLAGS) -T"$(LINKER_SCRIPT)" \
  --specs=nosys.specs --specs=nano.specs -static \
  -Wl,-Map="$(MAP)",--gc-sections -u _printf_float -u _scanf_float \
  -Wl,--start-group -lc -lm -Wl,--end-group

# ------------------------------------------------------------------------------
# Default target
# ------------------------------------------------------------------------------
all: $(ELF) secondary-outputs

# ------------------------------------------------------------------------------
# Link: **use $(OBJS)** so make knows it must compile first
# ------------------------------------------------------------------------------
$(ELF): $(OBJS) $(USER_OBJS) $(LINKER_SCRIPT) makefile
	@echo 'Linking $@'
	$(CC) -o $@ $(OBJS) $(USER_OBJS) $(LDFLAGS)

# Optional helpers (don’t rely on them for building)
objects.list: $(OBJS)
	@printf "%s\n" $(OBJS) > $@

$(SIZE_STDOUT): $(ELF)
	$(SIZE) $(ELF)
	@echo 'Finished building: $@'

$(LST): $(ELF)
	$(OBJDUMP) -h -S $(ELF) > $(LST)
	@echo 'Finished building: $@'

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $(ELF) $(HEX)
	@echo 'Finished building: $@'

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $(ELF) $(BIN)
	@echo 'Finished building: $@'

secondary-outputs: $(SIZE_STDOUT) $(LST) $(HEX) $(BIN)

# ------------------------------------------------------------------------------
# Clean
# ------------------------------------------------------------------------------
clean:
	-$(RM) $(ELF) $(MAP) $(HEX) $(BIN) $(LST) $(SIZE_STDOUT)
	# If CubeIDE subdir.mk rules put .o in source dirs, clean those too:
	-find Core -name '*.o' -delete || true
	-find Drivers -name '*.o' -delete || true
	@echo 'Clean done'

.PHONY: all clean secondary-outputs
