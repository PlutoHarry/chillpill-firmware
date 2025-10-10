################################################################################
# GNU Makefile for STM32F103C8 (ChillPill)
# Toolchain: arm-none-eabi (installed by apt in Codex)
# Notes:
#  - Relies on CubeIDE auto-generated fragments: sources.mk, objects.mk, subdir.mk
#  - Linker script is repo-relative (no absolute host paths).
################################################################################

# ------------------------------------------------------------------------------
# Auto-generated fragments (keep these includes as your project expects them)
# ------------------------------------------------------------------------------
-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

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

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets)

# ------------------------------------------------------------------------------
# Project artifact names
# ------------------------------------------------------------------------------
BUILD_ARTIFACT_NAME      := PLU
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX    :=
BUILD_ARTIFACT           := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

EXECUTABLES   += PLU.elf
MAP_FILES     += PLU.map
SIZE_OUTPUT   += default.size.stdout
OBJDUMP_LIST  += PLU.list
OBJCOPY_HEX   += PLU.hex
OBJCOPY_BIN   += PLU.bin

# ------------------------------------------------------------------------------
# Toolchain + flags
# ------------------------------------------------------------------------------
CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size

CPUFLAGS  := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
LIBGROUPS := -Wl,--start-group -lc -lm -Wl,--end-group

# Linker script (repo-relative). Override with `make LINKER_SCRIPT=...` if needed.
LINKER_SCRIPT ?= $(CURDIR)/Core/STM32F103C8TX_FLASH.ld

# Verify linker script early and provide a friendly error
ifeq ($(wildcard $(LINKER_SCRIPT)),)
$(error Linker script not found: $(LINKER_SCRIPT). Please ensure Core/STM32F103C8TX_FLASH.ld exists or pass LINKER_SCRIPT=<path>)
endif

LDFLAGS := $(CPUFLAGS) \
  -T"$(LINKER_SCRIPT)" \
  --specs=nosys.specs --specs=nano.specs -static \
  -Wl,-Map="PLU.map" -Wl,--gc-sections \
  -u _printf_float -u _scanf_float

# ------------------------------------------------------------------------------
# Default targets
# ------------------------------------------------------------------------------
# All Target
all: main-build

# Main-build Target
main-build: PLU.elf secondary-outputs

# ------------------------------------------------------------------------------
# Link, size, dump, hex/bin
# ------------------------------------------------------------------------------
PLU.elf: $(OBJS) $(USER_OBJS) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	$(CC) -o "$@" @"objects.list" $(USER_OBJS) $(LIBS) $(LDFLAGS) $(LIBGROUPS)
	@echo 'Finished building target: $@'
	@echo

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	$(SIZE) $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo

PLU.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	$(OBJDUMP) -h -S $(EXECUTABLES) > "$@"
	@echo 'Finished building: $@'
	@echo

PLU.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	$(OBJCOPY) -O ihex $(EXECUTABLES) "$@"
	@echo 'Finished building: $@'
	@echo

PLU.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	$(OBJCOPY) -O binary $(EXECUTABLES) "$@"
	@echo 'Finished building: $@'
	@echo

# ------------------------------------------------------------------------------
# Convenience / phony
# ------------------------------------------------------------------------------
clean:
	-$(RM) PLU.bin PLU.elf PLU.hex PLU.list PLU.map default.size.stdout
	-@echo

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents main-build secondary-outputs
-include ../makefile.targets
