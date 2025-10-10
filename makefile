################################################################################
# STM32F103C8T6 - standalone GNU Makefile (no CubeIDE fragments required)
# Project: PLU
################################################################################

# ---- Toolchain ---------------------------------------------------------------
CC      := arm-none-eabi-gcc
AS      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size
RM      := rm -rf
MKDIR_P := mkdir -p

# ---- MCU/LD ------------------------------------------------------------------
MCU      := cortex-m3
FPU      := nofp                  # no hardware FPU on F103
FLOATABI := soft
LDSCRIPT := Core/STM32F103C8TX_FLASH.ld  # committed in repo

# ---- Directories -------------------------------------------------------------
SRCDIRS  := Core/Src Drivers/STM32F1xx_HAL_Driver/Src
ASMDIRS  := Core/Startup
INCDIRS  := Core/Inc Drivers/STM32F1xx_HAL_Driver/Inc Drivers/CMSIS/Include Drivers/CMSIS/Device/ST/STM32F1xx/Include

# out-of-source build
BUILD    := build

# ---- Sources / Objects -------------------------------------------------------
C_SRCS := $(foreach d,$(SRCDIRS),$(wildcard $(d)/*.c))
S_SRCS := $(foreach d,$(ASMDIRS),$(wildcard $(d)/*.s) $(wildcard $(d)/*.S))
SRCS   := $(C_SRCS) $(S_SRCS)

OBJS   := $(addprefix $(BUILD)/,$(C_SRCS:.c=.o)) \
          $(addprefix $(BUILD)/,$(S_SRCS:.s=.o))
# also handle .S -> .o
OBJS   := $(OBJS:.S=.o)

DEPS   := $(OBJS:.o=.d)

# ---- Flags -------------------------------------------------------------------
CPUFLAGS  := -mcpu=$(MCU) -mthumb -mfloat-abi=$(FLOATABI)
OPTFLAGS  := -O2 -g3 -ffunction-sections -fdata-sections
WARNFLAGS := -Wall -Wextra -Werror=implicit-function-declaration

DEFS      := -DUSE_HAL_DRIVER -DSTM32F103xB

CFLAGS  := $(CPUFLAGS) $(OPTFLAGS) $(WARNFLAGS) $(DEFS) \
           $(addprefix -I,$(INCDIRS)) \
           -MMD -MP

ASFLAGS := $(CPUFLAGS) -x assembler-with-cpp $(DEFS) $(addprefix -I,$(INCDIRS))

LDFLAGS := $(CPUFLAGS) -T$(LDSCRIPT) \
           --specs=nosys.specs --specs=nano.specs \
           -Wl,-Map=$(BUILD)/PLU.map,--gc-sections \
           -Wl,--start-group -lc -lm -Wl,--end-group \
           -static -u _printf_float -u _scanf_float

# ---- Artifacts ---------------------------------------------------------------
TARGET := PLU
ELF    := $(BUILD)/$(TARGET).elf
BIN    := $(BUILD)/$(TARGET).bin
HEX    := $(BUILD)/$(TARGET).hex
LST    := $(BUILD)/$(TARGET).list

# ---- Default -----------------------------------------------------------------
all: $(ELF) $(HEX) $(BIN) $(LST) size

# ---- Rules -------------------------------------------------------------------
$(BUILD):
	$(MKDIR_P) $(BUILD)

# pattern rules for objects into $(BUILD)/...
$(BUILD)/%.o: %.c | $(BUILD)
	@$(MKDIR_P) $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s | $(BUILD)
	@$(MKDIR_P) $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD)/%.o: %.S | $(BUILD)
	@$(MKDIR_P) $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

# Link – includes startup object so Reset_Handler is present
$(ELF): $(OBJS) $(LDSCRIPT)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

$(LST): $(ELF)
	$(OBJDUMP) -h -S $< > $@

size: $(ELF)
	$(SIZE) $(ELF)

clean:
	$(RM) $(BUILD)

# Include dep files
-include $(DEPS)

.PHONY: all clean size
