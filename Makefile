# ===== Makefile: LLVM + wasi-libc (robust, direct wasm-ld linking) =====

# --- Project layout ---
SKETCH        := rotexign
SKETCH_DIR    := firmware/$(SKETCH)
BUILD_DIR     := build

SRC_DIR       := src
CHIP_NAME     := pulse-simulator.chip
CHIP_SRC      := $(SRC_DIR)/$(CHIP_NAME).c
CHIP_JSON_SRC := $(SRC_DIR)/$(CHIP_NAME).json


TARGET_WASM   := $(BUILD_DIR)/$(CHIP_NAME).wasm
TARGET_JSON   := $(BUILD_DIR)/$(CHIP_NAME).json
TARGET_HEX    := $(BUILD_DIR)/$(SKETCH).ino.hex
TARGET_ELF    := $(BUILD_DIR)/$(SKETCH).ino.elf

# --- Arduino ---
FQBN          := arduino:avr:uno



# --- Phonies ---
.PHONY: all clean env check

all:  $(TARGET_JSON) $(TARGET_HEX) #$(TARGET_WASM)

# -------- Firmware (Arduino CLI) --------
$(TARGET_HEX): | $(BUILD_DIR)
	arduino-cli compile --fqbn "$(FQBN)" --output-dir "$(BUILD_DIR)" --warnings default "$(SKETCH_DIR)"
	@test -f "$(TARGET_ELF)" || true



# Copy chip descriptor JSON
$(TARGET_JSON): $(CHIP_JSON_SRC) | $(BUILD_DIR)
	cp "$<" "$@"

# Ensure build dir exists
$(BUILD_DIR):
	mkdir -p "$@"



clean:
	rm -rf "$(BUILD_DIR)"
