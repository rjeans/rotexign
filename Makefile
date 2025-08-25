
# --- Project layout ---
SKETCH        := rotexign
SKETCH_DIR    := firmware/$(SKETCH)
BUILD_DIR     := build




TARGET_HEX    := $(BUILD_DIR)/$(SKETCH).ino.hex
TARGET_ELF    := $(BUILD_DIR)/$(SKETCH).ino.elf

# --- Arduino ---
FQBN          := arduino:avr:uno

# --- WASM ---
WASI_SYSROOT := /opt/wasi-sysroot

# --- Phonies ---
.PHONY: all clean 

all: $(TARGET_JSON) $(TARGET_HEX) $(TARGET_WASM)

# -------- Firmware (Arduino CLI) --------
$(TARGET_HEX): | $(BUILD_DIR)
	@echo "Compiling Arduino sketch..."
	arduino-cli compile --fqbn "$(FQBN)" --output-dir "$(BUILD_DIR)" --warnings default "$(SKETCH_DIR)"
	@test -f "$(TARGET_ELF)" || (echo "ERROR: ELF file not generated" && exit 1)
	@test -f "$(TARGET_HEX)" || (echo "ERROR: HEX file not generated" && exit 1)
	@echo "Arduino compilation successful"



# Ensure build dir exists
$(BUILD_DIR):
	@echo "Creating build directory..."
	mkdir -p "$@"

# List output files
list: all
	@echo "Build outputs:"
	@ls -la $(BUILD_DIR)/

