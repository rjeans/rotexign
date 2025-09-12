# --- Project layout ---
SKETCH        := rotexign
SKETCH_DIR    := .
BUILD_DIR     := build
SKETCH_SRC    := $(SKETCH_DIR)/$(SKETCH).ino




TARGET_HEX    := $(BUILD_DIR)/$(SKETCH).ino.hex

# --- Arduino ---
FQBN          := arduino:avr:uno



# --- Environment ---
ifneq ("$(wildcard .env)","")
  include .env
  export
endif

# --- WASM chip build ---
CHIP_SRC      := wokwi/chip/pulse-simulator.chip.c
CHIP_JSON     := wokwi/chip/pulse-simulator.chip.json
CHIP_WASM     := $(BUILD_DIR)/pulse-simulator.chip.wasm

# Default clang command if not set in environment
CLANG_CHIP ?= clang 
CLANG_OPTIONS := --target=wasm32-unknown-wasi -nostartfiles -Wl,--import-memory -Wl,--export-table -Wl,--no-entry -Werror

# --- Phonies ---
.PHONY: all clean diagnostic 

all: $(TARGET_HEX) $(CHIP_WASM) $(BUILD_DIR)/pulse-simulator.chip.json

# -------- Firmware (Arduino CLI) --------
$(TARGET_HEX): $(SKETCH_SRC) | $(BUILD_DIR)
	@echo "Compiling Arduino sketch..."
	arduino-cli compile --fqbn "$(FQBN)" --output-dir "$(BUILD_DIR)" --warnings default "$(SKETCH_DIR)"
	@test -f "$(TARGET_HEX)" || (echo "ERROR: HEX file $(TARGET_HEX) not generated" && exit 1)
	@echo "Arduino compilation successful"

# -------- Diagnostic build --------
diagnostic: $(TARGET_HEX).diagnostic $(CHIP_WASM) $(BUILD_DIR)/pulse-simulator.chip.json
	@echo "Diagnostic build complete"

$(TARGET_HEX).diagnostic: $(SKETCH_SRC) | $(BUILD_DIR)
	@echo "Compiling Arduino sketch with diagnostic logging..."
	arduino-cli compile --fqbn "$(FQBN)" --output-dir "$(BUILD_DIR)" --warnings default --build-property "compiler.cpp.extra_flags=-DENABLE_DIAGNOSTIC_LOGGING" "$(SKETCH_DIR)"
	@test -f "$(TARGET_HEX)" || (echo "ERROR: HEX file $(TARGET_HEX) not generated" && exit 1)
	@touch "$(TARGET_HEX).diagnostic"
	@echo "Arduino diagnostic compilation successful"

$(CHIP_WASM): $(CHIP_SRC) | $(BUILD_DIR)
	@echo "Compiling chip code to WASM..."
	$(CLANG_CHIP) $(CLANG_OPTIONS) -o $@ $<

$(BUILD_DIR)/pulse-simulator.chip.json: $(CHIP_JSON) | $(BUILD_DIR)
	@echo "Copying chip JSON to build directory..."
	cp $< $@

# Ensure build dir exists
$(BUILD_DIR):
	@echo "Creating build directory..."
	mkdir -p "$@"

# List output files
list: all
	@echo "Build outputs:"
	@ls -la $(BUILD_DIR)/


clean:
	@echo "Cleaning build outputs..."
	rm -rf $(BUILD_DIR)

