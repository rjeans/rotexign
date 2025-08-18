FQBN       ?= arduino:avr:uno
PORT       ?=
BUILD_DIR  ?= build
SKETCH     ?= rotax_ignition_controller.ino

.PHONY: build upload clean help

help:
	@echo "Targets:"
	@echo "  make build FQBN=arduino:avr:uno            # compile"
	@echo "  make upload FQBN=arduino:avr:uno PORT=/dev/tty.usbserial-XXXX"
	@echo "  make clean                                  # remove build artifacts"

build:
	arduino-cli compile --fqbn "$(FQBN)" --build-path "$(BUILD_DIR)" .

upload: build
	@if [ -z "$(PORT)" ]; then echo "PORT not set. Example: PORT=/dev/tty.usbserial-XXXX"; exit 2; fi
	arduino-cli upload -p "$(PORT)" --fqbn "$(FQBN)" .

clean:
	rm -rf "$(BUILD_DIR)"
