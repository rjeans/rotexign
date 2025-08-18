# rotexign — Arduino Ignition Controller

Modern, Arduino-based ignition timing controller for the Rotax 787 two-stroke engine. It delivers precise, programmable timing with robust safety and diagnostics. See `doc/` for full technical docs and calibration guides.

## Features
- Sub-microsecond timing using Timer1 (ICP/OC) with one-revolution-ahead scheduling.
- CDI and inductive coil support, rev limiting with hysteresis, watchdog, and error flags.
- Real-time diagnostics over serial (`STATUS`, `DIAG`, `RESET`, `ADVANCE`).
- Configurable timing curves (safe and performance modes).

## Hardware (summary)
- Board: Arduino Uno/Nano (ATmega328P).
- Input: D2 (INT0) falling-edge trigger via optocoupler; kill switch on D7 (debounced).
- Output: Smart coil on D9 (HIGH = dwell start, LOW = spark); auxiliary output on D10; status LED D13.
- Engine: Two trigger lobes (PPR=2); TDC occurs 47° after trigger; hard cut at 7000 RPM with hysteresis.
- Dwell: Target ~3 ms at 12V, clamped to ≤40% duty at high RPM.
- Power: Isolate 5V control from 12V coil supply; follow noise/EMI tips in docs.

## Build & Upload
- Arduino IDE: Open the repo, select board/port, Upload.
- Arduino CLI with Makefile:
  - One‑time setup: `arduino-cli core update-index && arduino-cli core install arduino:avr`
  - Compile: `make build FQBN=arduino:avr:uno`
  - Upload: `make upload FQBN=arduino:avr:uno PORT=/dev/tty.usbserial-XXXX`
- Serial monitor: 115200 baud.

## macOS Setup
- Install CLI: `brew install arduino-cli`
- Install AVR core: `arduino-cli core update-index && arduino-cli core install arduino:avr`
- Find your board port: `ls /dev/tty.usb* /dev/tty.wch* 2>/dev/null`
- Compile: `make build FQBN=arduino:avr:uno`
- Upload: `make upload FQBN=arduino:avr:uno PORT=/dev/tty.usbserial-XXXX`

## Testing & Calibration
- Bench-test first with a function generator and scope.
- Verify pickup angle (THETA_TRIGGER) and timing curve; follow `doc/testing_calibration_guide.md`.
- Validate kill switch, rev limiter (with hysteresis), dwell behavior, and relay logic before engine tests.

## Code Layout
- `rotexign.ino`: Main firmware (INT0 trigger, Timer1 timebase, dwell/spark scheduling, safety, serial).
- `doc/ignitioncontroller.md`: Architecture, hardware, algorithms, and constraints.
- `doc/testing_calibration_guide.md`: Bench, engine integration, and tuning steps.
- `doc/AgentDesignNotes.md`: Source of truth for design decisions (wiring, timing, limits).

## Safety
- Remove plugs for initial checks; use shielded wiring and proper drivers.
- Always test on the bench before live engine operation.

## License & Contributions
- Contributions welcome via PRs; keep changes scoped and documented. See AGENTS.md for contributor guidelines.
