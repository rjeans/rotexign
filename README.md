# rotexign — Arduino Ignition Controller

Modern, Arduino-based ignition timing controller for the Rotax 787 two-stroke engine. It delivers precise, programmable timing with robust safety and diagnostics. See `doc/` for full technical docs and calibration guides.

## Features
- Sub-microsecond timing using Timer1 (ICP/OC) with one-revolution-ahead scheduling.
- Inductive (smart coil) support, rev limiting with hysteresis, watchdog, and error flags.
- Real-time diagnostics over serial (`STATUS`, `DIAG`, `RESET`, `ADVANCE`).
- Configurable timing curves (safe and performance modes).
 - Clean dwell marker on D10 for scoping.

## Hardware (summary)
- Board: Arduino Uno/Nano (ATmega328P).
- Input: D2 (INT0) falling-edge trigger via optocoupler; kill switch on D7 (debounced).
- Output: Smart coil control on D9 (polarity configurable via `COIL_ACTIVE_HIGH`); dwell marker on D10 (HIGH during dwell); status LED D13.
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
- Verify pickup angle (THETA_TRIGGER) and timing curve; follow `doc/TestingCalibrationGuide.md`.
- Validate kill switch, rev limiter (with hysteresis), dwell behavior, and relay logic before engine tests.

## Code Layout
- `rotexign.ino`: Main firmware (INT0 trigger, Timer1 timebase, dwell/spark scheduling, safety, serial).
- `doc/IgnitionControllerDesignNotes.md`: Architecture, hardware, algorithms, and constraints.
- `doc/TestingCalibrationGuide.md`: Bench, engine integration, and tuning steps.
- `doc/AgentDesignNotes.md`: Source of truth for design decisions (wiring, timing, limits).

## Safety
- Remove plugs for initial checks; use shielded wiring and proper drivers.
- Always test on the bench before live engine operation.

## Changes After First Round Testing

These updates address the issues documented in `doc/FirstTestResults.md`:

- D9 polarity: Added `COIL_ACTIVE_HIGH` (default `false`) to safely match an inverted output stage; D9 now idles “coil off” and only energizes during dwell.
- Triple-pulse artifact: Timer1 compare matches are now one-shot; interrupts arm only when scheduled and disable themselves after firing, avoiding stale re-fires.
- Noise-induced retriggers: INT0 ISR uses Timer1 tick deltas (0.5 µs resolution) to reject pulses closer than 0.5 ms (glitches) and 2.0 ms (spark-noise guard). No `millis()` calls in ISR.
- D10 meaning: Repurposed as a dwell marker — HIGH while the coil is charging, LOW at spark — for clean scope validation.
- Kill switch behavior: Debounced read on D7; when active, coil is forced off and any scheduled events are cancelled; error flag reported in `STATUS`/`DIAG`.
- Safe startup: On boot and when signal is lost, the coil is off and no compare matches are armed. Relay outputs (pins 3 and 4) are explicitly configured and driven.
- Serial robustness: Replaced Arduino `String` parsing with a fixed buffer to avoid heap fragmentation.

Recommended re-test procedure:

- Use a function generator to feed D2; monitor D9 (coil control) and D10 (dwell marker). Expect D10 HIGH only during dwell; D9 transitions per hardware polarity.
- Confirm single dwell/spark per trigger at ~1350 RPM and across the curve.
- Assert the kill switch to verify immediate coil-off and cancellation of scheduled events.
- Sweep past the rev limit to confirm a clean cut with counters incrementing in `DIAG`.

## License & Contributions
- Contributions welcome via PRs; keep changes scoped and documented. See AGENTS.md for contributor guidelines.
