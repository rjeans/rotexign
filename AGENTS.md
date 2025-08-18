# Repository Guidelines

## Project Structure & Module Organization
- `rotexign.ino`: Main Arduino sketch (ATmega328P: Uno/Nano). INT0 (D2) trigger, Timer1 timebase, dwell/spark scheduling, serial.
- `doc/ignitioncontroller.md`: System overview and hardware wiring.
- `doc/testing_calibration_guide.md`: Bench testing and calibration steps.
 - `doc/AgentDesignNotes.md`: Canonical design constraints and wiring notes.

## Build, Test, and Development Commands
- Arduino IDE: Open the repo folder, select board/port, and upload.
- Arduino CLI: `arduino-cli core update-index && arduino-cli core install arduino:avr`
  - Compile: `make build FQBN=arduino:avr:uno`
  - Upload: `make upload FQBN=arduino:avr:uno PORT=/dev/tty.usbserial-XXXX`
- Serial monitor: 115200 baud (`STATUS`, `DIAG`, `RESET`). See testing guide.

## Coding Style & Naming Conventions
- Indentation: 2 spaces; no tabs; keep lines concise.
- Naming: `UPPER_SNAKE_CASE` for constants and pins; `snake_case` for functions and variables; Arduino `setup`/`loop` as entry points.
- C++ for Arduino (AVR). Prefer `const`, mark ISR-shared state `volatile`, keep ISRs short and deterministic. Avoid dynamic allocation.
- Comments: brief `//` inline notes; top-of-file block for high-level context.

## Testing Guidelines
- Framework: none yet; rely on bench testing per `doc/testing_calibration_guide.md`.
- Verification: use the serial console at 115200. Example: send `STATUS` during run; expect RPM, period (µs), advance (°), and error flags.
- Hardware checks: verify kill switch logic, LED patterns, and spark scheduling at safe RPMs before live tests.

## Commit & Pull Request Guidelines
- Commits: short, imperative subject (≤72 chars). Use scopes when helpful, e.g., `timer:`, `serial:`, `docs:`. Group related changes.
- Messages: explain what/why; reference issues (`#123`) when applicable.
- Pull Requests: include a concise description, affected modules/files, test notes (hardware used, serial output snippets), and updated docs if behavior changes. Add photos/diagrams when wiring changes.

## Security & Configuration Tips
- Trigger: D2 (INT0) falling edge via optocoupler. Outputs default LOW on boot.
- Configure in constants: `CDI_MODE=false` (smart coil), `PPR=2`, `THETA_TRIGGER=47.0`, `REV_LIMIT=7000`, `DWELL_MS≈3`.
- Dwell clamp: code limits to ≤40% duty at high RPM to protect the coil.
- Validate board/FQBN before uploading (Uno: `arduino:avr:uno`; Nano may require `arduino:avr:nano:cpu=atmega328old`).
- Bench-test with a function generator and scope before any live engine tests.
