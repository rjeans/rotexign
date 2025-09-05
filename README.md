# rotexign — Arduino Ignition Controller

Arduino-based ignition timing controller for the Rotax 787 two-stroke engine. This implementation uses interrupt-driven timing control with a deferred processing architecture for precise dwell and spark scheduling.

⚠️ **Status**: Hardware prototype with known timing bugs - missing sparks and acceleration errors

## Overview

This project implements a sophisticated ignition timing controller using an Arduino Uno/Nano (ATmega328P). The system uses hardware timer interrupts and optimized tick-based calculations to achieve microsecond-precision timing control suitable for high-performance two-stroke engines.

## Key Features

### Core Timing Engine
- **Hardware Timer-Based**: Uses Timer1 with 0.5μs resolution (prescaler /8 at 16MHz)
- **Deferred Processing**: External interrupt (INT0) captures timing, main loop processes events
- **Ring Buffer**: Buffered event system prevents missed triggers during processing
- **Direct Port Control**: Bypasses Arduino digitalRead/Write for minimal latency
- **Tick-Based Math**: All calculations in timer ticks to avoid floating-point

### Timing Control
- **Adaptive Advance Curve**: 201-point interpolated timing map stored in PROGMEM
- **Previous-Lobe Timing**: Always uses previous lobe for timing (one full period delay)
- **Precision Dwell Control**: 3ms target with 40% duty cycle protection
- **Two-Stage Scheduling**: Separate dwell start and spark fire events
- **RPM Range**: Designed for 800-8000 RPM operation

### Safety Features
- **Rev Limiter**: Hard cut at 7000 RPM with immediate ignition disable
- **Startup Protection**: 3-trigger stabilization before enabling ignition
- **Duty Cycle Protection**: Prevents coil overheating at high RPM
- **Input Pulse Filtering**: 30% period change rejection to filter noise
- **Relay Protection**: D4 relay keeps coil grounded until D2 is stable HIGH for 1 second
- **Clean Initialization**: All outputs start in safe state

## Hardware Configuration

### Pinout
| Pin | Function | Description |
|-----|----------|-------------|
| D2 | Trigger Input (INT0) | Falling edge trigger from crank sensor (47° BTDC) |
| D3 | Ignition Output | Coil control (HIGH→LOW = start dwell, LOW→HIGH = fire spark) |
| D4 | Safety Relay | Relay control (HIGH = armed/open, LOW = safe/closed at startup) |

### Engine Parameters
- **Trigger Configuration**: 2 pulses per revolution (2 lobes, 180° apart)
- **Trigger Position**: 47° BTDC
- **Coil Type**: Smart coil with 3ms dwell requirement at 12V
- **Maximum Duty Cycle**: 40% to prevent coil damage

## Implementation Details

### Current Architecture

The controller uses a deferred processing architecture with event buffering:

```
Trigger (INT0) → Buffer Event → Main Loop → Schedule Dwell → Fire Spark
```

1. **Trigger ISR** (`INT0_vect`):
   - Captures Timer1 count immediately for precise timing
   - Filters trigger pulses to reject 30% period changes (noise rejection)
   - Waits for 3-trigger stabilization before enabling ignition
   - Buffers timing events in ring buffer for main loop processing
   - Minimal processing in ISR to prevent missed triggers

2. **Main Loop Processing**:
   - Processes buffered timing events when engine is waiting
   - Calculates RPM, advance angle, and timing delays
   - Schedules dwell start using Timer1 Compare A interrupt
   - Handles relay arming logic and diagnostics

3. **Timer1 Compare A ISR** (`TIMER1_COMPA_vect`):
   - **State DWELL_SCHEDULED**: Start dwell (LOW), schedule spark
   - **State DWELLING**: Fire spark (HIGH), return to waiting
   - Implements two-stage scheduling for precise timing control

### Timing Calculations

All timing uses integer math with Timer1 ticks (0.5μs resolution):

```cpp
// RPM from period ticks (avoids floating point) 
rpm = 60,000,000 / (period_ticks * 2)

// Advance angle from 201-point curve (PROGMEM)
advance_tenths = get_advance_angle_tenths(rpm)

// Spark delay calculation
delay_angle_tenths = 470 - advance_tenths  // 47° BTDC - advance
delay_ticks = (delay_angle_tenths * period_ticks + 900) / 1800

// Dwell timing (always previous lobe)
dwell_delay_ticks = (period_ticks + spark_delay_ticks) - dwell_ticks
```

### Timing Curve Generation

The 201-point timing curve in PROGMEM is generated through a sophisticated smoothing process:

1. **Base Points**: Starts with 9 key RPM/advance points defining the desired curve shape
2. **Cubic Interpolation**: Generates 200 intermediate points using cubic spline interpolation
3. **Savitzky-Golay Filtering**: Applies polynomial smoothing to remove discontinuities
4. **Lookup Table Generation**: Creates final 201 points at 40 RPM intervals (0-8000 RPM)
5. **Fixed-Point Conversion**: Stores values as tenths of degrees (×10) for integer math

This process (`analysis/smooth_timing_curve.py`) ensures:
- Smooth transitions without abrupt changes
- Optimal engine performance across the RPM range
- Efficient storage in limited PROGMEM space
- Fast runtime lookup with linear interpolation between points

### Previous-Lobe Timing

The controller uses previous-lobe timing exclusively for all RPM ranges:

- **Always Previous-Lobe**: Adds one full period to dwell delay calculation  
- **Benefit**: Provides maximum dwell time window at all RPM
- **Trade-off**: Uses more recent RPM data but requires larger timing buffer

## Known Issues

⚠️ **Critical Issues Requiring Resolution**

### 1. Missing Sparks
- **Symptom**: Intermittent spark failures, especially during steady-state operation
- **Suspected Cause**: Event buffer overflow or timing race conditions
- **Impact**: Engine misfiring and poor performance
- **Location**: `rotexign.ino:516-528` event processing in main loop

### 2. Timing Errors on Acceleration  
- **Symptom**: Incorrect timing advance during RPM changes
- **Suspected Cause**: Using stale period data from ring buffer during acceleration
- **Impact**: Poor throttle response and potential engine damage
- **Location**: `rotexign.ino:520-527` period calculation and scheduling

### 3. Deferred Processing Latency
- **Symptom**: Variable delay between trigger and dwell start
- **Root Cause**: Main loop processing creates timing uncertainty
- **Impact**: Inconsistent ignition timing
- **Location**: Ring buffer architecture throughout main loop

## Previous Testing Status

The controller was previously extensively tested in Wokwi simulation:

🔗 **[Live Simulation Project](https://wokwi.com/projects/439745280978700289)**

**Previous Results**: 99.9% timing accuracy within ±0.1° across 14,533 measurements

**Current Status**: Hardware implementation shows timing bugs not present in simulation, likely due to:
- Real-world timing constraints not modeled in simulation
- Race conditions between ISR and main loop processing  
- Ring buffer management issues under varying load conditions

## Building and Installation

### Arduino IDE
1. Open `rotexign.ino` in Arduino IDE
2. Select Board: Arduino Uno/Nano
3. Select Port: Your Arduino's serial port
4. Click Upload

### Arduino CLI
```bash
# Install AVR core (one time)
arduino-cli core install arduino:avr

# Compile
arduino-cli compile --fqbn arduino:avr:uno rotexign.ino

# Upload
arduino-cli compile --fqbn arduino:avr:uno --upload --port /dev/ttyUSB0 rotexign.ino
```

### Testing in Simulation

1. Open the [Wokwi project](https://wokwi.com/projects/439745280978700289)
2. Click "Start Simulation"
3. Monitor serial output for timing data
4. Export VCD file for detailed analysis

### Analyzing Simulation Data

The project includes comprehensive Python-based timing analysis tools:

```bash
cd analysis
source venv/bin/activate  # Always use venv
python3 timing_analyzer.py wokwi-logic.vcd
```

This generates:
- `wokwi-logic-analysis.csv`: Detailed timing measurements
- `timing_vs_rpm.png`: Advance curve validation plot
- `timing_waveforms.png`: Sample waveform visualization

## Immediate Action Items

### 🚨 Critical Bug Fixes Required

1. **Fix Missing Sparks**
   - Investigate ring buffer overflow conditions
   - Add buffer status monitoring and diagnostics
   - Consider immediate processing instead of deferred
   - Test under high trigger rates to identify race conditions

2. **Resolve Acceleration Timing Errors**
   - Review period calculation during RPM changes
   - Consider using most recent trigger data instead of buffered data
   - Add acceleration detection and handling logic
   - Validate timing accuracy during RPM transitions

3. **Address Processing Latency**
   - Move critical timing calculations back to ISR if necessary
   - Minimize main loop processing delays
   - Consider hybrid approach: ISR for urgent, main loop for diagnostics

### 🛡️ Safety Implementation Status

- **Safety Relay**: ✅ Implemented and tested
- **Rev Limiter**: ✅ Functional at 7000 RPM  
- **Startup Protection**: ✅ 3-trigger stabilization working
- **Physical Hardware Testing**: ⚠️ **REQUIRED BEFORE ENGINE USE**

## Design Philosophy

This implementation prioritizes:

1. **Safety First**: Multiple protection mechanisms, fail-safe defaults
2. **Precision**: Hardware timers, tick-based math, direct port control  
3. **Robustness**: Noise filtering, startup protection, duty cycle limits
4. **Maintainability**: Clear code structure, comprehensive diagnostics

**Current Status**: Core design is sound, but timing architecture needs refinement to resolve hardware-specific issues not present in simulation.

## Files

- `rotexign.ino` - Main controller implementation
- `wokwi/` - Wokwi circuit design and custom test chip
- `analysis/timing_analyzer.py` - VCD timing analysis tool
- `analysis/smooth_timing_curve.py` - Timing curve smoothing and lookup table generator
- `analysis/*.csv` - Timing measurement data from simulation
- `analysis/*.png` - Performance validation plots
- `doc/IgnitionDesignNotes.md` - Theoretical background and design constraints
- `CLAUDE.md` - Development notes and debugging history

## License

This project is open source. Contributions welcome via pull requests.