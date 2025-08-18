# Arduino Ignition Controller - Testing & Calibration Guide

## Updates to Align with Current Implementation

### Pre-Installation Testing

#### Bench Testing Setup
- **Relay Logic**: Verify that the coil is grounded during startup.
- **Kill Switch**: Test debounce logic by simulating noisy conditions.
- **Rev Limiter**: Gradually increase RPM to test hysteresis functionality.

### Calibration Steps

#### Pickup Angle Determination
- **Static Timing**: Use a timing light to verify the updated `THETA_TRIGGER` value.
- **Dynamic Timing**: Adjust the `ADVANCE` command in real-time to fine-tune the timing curve.

#### Timing Curve Development
- **Safe Mode**: Ensure the maximum advance does not exceed 15°.
- **Performance Mode**: Gradually increase advance while monitoring for detonation.

### Safety System Validation

#### Kill Switch Testing
- **Debounce Logic**: Verify reliable operation under noisy conditions.
- **Recovery**: Ensure normal operation resumes after the switch is released.

#### Rev Limiter Testing
- **Hysteresis**: Confirm that the limiter prevents rapid toggling near the threshold.

---

## Advanced Tuning

### Real-Time Diagnostics
- Use the `ADVANCE` command to adjust the timing curve dynamically.
- Monitor diagnostic counters for glitches, rev limits, and kill switch activations.

### Data Logging
- Log RPM, advance, and error events to optimize performance.
- Analyze timing curve data to identify areas for improvement.

---

## Pre-Installation Testing

### 1. Bench Testing Setup

#### Required Equipment
- Function generator (1Hz - 1kHz capability)
- Digital oscilloscope (minimum 4 channels)
- Multimeter
- 12V power supply
- Breadboard and jumper wires
- 1kΩ and 10kΩ resistors

#### Test Circuit Setup
```
Function Generator → 1kΩ → Arduino D2 (INT0)
                     ↓
                   10kΩ to GND (pull-down)

Oscilloscope Channels:
Ch1: Function generator output
Ch2: Arduino D2 (filtered input)
Ch3: Arduino D9 (OC1A - spark output)
Ch4: Arduino D10 (OC1B - auxiliary output)
```

### 2. Timer Precision Validation

#### Test Procedure
1. **Connect function generator to input capture pin**
2. **Set generator to 10Hz (600 RPM equivalent)**
3. **Monitor timing accuracy with oscilloscope**
4. **Expected results:**
   - Input period: 100ms ±0.1%
   - Output timing should match calculated advance
   - Jitter should be <1µs

#### Verification Script
```cpp
// Add to setup() for bench testing
void bench_test_mode() {
  Serial.println("=== BENCH TEST MODE ===");
  while(1) {
    if (new_period_flag) {
      new_period_flag = false;
      Serial.print("Period: "); Serial.print(ticks_to_us(period_ticks));
      Serial.print("µs, RPM: "); Serial.print(rpm_filtered);
      Serial.print(", Advance: "); Serial.println(get_advance_for_rpm(rpm_filtered));
    }
    delay(100);
  }
}
```

### 3. One-Revolution-Ahead Testing

#### Critical Test Points
- **Trigger/TDC**: TDC occurs 47° after trigger
- **Test advance**: >47° BTDC should schedule next-rev spark
- **Expected delay**: (360° - (advance - 47°))/360° × 360°-scaled period

#### Validation
1. Set function generator to 83.33Hz (5000 RPM)
2. Verify spark occurs at correct time in next cycle
3. Measure actual delay vs calculated delay
4. Tolerance: ±0.5° equivalent time

## Hardware Integration Testing

### 1. Input Conditioning Verification

#### Magneto Pickup Testing
- **Signal amplitude**: Typically 1-50V AC
- **Frequency range**: 5Hz (300 RPM) to 200Hz (12000 RPM)
- **Waveform**: Sine wave or complex AC

#### Optocoupler Circuit
```
Magneto Pickup → Bridge Rectifier → Current Limiting Resistor → Optocoupler LED
Optocoupler Output → Pull-up Resistor → Arduino D2 (INT0, falling edge)
```

#### Test Points
1. **Raw magneto signal** (AC coupling)
2. **Rectified signal** 
3. **Optocoupler output** (should be clean digital edges)
4. **Arduino input** (after conditioning)

### 2. Output Driver Testing

#### Smart Coil (Inductive) Testing
- **Logic**: HIGH = dwell start, LOW = spark
- **Dwell**: ~3ms at 12V, clamp to ≤40% duty at high RPM
- **Isolation**: Verify galvanic isolation and grounding

#### Protection & Safety
- Outputs default LOW on boot; consider startup relay to ground coil until firmware ready.
- Kill switch: verify reliable cut at idle and under load.

## Engine Integration & Calibration

### 1. Initial Setup

#### Safety Precautions
⚠️ **CRITICAL SAFETY REQUIREMENTS**
- Remove spark plugs during initial testing
- Ensure proper grounding of all equipment
- Have fire extinguisher available
- Wear safety glasses and gloves
- Test in well-ventilated area

#### Physical Installation
1. **Mount controller** in weatherproof enclosure
2. **Connect pickup** with shielded cable
3. **Install output drivers** with proper heat sinking
4. **Connect kill switch** with emergency accessibility
5. **Verify all connections** before power-up

### 2. Pickup Angle Determination

This is the most critical calibration step!

#### Method 1: Static Timing (Preferred)
1. **Remove spark plug**, install compression gauge
2. **Rotate engine** to TDC (maximum compression)
3. **Mark flywheel** at TDC position
4. **Connect timing light** to spark output
5. **Set controller advance** to match pickup angle
6. **Crank engine slowly** and observe timing light
7. **Adjust THETA_TRIGGER (default 47.0)** until light flashes at TDC

#### Method 2: Dynamic Timing
1. **Install timing light** on spark plug wire
2. **Run engine** at low RPM (800-1000)
3. **Set advance table** to known value (e.g., 10° BTDC)
4. **Observe actual timing** with timing light
5. **Calculate offset** and adjust THETA_TRIGGER

#### Calculation Example
- Observed timing: 15° BTDC
- Commanded timing: 10° BTDC
- Pickup appears 5° late
- Adjust: THETA_TRIGGER = previous_value + 5°

### 3. Timing Curve Development

#### Conservative Starting Curve
```cpp
const uint16_t bp_rpm[] = {0, 600, 1000, 2000, 4000, 6000, 8000, 10000};
const int8_t bp_adv[]   = {5,   8,   12,   16,   20,   18,   15,    12};
```

#### Optimization Process
1. **Start conservative** (less advance)
2. **Gradually increase** advance at each RPM point
3. **Monitor for detonation** (knock sensor recommended)
4. **Check exhaust temperature** 
5. **Measure performance** (acceleration, top speed)

#### Performance Indicators
- **No detonation** under full load
- **Smooth acceleration** through RPM range
- **Stable idle** (if applicable)
- **Good fuel economy** at cruise
- **Maximum power** at high RPM

### 4. Safety System Validation

#### Kill Switch Testing
1. **Test at idle**: Should stop immediately
2. **Test under load**: Should stop safely
3. **Test intermittent**: Should not false trigger
4. **Test recovery**: Should restart normally

#### Rev Limiter Testing
1. **Hard cut**: configured at 7000 RPM
2. **Approach**: under controlled conditions; verify sparks stop above limit and resume below.

#### Over-Rev Protection
- **Hard limit**: Complete spark cutoff above maximum RPM
- **Soft limit**: Timing retard to limit power
- **Recovery**: Smooth return to normal operation

## Diagnostic Procedures

### 1. Real-Time Monitoring

#### Serial Commands
```
STATUS  - Current RPM, timing, and engine state
DIAG    - Diagnostic counters and error history
RESET   - Clear error counters
```

#### Key Parameters to Monitor
- **RPM stability**: Should be smooth, not erratic
- **Timing accuracy**: Compare commanded vs observed
- **Error counts**: Glitches, timeouts, rev limits
- **Signal quality**: Clean pickup transitions

### 2. Troubleshooting Guide

#### No Spark Output
1. Check input signal presence and quality
2. Verify kill switch is not active
3. Confirm power supply voltages
4. Test output driver circuits

#### Erratic Timing
1. Check pickup signal for noise
2. Verify grounding and shielding
3. Check for EMI from ignition system
4. Validate timing calculations

#### Poor Performance
1. Verify pickup angle calibration
2. Check timing curve appropriateness
3. Monitor for detonation
4. Validate fuel delivery system

### 3. Data Logging

#### Parameters to Log
- RPM vs Time
- Advance vs RPM
- Error events
- Performance metrics

#### Analysis Tools
- Plot timing curves
- Identify problem RPM ranges
- Correlate errors with conditions
- Optimize curve shape

## Advanced Tuning

### 1. Performance Optimization

#### Power Curve Development
1. **Baseline dyno run** with conservative timing
2. **Incremental advance** increases at each RPM
3. **Monitor power output** and exhaust temperature
4. **Find optimal advance** for maximum power
5. **Verify reliability** under sustained load

#### Fuel-Specific Tuning
- **Premium fuel**: More aggressive advance
- **Regular fuel**: Conservative to prevent knock
- **Racing fuel**: Maximum performance curves
- **Ethanol blends**: Adjusted for fuel characteristics

### 2. Environmental Compensation

#### Temperature Effects
- **Cold conditions**: Slightly more advance
- **Hot conditions**: Slightly less advance
- **Altitude effects**: Thinner air, adjust accordingly

#### Load-Based Adjustment
- **Light load**: Optimize for efficiency
- **Heavy load**: Optimize for power
- **Sustained operation**: Balance power and reliability

## Maintenance and Support

### 1. Routine Maintenance

#### Monthly Checks
- Verify timing accuracy with timing light
- Check error counters via serial interface
- Inspect connections for corrosion
- Test kill switch operation

#### Annual Service
- Re-calibrate pickup angle
- Update timing curves based on experience
- Replace output drivers if needed
- Full system validation

### 2. Firmware Updates

#### Version Control
- Document all timing curve changes
- Keep backup of working configurations
- Test updates on bench before installation
- Maintain calibration records

#### Performance Monitoring
- Log long-term reliability data
- Track component wear patterns
- Identify optimization opportunities
- Plan preventive maintenance

## Success Metrics

- **Timing Accuracy**: ±0.5° across all RPM ranges
- **Reliability**: >1000 hours between failures  
- **Performance**: Measurable improvement over stock
- **Safety**: Zero unsafe failure modes
- **Maintainability**: Easy calibration and diagnostics
