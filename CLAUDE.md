# Arduino Ignition Timing Controller - Development Notes

## Current Status
The ignition timing system is very close but has persistent timing bugs that need to be resolved.

## Main Issue: Inconsistent Dwell Times in Previous-Lobe Mode
At ~2190 RPM in previous-lobe timing mode, we're seeing alternating dwell times:
- **Correct**: ~3ms (3005μs) ✅
- **Wrong**: ~17ms (16900μs) ❌  
- **Very Wrong**: ~30ms (30604μs) ❌

The pattern suggests Timer1 wraparound issues where some Compare A interrupts fire in the wrong Timer1 cycle.

## What Works
- Dwell **start** timing is correct relative to trigger pulse
- Two-stage scheduling architecture (Stage 1: dwell start, Stage 2: spark)
- Same-lobe timing appears to work correctly
- Previous-lobe mode decision logic works (transitions at correct RPM)

## What's Broken
- Previous-lobe mode has inconsistent dwell durations
- Some sparks fire multiple periods late despite correct dwell start timing
- Mix of hardware and software timing may be causing state confusion

## Recent Fixes Attempted
1. ✅ Unified two-stage scheduling for both hardware and software timing
2. ✅ Proper Timer1 state cleanup between cycles 
3. ✅ Pre-validation that both dwell start AND spark fit in same Timer1 cycle
4. ✅ Fixed software timing to use actual vs scheduled dwell start times
5. ❌ **Still not working** - timing bugs persist

## Technical Details
- **Timer1 resolution**: 0.5μs (prescaler 8, 16MHz)
- **Timer1 range**: 65535 ticks = 32.768ms max
- **Target dwell**: 3ms = 6000 ticks
- **At 2190 RPM**: Period ≈ 27,397 ticks (13.7ms)
- **Previous-lobe**: Adds one period to delay_ticks

## Code Architecture
```
calculate_and_schedule_spark() {
  if (!use_same_lobe) delay_ticks += period;  // Previous-lobe
  
  if (both dwell_start AND spark fit in Timer1) {
    // Hardware two-stage: OCR1B → Compare B ISR → OCR1A
  } else {
    // Software two-stage: scheduled_us → actual_us → +3ms
  }
}
```

## Debug Commands
```bash
# Compile
arduino-cli compile --fqbn arduino:avr:uno rotexign.ino

# Analyze timing (in analysis directory - ALWAYS use venv)
cd analysis && source venv/bin/activate && python3 timing_analyzer.py wokwi-logic.vcd

# Generate timing calculations
cd analysis && source venv/bin/activate && python3 calculate_timing_values.py

# Check previous-lobe data
grep "True" wokwi-logic-analysis.csv | head -20
```

## Next Steps for Tomorrow
1. **Examine VCD file in detail** - trace through specific cycles with long dwells
2. **Add debug output** - log Timer1 values and timing decisions  
3. **Simplify further** - consider pure software timing for previous-lobe mode
4. **Step through Compare B ISR logic** - verify spark_time calculations
5. **Check for race conditions** - ensure hardware/software timing don't interfere

## Key Files
- `rotexign.ino` - Main timing controller
- `wokwi/timing_analyzer.py` - VCD analysis tool
- `wokwi/wokwi-logic.vcd` - Timing waveform data
- `wokwi-logic-analysis.csv` - Processed timing measurements

## Critical Bug Found & FIXED!
**The problem**: Hardware and software timing were running SIMULTANEOUSLY!
- When switching between hardware/software timing modes, the previous mode wasn't being canceled
- This caused duplicate dwell/spark events with chaotic timing
- Root cause: Main loop was processing software events BEFORE new triggers
- Race condition: Software events from previous trigger executed before new trigger could cancel them

**The fix**: 
1. ✅ **Main loop order fixed**: Process new triggers BEFORE software timing events
2. ✅ When choosing hardware timing: Call `SoftwareTiming::cancel_pending_events()` first  
3. ✅ When choosing software timing: Disable hardware compare interrupts with `TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B))`
4. ✅ Always clean up the previous timing mode before starting a new one

**Key insight**: The cancellation logic was correct, but the execution order was wrong. Now cancellation happens before event execution, preventing the race condition that caused dual timing systems to run simultaneously.