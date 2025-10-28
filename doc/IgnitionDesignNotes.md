# Ignition Coil Control and Timing Design Notes

## 1. Introduction
This document provides the theoretical background, physical principles, and design constraints for the rotexign ignition controller. The implementation in `rotexign.ino` represents a simplified, optimized version that achieves the same timing goals through efficient interrupt-driven code.

## 2. Key Terms and Theory

### 2.1 Ignition Coil Operation
- The ignition coil stores energy when current flows through its primary winding. This charging period is called the **dwell time**.
- In this implementation:
  - **Dwell starts** (coil charging) when output goes HIGH→LOW 
  - **Spark fires** (coil discharges) when output goes LOW→HIGH
- This creates a high-voltage spark on the secondary side at the rising edge.

### 2.2 Dwell Time
- **Dwell time** is the duration the output stays LOW, keeping the coil energized (charging).
- Example: If the Arduino drives the output LOW for 3 ms, then the dwell time is 3 ms.

### 2.3 Duty Cycle
- The **duty cycle** is the ratio of the dwell time to the total period between successive sparks:
  
  
  $$\text{Duty Cycle (\%)} = \frac{\text{Dwell Time}}{\text{Period per spark}} \times 100$$
  

- **Constraint**: The coil has a maximum continuous duty cycle of 40% to prevent overheating.

### 2.4 Engine and Spark Events
- Engine: **Rotax 787**, 2-cylinder, 2-stroke.
- One coil, wasted-spark system: the coil fires once per cylinder per revolution → **two sparks per revolution**.
- Period per spark event:
  
  $$T_{\text{spark}} = \frac{30000}{\text{RPM}} \; \text{ms}$$
  

### 2.5 Trigger and Advance Angles
- The trigger point occurs **43.5° before TDC** of the piston.
- Ignition advance curve determines how many degrees before TDC the spark occurs.
- Window available for dwell when using the same lobe:
  
  $$t_{\text{window}} = \frac{(43.5^\circ - \text{advance})}{360^\circ} \cdot \frac{60000}{\text{RPM}}$$
  
- If $t_{\text{window}} < t_{\text{dwell}}$, then previous-lobe timing must be used.

## 3. Coil Constraints from Manufacturer
- Maximum continuous duty cycle: **40%**
- Maximum dwell at 14 V: **9 ms**
- Recommended dwell at 12 V: **~3 ms**

The software must **never leave the output LOW continuously** (since LOW = dwell/charging state), otherwise the coil burns out. The output defaults to HIGH (safe state) on startup.

## 4. Duty Cycle and Safe Dwell vs RPM
At low RPM the dwell can be kept near 3 ms, but as RPM rises the period per spark shrinks, forcing the duty cycle up. Above ~4000 RPM, a fixed 3 ms dwell exceeds the 40% duty cap.

### Safe Dwell vs RPM Plot
![Plot showing safe dwell time versus RPM, with a 40% duty cycle limit indicated.](dwell_vs_rpm_40pct.png)

*Figure: Safe dwell time as a function of RPM, constrained by the 40% duty cycle limit.*

- \<=4000 RPM: 3 ms dwell is within the 40% duty cap.
- \>4000 RPM: dwell must reduce with RPM to respect the cap.

Full table: see [safe_dwell_vs_rpm.csv](safe_dwell_vs_rpm.csv).

## 5. Transition to Previous-Lobe Timing

### 5.1 Why Needed
- With same-lobe timing, dwell can only begin at the trigger point.
- At higher RPM, the available window between trigger and spark (based on advance angle) becomes too short for the required dwell.
- Using the **previous lobe** adds ~180° of crank rotation lead time, which is:
  
  $$\Delta t = \frac{30000}{\text{RPM}} \; \text{ms}$$
  
- At 7000 RPM this gives ~4.3 ms of additional margin, allowing safe dwell.

### 5.2 Implementation in rotexign.ino
- The controller calculates this transition dynamically in the `get_dwell_delay_us_from_rpm()` function
- Transition occurs when: `(dwell_us + margin) > spark_delay_us`
- In practice, this happens around **1800-2000 RPM** depending on the advance curve
- The transition is seamless with no timing discontinuities

### Switch Analysis Plot
![Previous-lobe switch](./previous_lobe_switch_plot.png)

Full table: see [previous_lobe_switch_points.csv](previous_lobe_switch_points.csv).

## 6. Practical Design Rules
1. Cap dwell time at the **minimum** of:
   - 3 ms target (at 12 V)
   - 40% duty-cycle limit (varies with RPM)
   - Absolute maximum 9 ms
2. Use **same-lobe timing** up to ~1800 RPM, then switch to **previous-lobe timing**.
3. Ensure software never leaves the output in dwell state continuously; outputs must default HIGH (safe) during startup.
4. Stabilization period: Skip first 2 trigger pulses to allow timing calculations to settle.
5. **Safety relay on D4**: Keeps coil grounded until D2 is stable HIGH for 1 second, then arms (opens relay).
   - Relay closed (LOW): Coil grounded, protected state at startup
   - Relay open (HIGH): Coil armed, ready to fire
   - Once armed, relay remains open during operation

## 7. Implementation Notes

### 7.1 Simplified Architecture in rotexign.ino
The production implementation uses a streamlined approach:
- Single trigger ISR calculates and schedules all timing
- Two Timer1 compare matches handle dwell start/stop
- All calculations done in timer ticks (0.5μs resolution)
- No floating point math in interrupt context

### 7.2 Key Optimizations
- Direct port manipulation instead of digitalWrite()
- PROGMEM storage for 81-point timing curve
- Tick-based math throughout (4 ticks = 1μs at prescaler /64)
- Inline functions for critical timing calculations

### 7.3 Validation Results
Testing with VCD analysis confirms:
- Timing accuracy: ±0.1° across entire RPM range
- Dwell consistency: ±50μs variation
- Seamless previous-lobe transition at ~2000 RPM
- Proper duty cycle limiting at high RPM

## 8. Conclusion
The rotexign controller successfully implements these theoretical principles in a production-ready system. The simplified interrupt-driven architecture achieves microsecond-precision timing while maintaining code clarity and reliability.

---
**Design Document**  
See `README.md` for implementation details and `analysis/` for validation data.
