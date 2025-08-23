#!/usr/bin/env python3
"""
VCD Timing Analyzer - First Principles
Measures trigger falling edge to spark falling edge timing
"""

def analyze_vcd(vcd_file):
    """Analyze VCD timing from first principles"""
    print(f"Analyzing: {vcd_file}")
    
    # Parse VCD file and extract all signal changes
    with open(vcd_file, 'r') as f:
        lines = f.readlines()
    
    signal_changes = []
    current_time = 0
    
    for line in lines:
        line = line.strip()
        
        if line.startswith('#'):
            current_time = int(line[1:])
        # VCD signal mapping based on logic analyzer connections:
        # D2 (trigger input) - HIGH→LOW falling edge triggers
        # D3 (spark output) - LOW→HIGH (dwell start), HIGH→LOW (spark fire)
        elif line == '0#':  # D2 (trigger input) goes low - TRIGGER EDGE (HIGH→LOW)
            signal_changes.append((current_time, 'trigger_fall'))
        elif line == '1#':  # D2 (trigger input) goes high (end of trigger pulse)
            signal_changes.append((current_time, 'trigger_rise'))
        elif line == '0$':  # D3 (spark output) goes low - SPARK FIRES (HIGH→LOW)
            signal_changes.append((current_time, 'spark_fall'))
        elif line == '1$':  # D3 (spark output) goes high - DWELL STARTS (LOW→HIGH)
            signal_changes.append((current_time, 'dwell_start'))
        # Legacy support for old mapping
        elif line == '0!':  # D0 trigger goes low - TRIGGER EDGE (HIGH→LOW)
            signal_changes.append((current_time, 'trigger_fall'))
        elif line == '1!':  # D0 trigger goes high (end of trigger pulse)
            signal_changes.append((current_time, 'trigger_rise'))
        elif line == '0"':  # D1 spark output goes low - SPARK FIRES (HIGH→LOW)
            signal_changes.append((current_time, 'spark_fall'))
        elif line == '1"':  # D1 spark output goes high - DWELL STARTS (LOW→HIGH)
            signal_changes.append((current_time, 'dwell_start'))
    
    # Sort all changes by time
    signal_changes.sort()
    
    print(f"Found {len(signal_changes)} signal changes")
    
    # Extract timing measurements
    timing_events = []
    
    for i, (time, event) in enumerate(signal_changes):
        if event == 'trigger_fall':  # Measure from HIGH→LOW trigger edge (falling edge)
            # Find the next spark falling edge after this trigger
            next_spark = None
            for j in range(i + 1, len(signal_changes)):
                if signal_changes[j][1] == 'spark_fall':
                    next_spark = signal_changes[j]
                    break
            
            if next_spark is None:
                continue
            
            spark_time, _ = next_spark
            delay_ns = spark_time - time
            delay_us = delay_ns / 1000.0
            
            # Determine if this is previous-lobe timing
            # Look for dwell_start before the trigger_fall
            is_previous_lobe = False
            output_state_at_trigger = False
            
            # Check if output was already HIGH when trigger fell (HIGH→LOW)
            for j in range(i - 1, -1, -1):
                prev_time, prev_event = signal_changes[j]
                if prev_event == 'dwell_start':
                    # Found dwell start before trigger - check if spark happened between
                    spark_between = False
                    for k in range(j, i):
                        if signal_changes[k][1] == 'spark_fall':
                            spark_between = True
                            break
                    if not spark_between:
                        is_previous_lobe = True
                        output_state_at_trigger = True
                    break
                elif prev_event == 'spark_fall':
                    # Output was LOW at trigger
                    break
            
            # Calculate RPM from trigger period  
            prev_trigger_time = None
            for j in range(i - 1, -1, -1):
                if signal_changes[j][1] == 'trigger_fall':  # Previous HIGH→LOW trigger edge
                    prev_trigger_time = signal_changes[j][0]
                    break
            
            if prev_trigger_time is None:
                continue
                
            period_ns = time - prev_trigger_time
            period_us = period_ns / 1000.0
            rpm = (60_000_000.0 / period_us) / 2  # 2 triggers per revolution
            
            # Convert delay to degrees
            degrees_per_us = 180.0 / period_us  # 180° per trigger period
            delay_degrees = delay_us * degrees_per_us
            advance_degrees = 47.0 - delay_degrees  # 47° BTDC trigger
            
            timing_events.append({
                'time_sec': time / 1e9,
                'rpm': rpm,
                'delay_us': delay_us,
                'delay_degrees': delay_degrees,
                'advance_degrees': advance_degrees,
                'is_previous_lobe': is_previous_lobe,
                'output_high_at_trigger': output_state_at_trigger
            })
    
    print(f"\nFound {len(timing_events)} timing measurements")
    
    # Show first few results
    print("\nFirst 10 measurements:")
    print("Time(s)   RPM   Delay(μs)  Delay(°)  Advance(°)  Mode")
    print("-" * 60)
    for i, t in enumerate(timing_events[:10]):
        mode = "PREV" if t['is_previous_lobe'] else "SAME"
        print(f"{t['time_sec']:6.2f} {t['rpm']:5.0f}  {t['delay_us']:8.1f}  {t['delay_degrees']:7.1f}  {t['advance_degrees']:8.1f}  {mode}")
    
    # RPM summary
    bins = {}
    for t in timing_events:
        rpm_bin = int(t['rpm'] / 500) * 500
        if rpm_bin not in bins:
            bins[rpm_bin] = {'advances': [], 'prev_count': 0, 'total': 0}
        bins[rpm_bin]['advances'].append(t['advance_degrees'])
        if t['is_previous_lobe']:
            bins[rpm_bin]['prev_count'] += 1
        bins[rpm_bin]['total'] += 1
    
    print(f"\nRPM Summary ({len(timing_events)} total measurements):")
    print("RPM Range    Count  Avg Advance  Prev%  Expected")
    print("-" * 50)
    
    expected = {0: 0, 500: 3, 1000: 6, 1500: 9, 2000: 12, 2500: 13, 3000: 15, 
                3500: 15, 4000: 15, 4500: 14, 5000: 14, 5500: 13, 6000: 13, 6500: 12, 7000: 12}
    
    for rpm_bin in sorted(bins.keys()):
        if rpm_bin > 0 and bins[rpm_bin]['total'] > 5:
            data = bins[rpm_bin]
            avg_advance = sum(data['advances']) / len(data['advances'])
            prev_percent = (data['prev_count'] / data['total']) * 100
            exp = expected.get(rpm_bin, 13)
            status = "OK" if abs(avg_advance - exp) < 3 else "FAIL"
            print(f"{rpm_bin:4d}-{rpm_bin+499:4d}  {data['total']:6d}  {avg_advance:8.1f}°  {prev_percent:4.0f}%  {exp:6.1f}° {status}")
    
    # Write detailed results
    output_file = vcd_file.replace('.vcd', '-analysis.txt')
    with open(output_file, 'w') as f:
        f.write(f"Timing Analysis: {vcd_file}\n")
        f.write(f"Total measurements: {len(timing_events)}\n\n")
        f.write("Time(s)    RPM   Delay(μs)  Delay(°)  Advance(°)  Mode\n")
        f.write("-" * 60 + "\n")
        for t in timing_events:
            mode = "PREV" if t['is_previous_lobe'] else "SAME"
            f.write(f"{t['time_sec']:6.2f}  {t['rpm']:5.0f}  {t['delay_us']:8.1f}  {t['delay_degrees']:7.1f}  {t['advance_degrees']:8.1f}  {mode}\n")
    
    print(f"\nDetailed results written to: {output_file}")
    
    # Create timing plots
    create_timing_plots(timing_events, vcd_file)
    return timing_events

def create_timing_plots(timing_events, vcd_file):
    """Create SVG plots for different RPM ranges"""
    print("\nCreating timing plots...")
    
    # Group by RPM
    rpm_groups = {}
    for t in timing_events:
        rpm_key = int(t['rpm'] / 100) * 100
        if rpm_key not in rpm_groups:
            rpm_groups[rpm_key] = []
        rpm_groups[rpm_key].append(t)
    
    # Select timing curve points for plots
    timing_curve_points = [200, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000]
    plot_rpms = []
    
    plot_data = []  # Store (target_rpm, actual_rpm, data) tuples
    
    for target_rpm in timing_curve_points:
        # Find the closest RPM bin to each timing curve point
        best = min([rpm for rpm in rpm_groups.keys() if len(rpm_groups[rpm]) > 10], 
                  key=lambda x: abs(x - target_rpm), default=None)
        if best:
            plot_data.append((target_rpm, best, rpm_groups[best]))
    
    for target_rpm, actual_rpm, data in plot_data:
        create_svg_plot(data, actual_rpm, target_rpm)
    
    print(f"Created {len(plot_data)} SVG plots for timing curve points:")
    for target_rpm, actual_rpm, data in plot_data:
        print(f"  - timing_curve_{target_rpm}rpm.svg (actual: {actual_rpm} RPM)")
    
    # Sample measured angles at timing curve points
    sample_timing_curve_points(rpm_groups)

def create_svg_plot(rpm_data, rpm, target_rpm=None):
    """Create SVG plot showing actual waveforms for specific RPM range"""
    
    # Find a representative time window from rpm_data
    sample_time = rpm_data[len(rpm_data)//2]['time_sec'] * 1e9  # Convert to ns
    
    # Read VCD file again to get waveform data around this time
    with open("wokwi-logic.vcd", 'r') as f:
        lines = f.readlines()
    
    # Extract signal changes around sample time
    signal_changes = []
    current_time = 0
    
    for line in lines:
        line = line.strip()
        if line.startswith('#'):
            current_time = int(line[1:])
        elif line in ['0#', '1#', '0$', '1$', '0!', '1!', '0"', '1"']:
            signal_changes.append((current_time, line))
    
    # Find trigger falling edges near sample time
    trigger_falls = []
    for t, e in signal_changes:
        if e in ['0#', '0!'] and abs(t - sample_time) < 200e6:  # Within 200ms (D2 or legacy D0)
            trigger_falls.append(t)
    
    if len(trigger_falls) < 3:
        return  # Not enough triggers for plot
    
    # Sort and find 3 consecutive triggers with middle one closest to sample
    trigger_falls.sort()
    best_idx = -1
    best_dist = float('inf')
    
    for i in range(len(trigger_falls) - 2):
        middle_trigger = trigger_falls[i + 1]
        dist = abs(middle_trigger - sample_time)
        if dist < best_dist:
            best_dist = dist
            best_idx = i
    
    if best_idx < 0:
        return
    
    # Get the 3 trigger times
    selected_triggers = trigger_falls[best_idx:best_idx + 3]
    first_trigger = selected_triggers[0]
    middle_trigger = selected_triggers[1]
    last_trigger = selected_triggers[2]
    
    # Window includes some margin before first trigger and after last trigger
    margin = (last_trigger - first_trigger) * 0.1  # 10% margin
    window_start = first_trigger - margin
    window_end = last_trigger + margin
    window_events = [(t, e) for t, e in signal_changes if window_start <= t <= window_end]
    
    if len(window_events) < 10:  # Not enough data
        return
    
    # Calculate averages for annotations
    avg_advance = sum(r['advance_degrees'] for r in rpm_data) / len(rpm_data)
    avg_delay = sum(r['delay_us'] for r in rpm_data) / len(rpm_data)
    prev_count = sum(1 for r in rpm_data if r['is_previous_lobe'])
    prev_percent = (prev_count / len(rpm_data)) * 100
    
    # Determine quality and color
    expected = get_expected_advance(rpm)
    error = abs(avg_advance - expected)
    mode_desc = "Previous-Lobe Mode" if prev_percent > 50 else "Same-Lobe Mode"
    
    if error < 2:
        color = '#2ECC71'  # Green
        quality = 'GOOD'
    elif error < 5:
        color = '#F39C12'  # Orange
        quality = 'FAIR'
    else:
        color = '#E74C3C'  # Red
        quality = 'POOR'
    
    # Create waveform plot
    width, height = 800, 300
    plot_width = 640  # Drawing area width
    plot_height = 200  # Drawing area height
    
    # Time range and scaling
    time_start = window_events[0][0]
    time_end = window_events[-1][0]
    time_range = time_end - time_start
    time_scale = plot_width / time_range if time_range > 0 else 1
    
    def time_to_x(t):
        return 80 + (t - time_start) * time_scale
    
    # Build waveforms  
    # Trigger: negative pulse (HIGH baseline with LOW pulses)
    # Spark: positive pulse (LOW baseline with HIGH dwell pulses)
    # SVG coordinates: y=80 is HIGH (top), y=120 is LOW (bottom) for trigger
    # SVG coordinates: y=200 is LOW (bottom), y=160 is HIGH (top) for spark
    trigger_state = True   # Start HIGH (normal state)
    spark_state = False    # Start LOW (normal state)
    trigger_path = "M 80 80"   # Start at y=80 (HIGH - top of trigger area)
    spark_path = "M 80 200"   # Start at y=200 (LOW - bottom of spark area)
    
    trigger_fall_positions = []  # X positions of trigger falls
    spark_fall_positions = []    # X positions of spark falls
    middle_trigger_x = None       # X position of middle trigger
    middle_spark_x = None         # X position of spark after middle trigger
    
    for t, event in window_events:
        x = time_to_x(t)
        
        if event in ['1#', '1!']:  # Trigger rises (end of negative pulse - return to HIGH)
            trigger_path += f" L {x} 120 L {x} 80"  # Rise from LOW to HIGH (y=120->y=80)
            trigger_state = True
        elif event in ['0#', '0!']:  # Trigger falls (start of negative pulse - HIGH to LOW)  
            trigger_path += f" L {x} 80 L {x} 120"  # Fall from HIGH to LOW (y=80->y=120)
            trigger_state = False
            trigger_fall_positions.append(x)
            # Check if this is the middle trigger
            if abs(t - middle_trigger) < 1000:  # Within 1us
                middle_trigger_x = x
        elif event in ['1$', '1"']:  # Spark rises (dwell start) (D3 or legacy D1)
            spark_path += f" L {x} 200 L {x} 160"
            spark_state = True
        elif event in ['0$', '0"']:  # Spark falls (spark fire) (D3 or legacy D1)
            spark_path += f" L {x} 160 L {x} 200"
            spark_state = False
            spark_fall_positions.append(x)
            # Check if this is the spark after middle trigger
            if middle_trigger_x is not None and middle_spark_x is None and x > middle_trigger_x:
                middle_spark_x = x
    
    # Extend paths to end
    end_x = 80 + plot_width
    trigger_path += f" L {end_x} {80 if trigger_state else 120}"  # HIGH=y80, LOW=y120
    spark_path += f" L {end_x} {160 if spark_state else 200}"    # HIGH=y160, LOW=y200
    
    # Create SVG
    svg = f'''<?xml version="1.0" ?>
<svg width="{width}" height="{height}" viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">
<style>
    .grid {{ stroke: #e8e8e8; stroke-width: 1; stroke-dasharray: 2,2; }}
    .trigger-line {{ stroke: #2E86C1; stroke-width: 2.5; fill: none; }}
    .spark-line {{ stroke: #E74C3C; stroke-width: 2.5; fill: none; }}
    .label {{ font-family: Arial, sans-serif; font-size: 12px; font-weight: bold; }}
    .title {{ font-family: Arial, sans-serif; font-size: 18px; font-weight: bold; }}
    .timing-label {{ font-family: Arial, sans-serif; font-size: 11px; fill: {color}; font-weight: bold; }}
    .axis-label {{ font-family: Arial, sans-serif; font-size: 10px; }}
</style>
<rect width="{width}" height="{height}" fill="white"/>

<!-- Grid lines -->
<line x1="80" y1="50" x2="80" y2="250" class="grid"/>
<line x1="240" y1="50" x2="240" y2="250" class="grid"/>
<line x1="400" y1="50" x2="400" y2="250" class="grid"/>
<line x1="560" y1="50" x2="560" y2="250" class="grid"/>
<line x1="720" y1="50" x2="720" y2="250" class="grid"/>
<line x1="80" y1="150" x2="720" y2="150" class="grid"/>

<!-- Waveforms -->
<path d="{trigger_path}" class="trigger-line"/>
<path d="{spark_path}" class="spark-line"/>

<!-- Signal labels -->
<text x="20" y="105" class="label" fill="#2E86C1">TRIGGER (neg)</text>
<text x="20" y="185" class="label" fill="#E74C3C">SPARK</text>'''
    
    # Add timing arrows between middle trigger fall and its spark fall
    if middle_trigger_x is not None and middle_spark_x is not None:
        mid_x = (middle_trigger_x + middle_spark_x) / 2
        
        svg += f'''
<!-- Timing measurement -->
<line x1="{middle_trigger_x}" y1="130" x2="{middle_trigger_x}" y2="170" stroke="{color}" stroke-width="1.5" stroke-dasharray="3,2"/>
<line x1="{middle_spark_x}" y1="130" x2="{middle_spark_x}" y2="170" stroke="{color}" stroke-width="1.5" stroke-dasharray="3,2"/>
<text x="{mid_x}" y="145" text-anchor="middle" class="timing-label">{avg_delay/1000:.1f} ms</text>
<text x="{mid_x}" y="158" text-anchor="middle" class="timing-label">{avg_advance:+.1f}° BTDC</text>'''
    
    # Add degree scale annotations
    if len(trigger_falls) >= 2:
        period_x = trigger_falls[1] - trigger_falls[0]
        # Each trigger period = 180°
        svg += f'''
<text x="{trigger_falls[0] + period_x/4}" y="40" text-anchor="middle" class="axis-label">45°</text>
<text x="{trigger_falls[0] + period_x/2}" y="40" text-anchor="middle" class="axis-label">90°</text>
<text x="{trigger_falls[0] + 3*period_x/4}" y="40" text-anchor="middle" class="axis-label">135°</text>
<text x="{trigger_falls[1]}" y="40" text-anchor="middle" class="axis-label">180°</text>'''
    
    # Create title showing target curve point if available
    if target_rpm and target_rpm != rpm:
        title = f"Curve Point {target_rpm} RPM (Actual: {rpm}) - {mode_desc}"
    else:
        title = f"RPM {rpm} - {mode_desc}"
        
    svg += f'''
<!-- Title and results -->
<text x="400" y="25" text-anchor="middle" class="title">{title}</text>
<text x="400" y="275" text-anchor="middle" class="timing-label">Expected: {expected:.1f}° BTDC | Quality: {quality} | Prev-lobe: {prev_percent:.0f}%</text>

<!-- Time axis -->
<text x="80" y="265" class="axis-label">0 ms</text>
<text x="400" y="265" class="axis-label">{(time_range/2)/1e6:.1f} ms</text>
<text x="720" y="265" class="axis-label">{time_range/1e6:.1f} ms</text>

</svg>'''
    
    # Use target RPM in filename if available, otherwise actual RPM
    curve_point = target_rpm if target_rpm else rpm
    filename = f"timing_curve_{curve_point}rpm.svg"
    with open(filename, 'w') as f:
        f.write(svg)

def sample_timing_curve_points(rpm_groups):
    """Sample measured timing at each timing curve point for comparison"""
    timing_curve_points = [200, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000]
    
    print(f"\n=== TIMING CURVE POINT ANALYSIS ===")
    print("Point   Expected  Measured  Error   Mode%   Status")
    print("-" * 50)
    
    for target_rpm in timing_curve_points:
        # Find closest RPM bin with sufficient data
        best_rpm = None
        best_distance = float('inf')
        
        for rpm_bin in rpm_groups.keys():
            if len(rpm_groups[rpm_bin]) >= 10:  # Need at least 10 measurements
                distance = abs(rpm_bin - target_rpm)
                if distance < best_distance and distance <= 100:  # Within 100 RPM
                    best_distance = distance
                    best_rpm = rpm_bin
        
        if best_rpm is None:
            print(f"{target_rpm:4d}    [NO DATA]")
            continue
            
        # Calculate statistics for this point
        data = rpm_groups[best_rpm]
        measured_advances = [t['advance_degrees'] for t in data]
        avg_measured = sum(measured_advances) / len(measured_advances)
        prev_lobe_count = sum(1 for t in data if t['is_previous_lobe'])
        prev_lobe_percent = (prev_lobe_count / len(data)) * 100
        
        # Get expected value
        expected = get_expected_advance(target_rpm)
        error = avg_measured - expected
        
        # Determine status
        if abs(error) < 2:
            status = "GOOD"
        elif abs(error) < 5:
            status = "FAIR" 
        else:
            status = "POOR"
            
        print(f"{target_rpm:4d}    {expected:6.1f}°  {avg_measured:7.1f}°  {error:+5.1f}°  {prev_lobe_percent:4.0f}%   {status}")

def get_expected_advance(rpm):
    """Get expected advance from Arduino timing curves (safe curve)"""
    # Arduino TimingCurves::safe_advance
    points = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000]
    advances = [0, 6, 12, 15, 15, 14, 13, 12]
    
    # Check if beyond last point FIRST (Arduino logic)
    if rpm >= points[-1]:
        return advances[-1]
    
    for i in range(1, len(points)):
        if rpm <= points[i]:
            ratio = (rpm - points[i-1]) / (points[i] - points[i-1])
            return advances[i-1] + (advances[i] - advances[i-1]) * ratio
    return advances[-1]

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        analyze_vcd(sys.argv[1])
    else:
        analyze_vcd("wokwi-logic.vcd")