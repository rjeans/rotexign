#!/usr/bin/env python3
"""
Simple VCD Timing Analyzer for rotexign
Analyzes D0 (input), D1 (spark), D2 (dwell) signals
"""

import sys
import statistics

def analyze_vcd(filename):
    """Analyze VCD file for timing curve data"""
    
    # Constants from design
    TRIGGER_ANGLE_BTDC = 47.0  # degrees
    PULSES_PER_REV = 2
    
    print(f"Analyzing: {filename}")
    
    # Parse VCD file
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    # Skip to data section
    data_start = 0
    for i, line in enumerate(lines):
        if line.strip() == '$enddefinitions $end':
            data_start = i + 1
            break
    
    # Parse timing data
    current_time = 0
    trigger_times = []  # D0 falling edges
    spark_events = []   # D1 transitions
    dwell_events = []   # D2 transitions
    
    for line in lines[data_start:]:
        line = line.strip()
        if not line:
            continue
            
        if line.startswith('#'):
            current_time = int(line[1:])
        elif line == '0!':  # D0 falling (trigger)
            trigger_times.append(current_time)
        elif line == '1"':  # D1 high (dwell start)
            spark_events.append(('dwell_start', current_time))
        elif line == '0"':  # D1 low (spark fire)
            spark_events.append(('spark_fire', current_time))
        elif line == '1#':  # D2 high (dwell marker start)
            dwell_events.append(('marker_start', current_time))
        elif line == '0#':  # D2 low (dwell marker end)
            dwell_events.append(('marker_end', current_time))
    
    print(f"Found {len(trigger_times)} triggers, {len(spark_events)} spark events")
    
    # Analyze timing relationships
    timing_data = []
    
    for i in range(len(spark_events)):
        if spark_events[i][0] == 'dwell_start':
            # Find corresponding spark fire
            spark_time = None
            for j in range(i+1, len(spark_events)):
                if spark_events[j][0] == 'spark_fire':
                    spark_time = spark_events[j][1]
                    break
            
            if spark_time is None:
                continue
                
            dwell_start_time = spark_events[i][1]
            
            # Find preceding trigger
            trigger_time = None
            for t in reversed(trigger_times):
                if t < dwell_start_time:
                    trigger_time = t
                    break
            
            if trigger_time is None:
                continue
            
            # Calculate RPM from trigger interval
            prev_trigger = None
            for k, t in enumerate(trigger_times):
                if t == trigger_time and k > 0:
                    prev_trigger = trigger_times[k-1]
                    break
            
            if prev_trigger is None:
                continue
            
            # Calculate parameters
            period_ns = trigger_time - prev_trigger
            period_us = period_ns / 1000.0
            rpm = (60_000_000.0 / period_us) / PULSES_PER_REV
            
            trigger_to_spark_ns = spark_time - trigger_time
            trigger_to_spark_us = trigger_to_spark_ns / 1000.0
            
            dwell_duration_ns = spark_time - dwell_start_time
            dwell_duration_us = dwell_duration_ns / 1000.0
            
            # Calculate advance angle
            # period_us is time between triggers (180°), not full revolution (360°)
            degrees_per_us = 180.0 / period_us
            trigger_to_spark_degrees = trigger_to_spark_us * degrees_per_us
            advance_angle = TRIGGER_ANGLE_BTDC - trigger_to_spark_degrees
            
            timing_data.append({
                'time_sec': spark_time / 1_000_000_000.0,
                'rpm': rpm,
                'advance_angle': advance_angle,
                'trigger_to_spark_us': trigger_to_spark_us,
                'dwell_duration_us': dwell_duration_us
            })
    
    # Display results
    if not timing_data:
        print("No complete timing events found")
        return
    
    print("\n" + "="*80)
    print("EXTRACTED TIMING CURVE")
    print("="*80)
    print(f"{'Time':>8} {'RPM':>6} {'Advance':>8} {'Trigger→Spark':>13} {'Dwell':>8}")
    print(f"{'(sec)':>8} {'':>6} {'(°BTDC)':>8} {'(μs)':>13} {'(μs)':>8}")
    print("-"*80)
    
    for data in timing_data:
        print(f"{data['time_sec']:8.3f} {data['rpm']:6.0f} {data['advance_angle']:8.1f} "
              f"{data['trigger_to_spark_us']:13.1f} {data['dwell_duration_us']:8.1f}")
    
    # Generate RPM curve summary
    generate_curve_summary(timing_data)

def generate_curve_summary(timing_data):
    """Generate RPM-binned timing curve"""
    
    # Group by RPM (500 RPM bins)
    rpm_groups = {}
    for data in timing_data:
        rpm_bin = int(data['rpm'] / 500) * 500
        if rpm_bin not in rpm_groups:
            rpm_groups[rpm_bin] = []
        rpm_groups[rpm_bin].append(data)
    
    print(f"\n" + "="*70)
    print("TIMING CURVE SUMMARY (500 RPM bins)")
    print("="*70)
    print(f"{'RPM Range':>12} {'Count':>6} {'Avg RPM':>8} {'Avg Advance':>12} {'Avg Dwell':>10}")
    print(f"{'':>12} {'':>6} {'':>8} {'(°BTDC)':>12} {'(μs)':>10}")
    print("-"*70)
    
    for rpm_bin in sorted(rpm_groups.keys()):
        if rpm_bin == 0:
            continue
            
        events = rpm_groups[rpm_bin]
        avg_rpm = statistics.mean([e['rpm'] for e in events])
        avg_advance = statistics.mean([e['advance_angle'] for e in events])
        avg_dwell = statistics.mean([e['dwell_duration_us'] for e in events])
        
        print(f"{rpm_bin:>7}-{rpm_bin+499:<4} {len(events):6d} {avg_rpm:8.0f} "
              f"{avg_advance:12.1f} {avg_dwell:10.1f}")
    
    # Compare with programmed curves
    compare_curves(rpm_groups)

def compare_curves(rpm_groups):
    """Compare with programmed timing curves"""
    
    # From rotexign.ino
    rpm_points = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000]
    safe_curve = [0, 6, 12, 15, 15, 14, 13, 12]
    perf_curve = [0, 6, 12, 20, 20, 19, 18, 17]
    
    def interpolate(rpm, points, values):
        if rpm <= points[0]: return values[0]
        if rpm >= points[-1]: return values[-1]
        
        for i in range(len(points)-1):
            if points[i] <= rpm <= points[i+1]:
                ratio = (rpm - points[i]) / (points[i+1] - points[i])
                return values[i] + ratio * (values[i+1] - values[i])
        return values[-1]
    
    print(f"\n" + "="*80)
    print("COMPARISON: ACTUAL vs PROGRAMMED CURVES")
    print("="*80)
    print(f"{'RPM':>6} {'Actual':>8} {'Safe':>8} {'Perf':>8} {'Error':>8} {'Curve Used':>10}")
    print(f"{'':>6} {'(°BTDC)':>8} {'(°BTDC)':>8} {'(°BTDC)':>8} {'(Act-Safe)':>8} {'':>10}")
    print("-"*80)
    
    for rpm_bin in sorted(rpm_groups.keys()):
        if rpm_bin == 0:
            continue
            
        events = rpm_groups[rpm_bin]
        avg_rpm = statistics.mean([e['rpm'] for e in events])
        actual_advance = statistics.mean([e['advance_angle'] for e in events])
        
        safe_advance = interpolate(avg_rpm, rpm_points, safe_curve)
        perf_advance = interpolate(avg_rpm, rpm_points, perf_curve)
        
        error_safe = actual_advance - safe_advance
        error_perf = actual_advance - perf_advance
        
        # Determine which curve is closer
        curve_used = "SAFE" if abs(error_safe) < abs(error_perf) else "PERF"
        
        print(f"{avg_rpm:6.0f} {actual_advance:8.1f} {safe_advance:8.1f} "
              f"{perf_advance:8.1f} {error_safe:+8.1f} {curve_used:>10}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 simple_timing_analyzer.py <vcd_file>")
        sys.exit(1)
    
    try:
        analyze_vcd(sys.argv[1])
    except FileNotFoundError:
        print(f"Error: File {sys.argv[1]} not found")
    except Exception as e:
        print(f"Error: {e}")