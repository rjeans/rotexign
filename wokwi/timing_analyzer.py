#!/usr/bin/env python3
"""
VCD Timing Analyzer - First Principles
Measures trigger falling edge to spark falling edge timing
"""
import sys
import math

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
    
    print(f"Found {len(signal_changes)} signal changes")
    
    # Process trigger falling edges and find their corresponding spark events
    timing_events = []
    missing_spark_events = []
    
    for i, (time, event) in enumerate(signal_changes):
        if event != 'trigger_fall':
            continue
        
        # Find the next spark after this trigger
        next_spark = None
        for j in range(i + 1, len(signal_changes)):
            if signal_changes[j][1] == 'spark_fall':
                next_spark = signal_changes[j]
                break
        
        if next_spark is None:
            # This is a missing spark - track it separately
            # Calculate RPM from previous trigger for missing spark analysis
            prev_trigger_time = None
            for k in range(i - 1, -1, -1):
                if signal_changes[k][1] == 'trigger_fall':
                    prev_trigger_time = signal_changes[k][0]
                    break
            
            if prev_trigger_time is not None:
                period_ns = time - prev_trigger_time
                period_us = period_ns / 1000.0
                rpm = (60_000_000.0 / period_us) / 2  # 2 triggers per revolution
                
                missing_spark_events.append({
                    'time_sec': time / 1e9,
                    'rpm': rpm,
                    'reason': 'No spark found after trigger'
                })
            continue
        
        spark_time, _ = next_spark
        delay_ns = spark_time - time
        delay_us = delay_ns / 1000.0
        
        # Check for over-rev condition - very large delays indicate missed sparks
        if delay_us > 100000:  # Over 100ms delay indicates over-rev
            continue  # Skip over-rev measurements
        
        # Determine if this is previous-lobe timing based on physical constraints
        # If available time from trigger to spark < required dwell time, must be previous-lobe
        is_previous_lobe = False
        output_state_at_trigger = False
        
        # First check the waveform pattern (original logic)
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
        
        # Calculate RPM from spark timing for comparison
        rpm_from_spark = None
        # Find previous spark time
        prev_spark_time = None
        for j in range(i - 1, -1, -1):
            if signal_changes[j][1] == 'spark_fall':
                prev_spark_time = signal_changes[j][0]
                break
        
        if prev_spark_time is not None:
            spark_period_ns = spark_time - prev_spark_time
            spark_period_us = spark_period_ns / 1000.0
            rpm_from_spark = (60_000_000.0 / spark_period_us) / 2  # 2 sparks per revolution
        
        # Convert delay to degrees
        degrees_per_us = 180.0 / period_us  # 180° per trigger period
        delay_degrees = delay_us * degrees_per_us
        advance_degrees = 47.0 - delay_degrees  # 47° BTDC trigger
        
        # Apply physical constraint check for previous-lobe timing
        # Required dwell time (from Arduino code: 3ms nominal)
        REQUIRED_DWELL_US = 3000.0
        
        # If delay time < required dwell time, must be previous-lobe timing
        if delay_us < REQUIRED_DWELL_US:
            is_previous_lobe = True
        
        # Find the actual dwell time - look for dwell start that corresponds to this spark
        actual_dwell_us = REQUIRED_DWELL_US  # Default to nominal
        
        # Find the spark index in signal_changes
        spark_idx = None
        for idx, (sig_time, sig_event) in enumerate(signal_changes):
            if sig_time == spark_time and sig_event == 'spark_fall':
                spark_idx = idx
                break
        
        if spark_idx is not None:
            # Look backwards from spark to find corresponding dwell start
            for k in range(spark_idx - 1, -1, -1):
                check_time, check_event = signal_changes[k]
                
                # Stop if we go too far back (200ms)
                if spark_time - check_time > 200000000:
                    break
                    
                if check_event == 'dwell_start':  # Dwell start
                    # Check if there's another spark between this dwell start and our spark
                    spark_between = False
                    for m in range(k + 1, spark_idx):
                        if signal_changes[m][1] == 'spark_fall':
                            spark_between = True
                            break
                    
                    if not spark_between:
                        actual_dwell_us = (spark_time - check_time) / 1000.0
                        break
        
        timing_events.append({
            'time_sec': time / 1e9,
            'rpm': rpm,
            'rpm_from_spark': rpm_from_spark,
            'delay_us': delay_us,
            'delay_degrees': delay_degrees,
            'advance_degrees': advance_degrees,
            'dwell_us': actual_dwell_us,
            'is_previous_lobe': is_previous_lobe
        })
    
    print(f"\nFound {len(timing_events)} timing measurements")
    print(f"Found {len(missing_spark_events)} missing spark events")
    
    # Display first 10 measurements
    print(f"\nFirst 10 measurements:")
    print("Time(s)   RPM   Delay(μs)  Delay(°)  Advance(°)  Mode")
    print("-" * 60)
    
    for i, t in enumerate(timing_events[:10]):
        mode = "PREV" if t['is_previous_lobe'] else "SAME"
        print(f"{t['time_sec']:6.2f} {t['rpm']:5.0f} {t['delay_us']:9.1f} {t['delay_degrees']:8.1f} {t['advance_degrees']:8.1f}  {mode}")
    
    # Group by RPM for summary
    rpm_buckets = {}
    missing_rpm_buckets = {}
    
    for t in timing_events:
        bucket = int(t['rpm'] / 500) * 500
        if bucket not in rpm_buckets:
            rpm_buckets[bucket] = []
        rpm_buckets[bucket].append(t)
    
    # Group missing sparks by RPM
    for t in missing_spark_events:
        bucket = int(t['rpm'] / 500) * 500
        if bucket not in missing_rpm_buckets:
            missing_rpm_buckets[bucket] = []
        missing_rpm_buckets[bucket].append(t)
    
    # Display accuracy metrics by RPM bucket with enhanced statistics
    print(f"\nRPM Accuracy Analysis ({len(timing_events)} total measurements):")
    print("RPM Range    Count  Missing  Dwell_SD  Delay_SD  Timing_SD  Avg_Advance  Expected  Safe   Perf  Status")
    print("-" * 105)
    
    for rpm_start in sorted(rpm_buckets.keys()):
        if rpm_start < 500:
            continue
        rpm_end = rpm_start + 499
        bucket_data = rpm_buckets[rpm_start]
        
        if len(bucket_data) < 5:  # Skip buckets with insufficient data
            continue
        
        # Calculate statistics for this bucket
        advances = [t['advance_degrees'] for t in bucket_data]
        dwells = [t['dwell_us'] for t in bucket_data]
        delays = [t['delay_degrees'] for t in bucket_data]
        prev_count = sum(1 for t in bucket_data if t['is_previous_lobe'])
        
        avg_advance = sum(advances) / len(advances)
        advance_std = (sum((x - avg_advance)**2 for x in advances) / len(advances))**0.5
        avg_dwell = sum(dwells) / len(dwells)
        dwell_std = (sum((x - avg_dwell)**2 for x in dwells) / len(dwells))**0.5
        avg_delay = sum(delays) / len(delays)
        delay_std = (sum((x - avg_delay)**2 for x in delays) / len(delays))**0.5
        prev_percent = (prev_count / len(bucket_data)) * 100
        
        rpm_mid = rpm_start + 250  # Use middle of bucket for curve calculations
        expected = get_expected_advance(rpm_mid)
        safe_timing = get_safe_curve(rpm_mid) 
        perf_timing = get_performance_curve(rpm_mid)
        error = abs(avg_advance - expected)
        
        # Get actual missing spark count for this RPM bucket
        missing_sparks = len(missing_rpm_buckets.get(rpm_start, []))
        
        status = "OK" if error < 5 else "FAIL"
        
        print(f"{rpm_start:4d}-{rpm_end:4d}     {len(bucket_data):3d}        {missing_sparks:3d}    {dwell_std:6.1f}   {delay_std:6.1f}     {advance_std:6.1f}      {avg_advance:6.1f}°     {expected:5.1f}°  {safe_timing:5.1f}° {perf_timing:5.1f}°  {status}")
    
    # Write detailed results as CSV
    csv_file = vcd_file.replace('.vcd', '-analysis.csv')
    with open(csv_file, 'w') as f:
        f.write("Time_sec,RPM,RPM_from_spark,Delay_us,Delay_deg,Advance_deg,Dwell_us,Mode,Expected_deg,Safe_deg,Performance_deg\n")
        for t in timing_events:
            mode = "PREV" if t['is_previous_lobe'] else "SAME"
            rpm_spark = t['rpm_from_spark'] if t['rpm_from_spark'] else ""
            
            # Calculate theoretical curves for this RPM
            expected_advance = get_expected_advance(t['rpm'])
            safe_advance = get_safe_curve(t['rpm'])
            performance_advance = get_performance_curve(t['rpm'])
            
            f.write(f"{t['time_sec']:.3f},{t['rpm']:.1f},{rpm_spark},{t['delay_us']:.1f},{t['delay_degrees']:.1f},{t['advance_degrees']:.1f},{t['dwell_us']:.1f},{mode},{expected_advance:.1f},{safe_advance:.1f},{performance_advance:.1f}\n")
    
    print(f"\nDetailed results written to: {csv_file}")
    
    # Write missing spark events to separate CSV
    missing_csv_file = vcd_file.replace('.vcd', '-missing-sparks.csv')
    with open(missing_csv_file, 'w') as f:
        f.write("Time_sec,RPM,Reason\n")
        for t in missing_spark_events:
            f.write(f"{t['time_sec']:.3f},{t['rpm']:.1f},{t['reason']}\n")
    
    if missing_spark_events:
        print(f"Missing spark events written to: {missing_csv_file}")
    
    return timing_events, missing_spark_events

def create_timing_analysis(timing_events, missing_spark_events):
    """Create timing analysis and PNG plots"""
    
    # Group by RPM
    rpm_groups = {}
    for t in timing_events:
        rpm_key = int(t['rpm'] / 100) * 100
        if rpm_key not in rpm_groups:
            rpm_groups[rpm_key] = []
        rpm_groups[rpm_key].append(t)
    
    # Sample measured angles at timing curve points
    sample_timing_curve_points(rpm_groups)
    
    # Generate PNG plots
    try:
        import matplotlib
        matplotlib.use('Agg')  # Use non-interactive backend
        import matplotlib.pyplot as plt
        import numpy as np
        
        create_timing_plots(timing_events)
        create_waveform_plots(timing_events)
        create_missing_spark_plot(missing_spark_events)
        
    except ImportError:
        print("\nWarning: matplotlib not available - skipping PNG plot generation")
        print("Install with: pip install matplotlib numpy")

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
        advances = [t['advance_degrees'] for t in data]
        prev_count = sum(1 for t in data if t['is_previous_lobe'])
        
        avg_advance = sum(advances) / len(advances)
        prev_percent = (prev_count / len(data)) * 100
        expected = get_expected_advance(target_rpm)
        error = avg_advance - expected
        
        status = "GOOD" if abs(error) < 2 else "FAIR" if abs(error) < 5 else "POOR"
        
        print(f"{target_rpm:4d}      {expected:5.1f}°     {avg_advance:5.1f}°  {error:+5.1f}°   {prev_percent:3.0f}%   {status}")

def get_expected_advance(rpm):
    """Get expected ignition advance for given RPM based on timing curve"""
    # Simplified timing curve - replace with actual curve data
    if rpm < 500:
        return 2.0
    elif rpm < 1000:
        return 3.0 + (rpm - 500) * 0.006  # 3° to 6°
    elif rpm < 2000:
        return 6.0 + (rpm - 1000) * 0.003  # 6° to 9°
    elif rpm < 3000:
        return 9.0 + (rpm - 2000) * 0.003  # 9° to 12°
    elif rpm < 4000:
        return 12.0 + (rpm - 3000) * 0.003  # 12° to 15°
    elif rpm < 5000:
        return 15.0  # Hold at 15°
    elif rpm < 6000:
        return 15.0 - (rpm - 5000) * 0.001  # 15° to 14°
    elif rpm < 7000:
        return 14.0 - (rpm - 6000) * 0.001  # 14° to 13°
    else:
        return 13.0 - (rpm - 7000) * 0.001  # 13° down

def get_safe_curve(rpm):
    """Get safe conservative timing curve for engine protection"""
    # Conservative timing for engine safety
    if rpm < 500:
        return 0.0
    elif rpm < 1000:
        return 2.0 + (rpm - 500) * 0.004  # 2° to 4°
    elif rpm < 2000:
        return 4.0 + (rpm - 1000) * 0.002  # 4° to 6°
    elif rpm < 3000:
        return 6.0 + (rpm - 2000) * 0.002  # 6° to 8°
    elif rpm < 4000:
        return 8.0 + (rpm - 3000) * 0.002  # 8° to 10°
    elif rpm < 5000:
        return 10.0  # Hold at 10°
    elif rpm < 6000:
        return 10.0 - (rpm - 5000) * 0.001  # 10° to 9°
    elif rpm < 7000:
        return 9.0 - (rpm - 6000) * 0.001  # 9° to 8°
    else:
        return 8.0  # Conservative limit

def get_performance_curve(rpm):
    """Get aggressive performance timing curve for maximum power"""
    # Aggressive timing for performance (requires premium fuel)
    if rpm < 500:
        return 5.0
    elif rpm < 1000:
        return 5.0 + (rpm - 500) * 0.008  # 5° to 9°
    elif rpm < 2000:
        return 9.0 + (rpm - 1000) * 0.004  # 9° to 13°
    elif rpm < 3000:
        return 13.0 + (rpm - 2000) * 0.004  # 13° to 17°
    elif rpm < 4000:
        return 17.0 + (rpm - 3000) * 0.003  # 17° to 20°
    elif rpm < 5000:
        return 20.0  # Hold at 20°
    elif rpm < 6000:
        return 20.0 - (rpm - 5000) * 0.002  # 20° to 18°
    elif rpm < 7000:
        return 18.0 - (rpm - 6000) * 0.002  # 18° to 16°
    else:
        return 16.0 - (rpm - 7000) * 0.001  # 16° down

def create_timing_plots(timing_events):
    """Create PNG timing analysis plots"""
    import matplotlib.pyplot as plt
    import numpy as np
    
    # Filter out problematic data
    filtered_events = []
    for t in timing_events:
        if -50 < t['advance_degrees'] < 50 and t['rpm'] < 8000:
            filtered_events.append(t)
    
    if len(filtered_events) < 10:
        print("Not enough clean data for plotting")
        return
    
    # Group data by RPM bins for analysis
    rpm_bins = {}
    for t in filtered_events:
        rpm_bin = int(t['rpm'] / 250) * 250
        if rpm_bin not in rpm_bins:
            rpm_bins[rpm_bin] = []
        rpm_bins[rpm_bin].append(t)
    
    # Create timing vs RPM plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Plot 1: Timing vs RPM with error bars
    rpm_points = []
    advance_means = []
    advance_stds = []
    
    for rpm in sorted(rpm_bins.keys()):
        if len(rpm_bins[rpm]) >= 5:  # At least 5 measurements
            advances = [t['advance_degrees'] for t in rpm_bins[rpm]]
            rpm_points.append(rpm)
            advance_means.append(np.mean(advances))
            advance_stds.append(np.std(advances))
    
    if rpm_points:
        # Measured data with error bars
        ax1.errorbar(rpm_points, advance_means, yerr=advance_stds, 
                    fmt='b.-', capsize=5, capthick=2, markersize=8, 
                    label='Measured (±1σ)', linewidth=2)
        
        # Theoretical curves
        rpm_curve = np.linspace(500, 8000, 100)
        expected_curve = [get_expected_advance(r) for r in rpm_curve]
        safe_curve = [get_safe_curve(r) for r in rpm_curve]
        perf_curve = [get_performance_curve(r) for r in rpm_curve]
        
        ax1.plot(rpm_curve, expected_curve, 'g-', linewidth=2, label='Expected curve')
        ax1.plot(rpm_curve, safe_curve, 'orange', linestyle='--', linewidth=2, label='Safe curve')
        ax1.plot(rpm_curve, perf_curve, 'r--', linewidth=2, label='Performance curve')
    
    ax1.set_xlabel('RPM')
    ax1.set_ylabel('Ignition Advance (degrees BTDC)')
    ax1.set_title('Ignition Timing vs RPM')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_xlim(0, 8000)
    ax1.set_ylim(-5, 25)
    
    # Plot 2: Timing Error vs Expected
    error_data = []
    rpm_data = []
    for rpm in sorted(rpm_bins.keys()):
        if len(rpm_bins[rpm]) >= 5:
            advances = [t['advance_degrees'] for t in rpm_bins[rpm]]
            expected = get_expected_advance(rpm)
            for advance in advances:
                error_data.append(advance - expected)
                rpm_data.append(rpm)
    
    if error_data:
        # Binned error analysis
        rpm_error_bins = {}
        for rpm, error in zip(rpm_data, error_data):
            rpm_bin = int(rpm / 250) * 250
            if rpm_bin not in rpm_error_bins:
                rpm_error_bins[rpm_bin] = []
            rpm_error_bins[rpm_bin].append(error)
        
        rpm_err_points = []
        error_means = []
        error_stds = []
        
        for rpm in sorted(rpm_error_bins.keys()):
            if len(rpm_error_bins[rpm]) >= 5:
                errors = rpm_error_bins[rpm]
                rpm_err_points.append(rpm)
                error_means.append(np.mean(errors))
                error_stds.append(np.std(errors))
        
        if rpm_err_points:
            ax2.errorbar(rpm_err_points, error_means, yerr=error_stds,
                        fmt='r.-', capsize=5, capthick=2, markersize=8,
                        label='Mean error ± std dev', linewidth=2)
            
            # Scatter plot of individual errors (lighter)
            ax2.scatter(rpm_data, error_data, alpha=0.3, s=20, color='lightcoral', 
                       label='Individual errors')
    
    ax2.axhline(y=0, color='green', linestyle='-', linewidth=2, label='Perfect timing')
    ax2.axhline(y=2, color='orange', linestyle='--', alpha=0.7, label='±2° tolerance')
    ax2.axhline(y=-2, color='orange', linestyle='--', alpha=0.7)
    ax2.set_xlabel('RPM')
    ax2.set_ylabel('Timing Error (degrees)')
    ax2.set_title('Timing Error vs Expected')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_xlim(0, 8000)
    ax2.set_ylim(-10, 10)
    
    plt.tight_layout()
    plt.savefig('timing_vs_rpm.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("Generated: timing_vs_rpm.png")

def create_waveform_plots(timing_events):
    """Create waveform display PNG for different RPM ranges"""
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    
    # Select representative RPM points
    target_rpms = [1000, 2000, 4000, 6000]
    
    # Group by RPM and find closest matches
    rpm_groups = {}
    for t in timing_events:
        rpm_key = int(t['rpm'] / 100) * 100
        if rpm_key not in rpm_groups:
            rpm_groups[rpm_key] = []
        rpm_groups[rpm_key].append(t)
    
    fig, axes = plt.subplots(4, 1, figsize=(14, 12))
    fig.suptitle('Ignition Timing Waveforms', fontsize=16, fontweight='bold')
    
    for i, target_rpm in enumerate(target_rpms):
        ax = axes[i]
        
        # Find closest RPM with data
        best_rpm = min([rpm for rpm in rpm_groups.keys() if len(rpm_groups[rpm]) > 5],
                      key=lambda x: abs(x - target_rpm), default=None)
        
        if best_rpm is None:
            ax.text(0.5, 0.5, f'No data for {target_rpm} RPM', 
                   transform=ax.transAxes, ha='center', va='center')
            continue
        
        data = rpm_groups[best_rpm]
        sample_point = data[len(data)//2]  # Middle sample
        
        # Calculate timing parameters
        period_us = 60_000_000.0 / (best_rpm * 2)  # Time for 180°
        advance_us = sample_point['delay_us'] 
        dwell_us = sample_point['dwell_us']
        
        # Create time axis (show ~2.5 periods)
        total_time = period_us * 2.5
        time_axis = [t for t in range(0, int(total_time), int(total_time/1000))]
        
        # Generate trigger waveform (negative pulses every 180°)
        trigger_signal = []
        spark_signal = []
        
        for t in time_axis:
            # Trigger: negative pulse every period_us
            trigger_phase = (t % period_us) / period_us
            if 0.0 <= trigger_phase <= 0.05:  # 5% duty cycle negative pulse
                trigger_signal.append(0)  # LOW (trigger active)
            else:
                trigger_signal.append(1)  # HIGH (trigger inactive)
            
            # Spark: positive pulse for dwell time, ending at spark time
            spark_phase = (t % period_us) / period_us
            advance_phase = advance_us / period_us
            dwell_phase = dwell_us / period_us
            dwell_start_phase = advance_phase - dwell_phase
            
            if dwell_start_phase <= spark_phase <= advance_phase:
                spark_signal.append(1)  # HIGH (dwell active)
            else:
                spark_signal.append(0)  # LOW (no dwell)
        
        # Plot waveforms
        time_ms = [t/1000 for t in time_axis]
        
        # Trigger signal (offset for visibility)
        trigger_plot = [t + 2 for t in trigger_signal]
        ax.plot(time_ms, trigger_plot, 'b-', linewidth=2, label='Trigger')
        ax.fill_between(time_ms, [2]*len(time_ms), trigger_plot, alpha=0.3, color='blue')
        
        # Spark signal
        ax.plot(time_ms, spark_signal, 'r-', linewidth=2, label='Spark')
        ax.fill_between(time_ms, [0]*len(time_ms), spark_signal, alpha=0.3, color='red')
        
        # Add dwell measurement annotations
        # Find dwell start and end times in the first period for annotation
        dwell_start_time = None
        dwell_end_time = None
        for i, (t, signal) in enumerate(zip(time_ms, spark_signal)):
            if signal == 1 and (i == 0 or spark_signal[i-1] == 0):  # Rising edge
                dwell_start_time = t
            elif signal == 0 and i > 0 and spark_signal[i-1] == 1:  # Falling edge
                dwell_end_time = t
                break
        
        # Add dwell measurement arrows if we found both start and end
        if dwell_start_time is not None and dwell_end_time is not None:
            # Vertical lines at dwell start/end
            ax.axvline(x=dwell_start_time, color='green', linestyle='--', alpha=0.7, linewidth=2)
            ax.axvline(x=dwell_end_time, color='green', linestyle='--', alpha=0.7, linewidth=2)
            
            # Horizontal arrow showing dwell duration
            arrow_y = 1.5
            ax.annotate('', xy=(dwell_end_time, arrow_y), xytext=(dwell_start_time, arrow_y),
                       arrowprops=dict(arrowstyle='<->', color='green', lw=2))
            
            # Dwell time label
            dwell_mid = (dwell_start_time + dwell_end_time) / 2
            measured_dwell = sample_point['dwell_us'] / 1000.0
            ax.text(dwell_mid, arrow_y + 0.2, f'{measured_dwell:.1f}ms', 
                   ha='center', va='bottom', color='green', fontweight='bold', fontsize=9)
        
        # Add timing annotation
        advance_deg = sample_point['advance_degrees']
        dwell_ms = dwell_us / 1000.0
        
        # Add yellow box with timing info including actual measured dwell
        actual_dwell_ms = sample_point['dwell_us'] / 1000.0
        info_text = f"{advance_deg:.1f}° BTDC\n{actual_dwell_ms:.1f} ms dwell"
        bbox_props = dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8)
        ax.text(0.7, 0.7, info_text, transform=ax.transAxes, fontsize=10,
                verticalalignment='center', bbox=bbox_props, fontweight='bold')
        
        # Formatting
        ax.set_ylabel('Signal')
        ax.set_title(f'{target_rpm} RPM (Same-lobe) - {advance_deg:.1f}° advance - GOOD', 
                    fontsize=12, fontweight='bold')
        ax.set_ylim(-0.5, 3.5)
        ax.set_xlim(0, total_time/1000)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper left')
        
        # Add labels
        ax.text(0.02, 2.7, 'Trigger', fontsize=10, color='blue', fontweight='bold')
        ax.text(0.02, 0.7, 'Spark', fontsize=10, color='red', fontweight='bold')
    
    axes[-1].set_xlabel('Time (ms)')
    plt.tight_layout()
    plt.savefig('timing_waveforms.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("Generated: timing_waveforms.png")

def create_missing_spark_plot(missing_spark_events):
    """Create missing spark analysis plot by RPM"""
    import matplotlib.pyplot as plt
    import numpy as np
    
    if not missing_spark_events:
        print("No missing sparks to plot")
        return
    
    # Group missing sparks by RPM bins
    rpm_bins = {}
    for event in missing_spark_events:
        rpm_bin = int(event['rpm'] / 250) * 250  # 250 RPM bins
        if rpm_bin not in rpm_bins:
            rpm_bins[rpm_bin] = 0
        rpm_bins[rpm_bin] += 1
    
    # Create bar chart
    fig, ax = plt.subplots(1, 1, figsize=(12, 6))
    
    rpm_points = sorted(rpm_bins.keys())
    missing_counts = [rpm_bins[rpm] for rpm in rpm_points]
    
    bars = ax.bar(rpm_points, missing_counts, width=200, alpha=0.7, color='red', edgecolor='darkred')
    
    # Add value labels on bars
    for bar, count in zip(bars, missing_counts):
        if count > 0:
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                   str(count), ha='center', va='bottom', fontweight='bold')
    
    ax.set_xlabel('RPM')
    ax.set_ylabel('Number of Missing Sparks')
    ax.set_title('Missing Spark Events by RPM Range')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 8000)
    
    # Add total missing count
    total_missing = sum(missing_counts)
    ax.text(0.02, 0.95, f'Total Missing Sparks: {total_missing}', 
           transform=ax.transAxes, fontsize=12, fontweight='bold',
           bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8))
    
    plt.tight_layout()
    plt.savefig('missing_sparks_by_rpm.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("Generated: missing_sparks_by_rpm.png")

if __name__ == "__main__":
    import sys
    vcd_file = sys.argv[1] if len(sys.argv) > 1 else "wokwi-logic.vcd"
    timing_events, missing_spark_events = analyze_vcd(vcd_file)
    create_timing_analysis(timing_events, missing_spark_events)