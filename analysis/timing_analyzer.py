#!/usr/bin/env python3
"""
Simplified VCD Timing Analyzer
One line per trigger pulse with theoretical comparison
"""
import sys
import csv
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict, Optional

# Signal mappings for VCD parsing
SIGNAL_MAPPINGS = {
    # Current VCD format (D2=trigger, D3=spark)  
    '0!': 'trigger_fall',   # D2 falling edge
    '1!': 'trigger_rise',   # D2 rising edge
    '0"': 'dwell_start',    # D3 falling edge (dwell start)
    '1"': 'spark_fire',     # D3 rising edge (spark fire)
    
    # Legacy format support  
    '0#': 'd4_fall',
    '1#': 'd4_rise',
    '0$': 'dwell_start',
    '1$': 'spark_fire',
}

def parse_vcd_file(vcd_file: str) -> List[Tuple[int, str]]:
    """Parse VCD file and extract signal changes."""
    signal_changes = []
    current_time = 0

    with open(vcd_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#'):
                current_time = int(line[1:])
            elif line in SIGNAL_MAPPINGS:
                signal_changes.append((current_time, SIGNAL_MAPPINGS[line]))

    return signal_changes

def calculate_rpm_from_period(period_ns: int) -> float:
    """Calculate RPM from period between triggers (2 pulses per revolution)."""
    if period_ns <= 0:
        return 0
    period_us = period_ns / 1000.0
    return (60_000_000.0 / period_us) / 2  # 2 pulses per revolution

def get_theoretical_advance(rpm: float) -> float:
    """Calculate theoretical advance using interpolated smooth curve."""
    try:
        # Try to load the smoothed curve from CSV
        import os
        curve_file = 'timing_curve_cubic.csv'
        
        if os.path.exists(curve_file):
            # Load the smoothed curve data once and cache it
            if not hasattr(get_theoretical_advance, 'curve_data'):
                curve_rpm = []
                curve_advance = []
                
                with open(curve_file, 'r') as f:
                    reader = csv.reader(f)
                    next(reader)  # Skip comment line
                    next(reader)  # Skip header
                    for row in reader:
                        curve_rpm.append(float(row[0]))
                        curve_advance.append(float(row[1]))
                
                get_theoretical_advance.curve_data = (curve_rpm, curve_advance)
            
            # Interpolate for the requested RPM
            curve_rpm, curve_advance = get_theoretical_advance.curve_data
            return float(np.interp(rpm, curve_rpm, curve_advance))
            
    except (FileNotFoundError, ImportError, ValueError):
        pass
    
    # Fallback to simple curve if file not available
    if rpm <= 1000:
        return rpm * 0.006  # 0-6° from 0-1000 RPM
    elif rpm <= 2000:
        return 6 + (rpm - 1000) * 0.006  # 6-12° from 1000-2000 RPM
    elif rpm <= 3000:
        return 12 + (rpm - 2000) * 0.003  # 12-15° from 2000-3000 RPM
    elif rpm <= 4000:
        return 15  # Hold at 15°
    elif rpm <= 5000:
        return 15 - (rpm - 4000) * 0.001  # 15-14° from 4000-5000 RPM
    elif rpm <= 6000:
        return 14 - (rpm - 5000) * 0.001  # 14-13° from 5000-6000 RPM
    else:
        return max(12 - (rpm - 6000) * 0.001, 10)  # 13-12° from 6000-7000 RPM

def analyze_triggers(vcd_file: str) -> List[Dict]:
    """
    Simple first-principles spark detection:
    Only analyze triggers after D4 goes high (ready signal)
    For trigger T1 and next trigger T2:
    - If spark exists > T1 and <= T2, it belongs to T1
    - Otherwise T1 has missing spark
    - Find dwell start > T1 and < T2 (can be negative delay = previous lobe)
    """
    signal_changes = parse_vcd_file(vcd_file)
    print(f"Found {len(signal_changes)} signal changes")
    
    # Find when D4 goes high (ready signal)
    d4_ready_time = None
    for time, event in signal_changes:
        if event == 'd4_rise':
            d4_ready_time = time
            print(f"D4 ready signal at {time} ns = {time/1000000:.3f} ms")
            break
    
    if d4_ready_time is None:
        print("Warning: D4 ready signal not found, analyzing all triggers")
        d4_ready_time = 0
    
    results = []
    
    # Find all trigger falling edges after D4 goes high
    trigger_indices = []
    for i, (time, event) in enumerate(signal_changes):
        if event == 'trigger_fall' and time >= d4_ready_time:
            trigger_indices.append(i)
    
    print(f"Found {len(trigger_indices)} trigger pulses")
    
    # Process each trigger with improved noise detection
    # First pass: identify noise spikes by looking at period patterns
    trigger_times = [signal_changes[idx][0] for idx in trigger_indices]
    noise_flags = [False] * len(trigger_indices)
    
    # Calculate all periods
    periods = []
    for i in range(len(trigger_times) - 1):
        periods.append(trigger_times[i+1] - trigger_times[i])
    
    # Look for noise spikes using local context
    # Pattern: If we see a very short period followed by a compensating period,
    # the trigger causing the short period is likely noise
    MIN_PERIOD_NS = 1_000_000  # 1ms minimum period
    
    for i in range(len(periods)):
        if i > 0 and i < len(periods) - 1:  # Need previous and next periods
            prev_period = periods[i-1]
            current_period = periods[i]
            next_period = periods[i+1]
            
            # If previous period was reasonable (> 5ms)
            if prev_period > MIN_PERIOD_NS * 5:
                # And current period is much shorter than previous (< 25%)
                if current_period < prev_period * 0.25:
                    # And the sum of current + next is close to previous (within 20%)
                    combined = current_period + next_period
                    if abs(combined - prev_period) < prev_period * 0.2:
                        # The trigger after the short period (i+1) is noise
                        noise_flags[i+1] = True
    
    # Process each trigger including the last one
    previous_valid_period_ns = None
    last_valid_trigger_idx = None
    last_valid_trigger_time = None
    T0_time = None  # Time of previous trigger (for first trigger handling)
    trigger_num = 0  # Track valid trigger count (excludes noise)
    
    for idx in range(len(trigger_indices)):
        T1_idx = trigger_indices[idx]
        T1_time = signal_changes[T1_idx][0]
        T0_time = T1_time if T0_time is None else T0_time
        
        # Check if this trigger was pre-identified as noise
        is_noise = noise_flags[idx]
        noise_reason = "pattern detection" if is_noise else ""
        
        # Increment trigger_num for ALL triggers (including noise)
        trigger_num += 1
        
        # Calculate period based on whether this trigger is noise or not
        if is_noise:
            # For noise triggers, calculate period from last valid trigger (if exists)
            if last_valid_trigger_time is not None:
                period_ns = T1_time - last_valid_trigger_time
            else:
                # No previous valid trigger, use immediate previous
                if idx > 0:
                    prev_time = signal_changes[trigger_indices[idx-1]][0]
                    period_ns = T1_time - prev_time
                else:
                    period_ns = 0
        else:
            # For valid triggers, calculate period from last valid trigger
            if last_valid_trigger_time is not None:
                period_ns = T1_time - last_valid_trigger_time
            else:
                # This is the first valid trigger, use immediate previous
                if idx > 0:
                    prev_time = signal_changes[trigger_indices[idx-1]][0]
                    period_ns = T1_time - prev_time
                else:
                    period_ns = 0
        
        period_us = period_ns / 1000.0
        
        # For spark search, use next trigger if available, otherwise search to end of data
        if idx < len(trigger_indices) - 1:
            T2_idx = trigger_indices[idx + 1]
            T2_time_spark_search = signal_changes[T2_idx][0]
            next_is_noise = noise_flags[idx+1] if idx+1 < len(noise_flags) else False
        else:
            # Last trigger - search from this trigger to end of VCD data
            T2_time_spark_search = signal_changes[-1][0] if signal_changes else T1_time + 100_000_000  # 100ms window
            next_is_noise = False
        
        # Additional noise checks if not already flagged
        if not is_noise and not next_is_noise:
            # Special handling for first trigger (idx == 0)
            # Don't mark as noise just because it has no period
            if idx == 0:
                # First trigger - keep it valid unless it's a spike
                pass
            # Handle zero or very small periods (but not for first trigger)
            elif period_ns <= 1000:  # Less than 1 microsecond - likely duplicate
                is_noise = True
                noise_reason = "duplicate"
            
            # Apply 30% window filter (if we have a previous valid period)
            # But skip this check if the next trigger is noise (would give false positive)
            elif previous_valid_period_ns is not None:
                min_valid_period = max(previous_valid_period_ns // 3, MIN_PERIOD_NS)
                if period_ns < min_valid_period:
                    is_noise = True
                    noise_reason = "30% filter"
        
        rpm = calculate_rpm_from_period(period_ns)
        
        # Update tracking variables
        if not is_noise:
            # Update tracking for next iteration
            previous_valid_period_ns = period_ns
            last_valid_trigger_idx = idx
            last_valid_trigger_time = T1_time
        
        # Find spark between T1 and T2_spark_search: spark > T1_time and spark <= T2_time_spark_search
        spark_time = None
        spark_found = False
        
        for i in range(len(signal_changes)):
            time, event = signal_changes[i]
            if event == 'spark_fire' and time > T1_time and time <= T2_time_spark_search:
                spark_time = time
                spark_found = True
                break
        
        # Calculate timing if spark found
        if spark_found:
            delay_us = (spark_time - T1_time) / 1000.0
            
            # Find dwell start by going backward from spark
            dwell_start_time = None
            for i in range(len(signal_changes)):
                time, event = signal_changes[i]
                if time >= spark_time:
                    # Found spark or later event, search backwards for dwell_start
                    for j in range(i-1, -1, -1):
                        back_time, back_event = signal_changes[j]
                        if back_event == 'dwell_start':
                            # Found a dwell start, check if it's reasonable (within 50ms)
                            if (spark_time - back_time) < 50_000_000:  # 50ms limit
                                dwell_start_time = back_time
                                break
                    break
            
            # Calculate dwell duration (always positive) and delay (can be negative)
            if dwell_start_time is not None:
                dwell_us = (spark_time - dwell_start_time) / 1000.0  # Always positive
                dwell_delay_us = (dwell_start_time - T1_time) / 1000.0  # Can be negative for previous-lobe
            else:
                dwell_us = None
                dwell_delay_us = None
            
            # Convert delay to degrees (180° per trigger period)
            if period_us > 0:
                degrees_per_us = 180.0 / period_us
                delay_degrees = delay_us * degrees_per_us
                advance_degrees = 47.0 - delay_degrees  # 47° is TDC position
            else:
                delay_degrees = None
                advance_degrees = None
        else:
            # Missing spark
            delay_us = None
            delay_degrees = None
            advance_degrees = None
            dwell_us = None
            dwell_delay_us = None
        
        # Calculate theoretical advance and delay
        theoretical_advance = get_theoretical_advance(rpm)
        
        # Convert theoretical advance to delay in microseconds
        # theoretical_advance is degrees BTDC, delay is degrees ATDC
        theoretical_delay_degrees = 47.0 - theoretical_advance  # 47° is TDC position
        degrees_per_us = 180.0 / period_us if period_us > 0 else 0
        theoretical_delay_us = theoretical_delay_degrees / degrees_per_us if degrees_per_us > 0 else None
        
        # Calculate error if spark present
        if advance_degrees is not None:
            error_degrees = advance_degrees - theoretical_advance
        else:
            error_degrees = None
        
        results.append({
            'trigger_num': trigger_num,
            'trigger_time_ns': T1_time,
            'trigger_time_ms': T1_time / 1_000_000,
            'diagnostics_time_ms': (T1_time - T0_time)/1_000_000 if T0_time is not None else 0,
            'period_us': period_us,
            'rpm': rpm,
            'spark_found': spark_found,
            'delay_us': delay_us,
            'delay_degrees': delay_degrees,
            'advance_degrees': advance_degrees,
            'dwell_us': dwell_us,
            'dwell_delay_us': dwell_delay_us,
            'theoretical_advance': theoretical_advance,
            'theoretical_delay_us': theoretical_delay_us,
            'error_degrees': error_degrees,
            'is_noise': is_noise,
            'noise_reason': noise_reason
        })
    
    return results

def generate_csv_output(results: List[Dict]):
    """Generate CSV file with one line per trigger."""
    with open('wokwi-logic-analysis.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Header
        writer.writerow([
            'trigger_num', 'time_ms', 'diagnostics_time_ms', 'period_us', 'rpm', 
            'spark_found', 'delay_us', 'delay_degrees', 
            'advance_degrees', 'dwell_us', 'dwell_delay_us', 'theoretical_advance', 
            'theoretical_delay_us', 'error_degrees', 'is_noise', 'noise_reason'
        ])
        
        # Data rows - exclude noise triggers from CSV output
        csv_trigger_num = 0
        for r in results:
            if r.get('is_noise', False):
                continue  # Skip noise triggers
            csv_trigger_num += 1
            writer.writerow([
                csv_trigger_num,
                f"{r['trigger_time_ms']:.3f}",
                f"{r['diagnostics_time_ms']:.3f}",
                f"{r['period_us']:.1f}",
                f"{r['rpm']:.0f}",
                r['spark_found'],
                f"{r['delay_us']:.1f}" if r['delay_us'] is not None else 'MISSING',
                f"{r['delay_degrees']:.2f}" if r['delay_degrees'] is not None else 'MISSING',
                f"{r['advance_degrees']:.2f}" if r['advance_degrees'] is not None else 'MISSING',
                f"{r['dwell_us']:.1f}" if r['dwell_us'] is not None else 'MISSING',
                f"{r['dwell_delay_us']:.1f}" if r['dwell_delay_us'] is not None else 'MISSING',
                f"{r['theoretical_advance']:.2f}",
                f"{r['theoretical_delay_us']:.1f}" if r['theoretical_delay_us'] is not None else 'N/A',
                f"{r['error_degrees']:.2f}" if r['error_degrees'] is not None else 'N/A',
                r['is_noise'],
                r['noise_reason']
            ])
    
    valid_count = sum(1 for r in results if not r.get('is_noise', False))
    noise_count = sum(1 for r in results if r.get('is_noise', False))
    print(f"Generated wokwi-logic-analysis.csv with {valid_count} valid triggers (excluded {noise_count} noise spikes)")

def create_timing_plot(results: List[Dict]):
    """Create plots showing timing analysis and dwell characteristics."""
    if not results:
        print("No results to plot")
        return
    
    # Filter out noise triggers first
    valid_results = [r for r in results if not r.get('is_noise', False)]
    noise_results = [r for r in results if r.get('is_noise', False)]
    
    # Calculate RPM plot range from VALID triggers only (exclude noise)
    if valid_results:
        valid_rpms = [r['rpm'] for r in valid_results]
        min_rpm = min(valid_rpms)
        max_rpm = max(valid_rpms)
    else:
        min_rpm = 0
        max_rpm = 6000
    
    # Round to nearest 1000
    min_rpm_plot = int(min_rpm // 1000) * 1000
    max_rpm_plot = int((max_rpm + 999) // 1000) * 1000  # Round up
    
    # Separate valid data into found and missing sparks
    found_results = [r for r in valid_results if r['spark_found']]
    missing_results = [r for r in valid_results if not r['spark_found']]
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))
    
    # Top plot: Timing advance vs RPM
    if found_results:
        found_rpms = [r['rpm'] for r in found_results]
        found_advances = [r['advance_degrees'] for r in found_results]
        ax1.scatter(found_rpms, found_advances, alpha=0.4, s=8, c='blue', 
                   label=f'Observed ({len(found_results)} sparks)', zorder=2)
    
    # Plot theoretical curve over the data range
    rpm_range = np.linspace(min_rpm_plot, max_rpm_plot, 100)
    theoretical_curve = [get_theoretical_advance(rpm) for rpm in rpm_range]
    ax1.plot(rpm_range, theoretical_curve, 'r--', linewidth=2, 
            label='Theoretical curve', zorder=1)
    
    # Note: Missing sparks are excluded from the timing graph for clarity
    # They are still tracked in the statistics and CSV output
    
    ax1.set_xlabel('RPM')
    ax1.set_ylabel('Ignition Advance (degrees BTDC)')
    title = 'Ignition Timing: Observed vs Theoretical'
    exclusions = []
    if noise_results:
        exclusions.append(f'{len(noise_results)} noise')
    if missing_results:
        exclusions.append(f'{len(missing_results)} missing')
    if exclusions:
        title += f' ({" & ".join(exclusions)} excluded)'
    ax1.set_title(title)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='lower right')
    
    # Set x-axis limits to rounded RPM range
    ax1.set_xlim(min_rpm_plot, max_rpm_plot)
    
    # Set y-axis limits based on actual data range
    if found_results:
        found_advances = [r['advance_degrees'] for r in found_results]
        y_min = min(found_advances)
        y_max = max(found_advances)
        
        # Add some padding (10% on each side)
        y_range = y_max - y_min
        y_padding = y_range * 0.1
        
        # Round to nice values
        y_min_plot = math.floor(y_min - y_padding)
        y_max_plot = math.ceil(y_max + y_padding)
        
        # Ensure we show the theoretical curve range too
        theoretical_max = max(theoretical_curve) if theoretical_curve else 15
        y_max_plot = max(y_max_plot, math.ceil(theoretical_max + 1))
        
        ax1.set_ylim(y_min_plot, y_max_plot)
    else:
        ax1.set_ylim(-5, 20)
    
    # Bottom plot: Dwell time and duty cycle vs RPM
    dwell_results = [r for r in found_results if r['dwell_us'] is not None]
    
    if dwell_results:
        dwell_rpms = np.array([r['rpm'] for r in dwell_results])
        dwell_times_ms = np.array([r['dwell_us'] / 1000.0 for r in dwell_results])
        periods_us = np.array([r['period_us'] for r in dwell_results])
        duty_cycles = (np.array([r['dwell_us'] for r in dwell_results]) / periods_us) * 100
        
        # Create twin y-axis for duty cycle
        ax2_twin = ax2.twinx()
        
        # Plot dwell time (left axis)
        ax2.scatter(dwell_rpms, dwell_times_ms, alpha=0.4, s=8, c='green', 
                   label='Dwell time', zorder=2)
        
        # Plot duty cycle (right axis) - cap at 100% for visualization
        duty_cycles_capped = np.minimum(duty_cycles, 100)
        ax2_twin.scatter(dwell_rpms, duty_cycles_capped, alpha=0.4, s=8, c='orange',
                        label='Duty cycle', zorder=2)
        
        # Add reference lines
        ax2.axhline(y=3, color='green', linestyle='--', alpha=0.7, 
                   label='Target 3ms dwell')
        ax2_twin.axhline(y=40, color='orange', linestyle='--', alpha=0.7,
                        label='40% max duty cycle')
        
        # Configure axes
        ax2.set_xlabel('RPM')
        ax2.set_ylabel('Dwell Time (ms)', color='green')
        ax2_twin.set_ylabel('Duty Cycle (%)', color='orange')
        ax2.set_title('Dwell Time and Duty Cycle vs RPM')
        ax2.grid(True, alpha=0.3)
        
        # Color the tick labels
        ax2.tick_params(axis='y', labelcolor='green')
        ax2_twin.tick_params(axis='y', labelcolor='orange')
        
        # Set axis limits using same RPM range as top plot
        ax2.set_xlim(min_rpm_plot, max_rpm_plot)
        ax2.set_ylim(0, max(dwell_times_ms) * 1.1 if len(dwell_times_ms) > 0 else 10)
        
        # Better duty cycle scaling - use capped values for reasonable max
        max_duty_capped = max(duty_cycles_capped) if len(duty_cycles_capped) > 0 else 50
        if max_duty_capped <= 10:
            duty_cycle_max = 10
        elif max_duty_capped <= 20:
            duty_cycle_max = 20
        elif max_duty_capped <= 50:
            duty_cycle_max = 50
        else:
            duty_cycle_max = 100  # Cap at 100%
        ax2_twin.set_ylim(0, duty_cycle_max)
        
        # Legends
        lines1, labels1 = ax2.get_legend_handles_labels()
        lines2, labels2 = ax2_twin.get_legend_handles_labels()
        ax2.legend(lines1 + lines2, labels1 + labels2, loc='lower right')
    else:
        ax2.text(0.5, 0.5, 'No dwell data available', transform=ax2.transAxes,
                ha='center', va='center', fontsize=12)
        ax2.set_title('Dwell Time and Duty Cycle vs RPM')
    
    plt.tight_layout()
    plt.savefig('timing_vs_rpm.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("Generated timing_vs_rpm.png")
    
    # Generate timing waveforms for key RPM points
    generate_timing_waveforms(results)

def generate_timing_waveforms(results: List[Dict]):
    """Generate timing waveform plots for key operating points."""
    if not results:
        print("No results for waveform generation")
        return
    
    # Find valid results with spark data
    valid_results = [r for r in results if r.get('spark_found', False) and not r.get('is_noise', False)]
    if not valid_results:
        print("No valid results for waveform generation")
        return
    
    # Sort by RPM
    valid_results.sort(key=lambda x: x['rpm'])
    
    # Select four key timing scenarios to demonstrate:
    # 1. SAME LOBE: Low RPM where dwell starts after trigger (positive delay)
    # 2. INSTANT DWELL: Transition RPM where dwell starts immediately (small negative delay)  
    # 3. PREVIOUS LOBE: High RPM where dwell should start in previous period (large negative delay)
    # 4. NOISE FILTERING: Same lobe with noise pulse demonstration
    
    selected_results = []
    
    # 1. Find SAME LOBE timing (positive dwell delay - dwell starts after trigger)
    same_lobe_candidates = [r for r in valid_results 
                           if r.get('dwell_delay_us') is not None 
                           and float(r.get('dwell_delay_us', 0)) > 500]
    if same_lobe_candidates:
        same_lobe = min(same_lobe_candidates, key=lambda x: abs(x['rpm'] - 1500))
        selected_results.append(same_lobe)
    
    # 2. Find INSTANT DWELL transition point (small negative delay, around -100 to -400μs)
    instant_dwell_candidates = [r for r in valid_results 
                               if r.get('dwell_delay_us') is not None 
                               and -400 < float(r.get('dwell_delay_us', 0)) < -50]
    if instant_dwell_candidates:
        # Pick one closest to the transition point
        instant_dwell = min(instant_dwell_candidates, key=lambda x: abs(x['rpm'] - 2100))
        selected_results.append(instant_dwell)
    
    # 3. Find PREVIOUS LOBE timing (large negative delay at high RPM)
    previous_lobe_candidates = [r for r in valid_results 
                               if r.get('dwell_delay_us') is not None 
                               and float(r.get('dwell_delay_us', 0)) < -1000 
                               and r['rpm'] > 4000]
    if previous_lobe_candidates:
        # Pick a high RPM example to show clear previous lobe timing
        previous_lobe = min(previous_lobe_candidates, key=lambda x: abs(x['rpm'] - 4500))
        # Convert the instant dwell implementation to show proper previous lobe visualization
        period_us = previous_lobe['period_us']
        # Save the original negative delay for description
        original_delay = previous_lobe['dwell_delay_us']
        # Convert this to a positive delay from the PREVIOUS trigger
        actual_delay = abs(original_delay)
        previous_lobe_delay = period_us - actual_delay
        previous_lobe['original_delay_us'] = original_delay
        previous_lobe['dwell_delay_us'] = previous_lobe_delay
        previous_lobe['is_previous_lobe'] = True
        selected_results.append(previous_lobe)
    
    # 4. Always add NOISE FILTERING demonstration
    # Use a same lobe example to show noise filtering
    if same_lobe_candidates:
        # Create noise filtering demo based on same lobe timing
        noise_demo = same_lobe_candidates[0].copy()  # Use first same lobe candidate
        noise_demo['show_noise_filtering'] = True
        # Add info about the actual noise data from analysis
        noise_stats = {
            'total_noise': len([r for r in results if r.get('is_noise', False)]),
            'pattern_detection': len([r for r in results if r.get('noise_reason') == 'pattern detection']),
            'filter_30percent': len([r for r in results if r.get('noise_reason') == '30% filter'])
        }
        noise_demo['noise_stats'] = noise_stats
        selected_results.append(noise_demo)
    
    if len(selected_results) < 2:
        print("Not enough data points for waveform generation")
        return
    
    # Create waveform plot
    fig, axes = plt.subplots(len(selected_results), 1, figsize=(12, 2.5 * len(selected_results)))
    if len(selected_results) == 1:
        axes = [axes]
    
    for idx, result in enumerate(selected_results):
        ax = axes[idx]
        
        # Extract timing data
        rpm = result['rpm']
        period_us = result['period_us']
        advance_deg = result['advance_degrees']
        dwell_us = result.get('dwell_us', 3000)
        # Handle dwell_delay_us that might be None or string
        dwell_delay_raw = result.get('dwell_delay_us', 0)
        if dwell_delay_raw is None or dwell_delay_raw == 'MISSING':
            dwell_delay_us = 0
        else:
            dwell_delay_us = float(dwell_delay_raw)
        
        # Determine timing mode based on characteristics 
        is_previous_lobe = result.get('is_previous_lobe', False)
        show_noise_filtering = result.get('show_noise_filtering', False)
        original_delay = result.get('original_delay_us', dwell_delay_us)
        
        if show_noise_filtering:
            noise_stats = result.get('noise_stats', {})
            total_noise = noise_stats.get('total_noise', 0)
            mode_text = "NOISE FILTERING"
            mode_description = f"Filtered {total_noise} noise triggers - 30% period change rejection"
        elif is_previous_lobe:
            mode_text = "PREVIOUS LOBE"
            mode_description = f"Dwell starts in previous period ({abs(original_delay):.0f}μs early)"
        elif original_delay > 500 or (not is_previous_lobe and dwell_delay_us > 500):
            mode_text = "SAME LOBE"
            mode_description = "Dwell starts after trigger in same 180° period"
        elif original_delay < 0 or dwell_delay_us < 0:
            mode_text = "INSTANT DWELL" 
            mode_description = f"Should start {abs(original_delay):.0f}μs early - immediate dwell"
        else:
            mode_text = "UNKNOWN"
            mode_description = "Unknown timing mode"
        
        # Create time axis (show 2 full periods)
        t_max = period_us * 2
        t = np.linspace(0, t_max, 1000)
        
        # Generate trigger pulse (LOW pulse for 6 degrees)
        trigger = np.ones_like(t) * 3
        pulse_duration_us = period_us * 6 / 360  # 6 degrees out of 360
        for i in range(2):
            pulse_start = i * period_us
            pulse_end = pulse_start + pulse_duration_us
            trigger[(t >= pulse_start) & (t <= pulse_end)] = 0
        
        # Add noise pulse demonstration for noise filtering waveform
        if show_noise_filtering:
            # Show noise pulse that coincides with spark timing from PREVIOUS trigger
            # Calculate when the spark from trigger 0 would fire
            trigger_0_time = 0
            if dwell_delay_us > 500:
                # Same lobe: spark fires after trigger + dwell_delay + dwell_duration
                spark_0_time = trigger_0_time + dwell_delay_us + dwell_us
            else:
                # Other modes: spark fires after trigger + dwell_duration  
                spark_0_time = trigger_0_time + dwell_us
            
            # Add noise pulse on trigger signal coinciding with previous spark
            if spark_0_time < t_max:
                noise_duration = 1000  # 1ms as per pulse-simulator
                noise_mask = (t >= spark_0_time) & (t <= spark_0_time + noise_duration)
                trigger[noise_mask] = 0
                
                # Add annotation for the noise pulse
                ax.annotate('Noise pulse on trigger\\ncoincides with previous\\nspark output', 
                           xy=(spark_0_time/1000, 1.5), xytext=(spark_0_time/1000 + 3, 2.8),
                           fontsize=7, color='orange', ha='center',
                           arrowprops=dict(arrowstyle='->', color='orange', lw=1))
        
        # Generate spark signal
        spark = np.ones_like(t) * 3
        for i in range(2):
            trigger_time = i * period_us
            
            # Calculate dwell start time based on timing mode
            if dwell_delay_us > 500:
                # SAME LOBE: dwell starts after trigger within same period
                dwell_start = trigger_time + dwell_delay_us
            elif is_previous_lobe:
                # PREVIOUS LOBE: dwell starts in PREVIOUS 180° period
                if i == 0:
                    # For first period, show dwell starting before displayed time
                    # Use the converted delay which is from previous trigger
                    dwell_start = trigger_time - (period_us - dwell_delay_us)
                else:
                    # For second period, show dwell starting from previous trigger
                    prev_trigger_time = (i-1) * period_us
                    dwell_start = prev_trigger_time + dwell_delay_us
            else:
                # INSTANT DWELL: starts immediately at trigger (can't schedule in past)
                dwell_start = trigger_time
            
            # Calculate spark time
            spark_time = dwell_start + dwell_us
            
            # Only show if within plot range and dwell_start is reasonable
            if dwell_start >= 0 and dwell_start < t_max and spark_time < t_max:
                spark[(t >= dwell_start) & (t <= spark_time)] = 0
        
        # Plot signals
        ax.plot(t/1000, trigger, 'b-', linewidth=2, label='Trigger')
        ax.plot(t/1000, spark + 0.1, 'r-', linewidth=2, label='Spark')
        
        # Add labels and formatting
        title = f"{rpm:.0f} RPM - {advance_deg:.1f}° advance - {mode_text}"
        if is_previous_lobe and original_delay < 0:
            title += f" ({abs(original_delay):.0f}μs early)"
        elif not is_previous_lobe and not show_noise_filtering and dwell_delay_us < 0:
            title += f" ({abs(dwell_delay_us):.0f}μs behind)"
        
        ax.set_title(title, fontsize=10, fontweight='bold' if mode_text != "SAME LOBE" else 'normal')
        
        # Add mode description as subtitle
        ax.text(0.5, 0.95, mode_description, transform=ax.transAxes,
                ha='center', va='top', fontsize=8, style='italic')
        ax.set_ylabel('Signal', fontsize=9)
        ax.set_ylim(-0.5, 3.8)
        ax.set_xlim(0, t_max/1000)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
        
        # Add dwell duration annotation
        ax.annotate(f'{dwell_us/1000:.1f}ms dwell', 
                   xy=(0.02, 0.5), xycoords='axes fraction',
                   fontsize=8, color='red')
    
    axes[-1].set_xlabel('Time (ms)', fontsize=9)
    
    plt.suptitle('Ignition Timing Waveforms', fontsize=12, fontweight='bold')
    plt.tight_layout()
    plt.savefig('timing_waveforms.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("Generated timing_waveforms.png")

def print_summary(results: List[Dict]):
    """Print analysis summary."""
    if not results:
        print("No results to summarize")
        return
    
    # Separate noise from valid triggers
    noise_triggers = [r for r in results if r.get('is_noise', False)]
    valid_triggers = [r for r in results if not r.get('is_noise', False)]
    
    total_triggers = len(results)
    found_sparks = sum(1 for r in valid_triggers if r['spark_found'])
    missing_sparks = sum(1 for r in valid_triggers if not r['spark_found'])
    
    print("\n" + "="*50)
    print("TIMING ANALYSIS SUMMARY")
    print("="*50)
    print(f"Total trigger pulses analyzed: {total_triggers}")
    print(f"  Valid triggers: {len(valid_triggers)}")
    print(f"  Noise triggers: {len(noise_triggers)} (excluded from analysis)")
    
    if valid_triggers:
        print(f"\nValid trigger analysis:")
        print(f"  Sparks found: {found_sparks} ({100*found_sparks/len(valid_triggers):.1f}%)")
        print(f"  Sparks missing: {missing_sparks} ({100*missing_sparks/len(valid_triggers):.1f}%)")
    
    if found_sparks > 0:
        found_results = [r for r in valid_triggers if r['spark_found']]
        rpms = [r['rpm'] for r in found_results]
        advances = [r['advance_degrees'] for r in found_results]
        errors = [r['error_degrees'] for r in found_results]
        
        print(f"\nRPM range: {min(rpms):.0f} - {max(rpms):.0f}")
        print(f"Advance range: {min(advances):.1f}° - {max(advances):.1f}°")
        print(f"Mean error: {np.mean(errors):.2f}°")
        print(f"Error std dev: {np.std(errors):.2f}°")
        
        # Count dwell delay statistics
        dwell_delays = [r['dwell_delay_us'] for r in found_results if r.get('dwell_delay_us') is not None]
        if dwell_delays:
            positive_delays = sum(1 for d in dwell_delays if d > 0)
            negative_delays = sum(1 for d in dwell_delays if d < 0)
            print(f"\nDwell delays: {positive_delays} positive, {negative_delays} negative")
            print(f"Dwell delay range: {min(dwell_delays):.1f} to {max(dwell_delays):.1f} μs")
    
    if missing_sparks > 0:
        missing_results = [r for r in valid_triggers if not r['spark_found']]
        missing_rpms = [r['rpm'] for r in missing_results]
        print(f"\nMissing sparks at RPMs: {sorted(set(int(r) for r in missing_rpms))}")
    
    if noise_triggers:
        noise_reasons = {}
        for r in noise_triggers:
            reason = r.get('noise_reason', 'unknown')
            noise_reasons[reason] = noise_reasons.get(reason, 0) + 1
        print(f"\nNoise trigger breakdown:")
        for reason, count in sorted(noise_reasons.items()):
            print(f"  {reason}: {count}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python timing_analyzer.py <vcd_file>")
        sys.exit(1)
    
    vcd_file = sys.argv[1]
    results = analyze_triggers(vcd_file)
    
    # Generate output
    generate_csv_output(results)
    create_timing_plot(results)
    print_summary(results)
    
    print("\nAnalysis complete!")
    print("Generated files:")
    print("- wokwi-logic-analysis.csv")
    print("- timing_vs_rpm.png")