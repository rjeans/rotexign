#!/usr/bin/env python3
"""
VCD Timing Analyzer with Comprehensive Plotting
Measures trigger falling edge to spark falling edge timing
Generates timing curves, waveforms, and analysis plots with scatter points
"""
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict

# Signal mappings for VCD parsing
SIGNAL_MAPPINGS = {
    '0#': 'trigger_fall',
    '1#': 'trigger_rise',
    '0$': 'spark_fall',
    '1$': 'dwell_start',
    '0!': 'trigger_fall',  # Legacy
    '1!': 'trigger_rise',  # Legacy
    '0"': 'spark_fall',    # Legacy
    '1"': 'dwell_start'    # Legacy
}

def parse_vcd_file(vcd_file: str) -> List[Tuple[int, str]]:
    """
    Parse the VCD file and extract signal changes.

    Args:
        vcd_file: Path to the VCD file.

    Returns:
        A list of tuples containing the time and event type.
    """
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

def find_previous_event(signal_changes: List[Tuple[int, str]], start_index: int, event_type: str) -> int:
    """
    Find the most recent event of a given type before a specific index.

    Args:
        signal_changes: List of signal changes.
        start_index: Index to start searching backward from.
        event_type: The event type to search for.

    Returns:
        The time of the most recent event, or None if not found.
    """
    for j in range(start_index - 1, -1, -1):
        if signal_changes[j][1] == event_type:
            return signal_changes[j][0]
    return None

def calculate_rpm(period_ns: int, pulses_per_revolution: int = 2) -> float:
    """
    Calculate RPM from a period in nanoseconds.

    Args:
        period_ns: The period in nanoseconds.
        pulses_per_revolution: Number of pulses per revolution (default: 2).

    Returns:
        The calculated RPM.
    """
    period_us = period_ns / 1000.0
    return (60_000_000.0 / period_us) / pulses_per_revolution

def find_corresponding_dwell(signal_changes: List[Tuple[int, str]], spark_idx: int, spark_time: int) -> float:
    """
    Find the dwell start corresponding to a given spark event.

    Args:
        signal_changes: List of signal changes.
        spark_idx: Index of the spark event.
        spark_time: Time of the spark event.

    Returns:
        The dwell time in microseconds, or None if not found.
    """
    for k in range(spark_idx - 1, -1, -1):
        check_time, check_event = signal_changes[k]
        if spark_time - check_time > 200000000:  # Stop if too far back (200ms)
            break
        if check_event == 'dwell_start':
            spark_between = any(
                signal_changes[m][1] == 'spark_fall'
                for m in range(k + 1, spark_idx)
            )
            if not spark_between:
                return (spark_time - check_time) / 1000.0
    return None

def find_trigger_pulse_width(signal_changes, trigger_fall_idx):
    """Find the trigger pulse width by looking for the next trigger_rise."""
    if trigger_fall_idx >= len(signal_changes) - 1:
        return None
    
    trigger_fall_time = signal_changes[trigger_fall_idx][0]
    
    for j in range(trigger_fall_idx + 1, min(len(signal_changes), trigger_fall_idx + 10)):
        check_time, check_event = signal_changes[j]
        if check_event == 'trigger_rise':
            return (check_time - trigger_fall_time) / 1000.0  # Return in microseconds
    return None

def analyze_vcd(vcd_file: str) -> Tuple[List[Dict], List[Dict]]:
    """
    Analyze VCD timing from first principles.

    Args:
        vcd_file: Path to the VCD file.

    Returns:
        A tuple containing timing events and missing spark events.
    """
    signal_changes = parse_vcd_file(vcd_file)
    print(f"Found {len(signal_changes)} signal changes")

    timing_events = []
    missing_spark_events = []

    for i, (time, event) in enumerate(signal_changes):
        if event != 'trigger_fall':
            continue

        # Find the next spark after this trigger
        next_spark = next((signal_changes[j] for j in range(i + 1, len(signal_changes)) if signal_changes[j][1] == 'spark_fall'), None)

        if next_spark is None:
            prev_trigger_time = find_previous_event(signal_changes, i, 'trigger_fall')
            if prev_trigger_time is not None:
                period_ns = time - prev_trigger_time
                rpm = calculate_rpm(period_ns)
                missing_spark_events.append({
                    'time_sec': time / 1e9,
                    'rpm': rpm,
                    'reason': 'No spark found after trigger'
                })
            continue

        spark_time, _ = next_spark
        delay_ns = spark_time - time
        delay_us = delay_ns / 1000.0

        if delay_us > 100000:  # Over 100ms delay indicates over-rev
            continue

        prev_trigger_time = find_previous_event(signal_changes, i, 'trigger_fall')
        if prev_trigger_time is None:
            continue

        period_ns = time - prev_trigger_time
        rpm = calculate_rpm(period_ns)

        degrees_per_us = 180.0 / (period_ns / 1000.0)
        delay_degrees = delay_us * degrees_per_us
        advance_degrees = 47.0 - delay_degrees

        REQUIRED_DWELL_US = 3000.0
        is_previous_lobe = delay_us < REQUIRED_DWELL_US

        spark_idx = next((idx for idx, (sig_time, sig_event) in enumerate(signal_changes) if sig_time == spark_time and sig_event == 'spark_fall'), None)
        actual_dwell_us = find_corresponding_dwell(signal_changes, spark_idx, spark_time) if spark_idx is not None else REQUIRED_DWELL_US
        trigger_pulse_width_us = find_trigger_pulse_width(signal_changes, i)

        timing_events.append({
            'time_sec': time / 1e9,
            'rpm': rpm,
            'delay_us': delay_us,
            'delay_degrees': delay_degrees,
            'advance_degrees': advance_degrees,
            'dwell_us': actual_dwell_us,
            'trigger_pulse_width_us': trigger_pulse_width_us,
            'is_previous_lobe': is_previous_lobe
        })

    print(f"\nFound {len(timing_events)} timing measurements")
    print(f"Found {len(missing_spark_events)} missing spark events")

    return timing_events, missing_spark_events

def get_theoretical_curves(rpm_range):
    """Calculate theoretical timing curves."""
    # Safe curve: conservative timing
    safe_curve = []
    # Performance curve: aggressive timing  
    perf_curve = []
    
    for rpm in rpm_range:
        if rpm <= 1000:
            safe_adv = rpm * 0.006  # 0-6° from 0-1000 RPM
            perf_adv = rpm * 0.006
        elif rpm <= 2000:
            safe_adv = 6 + (rpm - 1000) * 0.006  # 6-12° from 1000-2000 RPM
            perf_adv = 6 + (rpm - 1000) * 0.006
        elif rpm <= 3000:
            safe_adv = 12 + (rpm - 2000) * 0.003  # 12-15° from 2000-3000 RPM
            perf_adv = 12 + (rpm - 2000) * 0.008  # 12-20° from 2000-3000 RPM
        elif rpm <= 4000:
            safe_adv = 15  # Hold at 15°
            perf_adv = 20  # Hold at 20°
        elif rpm <= 5000:
            safe_adv = 15 - (rpm - 4000) * 0.001  # 15-14° from 4000-5000 RPM
            perf_adv = 20 - (rpm - 4000) * 0.001  # 20-19° from 4000-5000 RPM
        elif rpm <= 6000:
            safe_adv = 14 - (rpm - 5000) * 0.001  # 14-13° from 5000-6000 RPM
            perf_adv = 19 - (rpm - 5000) * 0.001  # 19-18° from 5000-6000 RPM
        else:
            safe_adv = 13 - (rpm - 6000) * 0.001  # 13-12° from 6000-7000 RPM
            perf_adv = 18 - (rpm - 6000) * 0.001  # 18-17° from 6000-7000 RPM
        
        safe_curve.append(max(0, safe_adv))
        perf_curve.append(max(0, perf_adv))
    
    return np.array(safe_curve), np.array(perf_curve)

def generate_csv_output(timing_events, missing_spark_events):
    """Generate CSV files with timing data."""
    
    # Main timing analysis CSV
    with open('wokwi-logic-analysis.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Calculate theoretical curves for comparison
        rpms = [event['rpm'] for event in timing_events]
        if rpms:
            rpm_range = np.linspace(min(rpms), max(rpms), 100)
            safe_curve, perf_curve = get_theoretical_curves(rpm_range)
            
            # Create lookup for theoretical values
            safe_interp = np.interp(rpms, rpm_range, safe_curve)
            perf_interp = np.interp(rpms, rpm_range, perf_curve)
        else:
            safe_interp = perf_interp = []
        
        # Header
        writer.writerow([
            'time_sec', 'rpm', 'delay_us', 'delay_degrees', 'advance_degrees',
            'dwell_us', 'trigger_pulse_width_us', 'is_previous_lobe', 'safe_advance', 'perf_advance'
        ])
        
        # Data rows
        for i, event in enumerate(timing_events):
            writer.writerow([
                f"{event['time_sec']:.3f}",
                f"{event['rpm']:.0f}",
                f"{event['delay_us']:.1f}",
                f"{event['delay_degrees']:.2f}",
                f"{event['advance_degrees']:.2f}",
                f"{event['dwell_us']:.1f}",
                f"{event['trigger_pulse_width_us']:.1f}" if event['trigger_pulse_width_us'] is not None else "N/A",
                event['is_previous_lobe'],
                f"{safe_interp[i]:.2f}" if i < len(safe_interp) else "0.00",
                f"{perf_interp[i]:.2f}" if i < len(perf_interp) else "0.00"
            ])
    
    # Missing sparks CSV
    with open('wokwi-logic-missing-sparks.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_sec', 'rpm', 'reason'])
        
        for event in missing_spark_events:
            writer.writerow([
                f"{event['time_sec']:.3f}",
                f"{event['rpm']:.0f}",
                event['reason']
            ])

def create_timing_vs_rpm_plot(timing_events):
    """Create ignition timing vs RPM plot with scatter points."""
    if not timing_events:
        print("No timing events to plot")
        return
    
    # Filter out outliers for better visualization
    filtered_events = []
    for event in timing_events:
        rpm = event['rpm']
        advance = event['advance_degrees']
        # Keep reasonable data: RPM 500-8000, advance -10° to 30°
        if 500 <= rpm <= 8000 and -10 <= advance <= 30:
            filtered_events.append(event)
    
    print(f"Filtered {len(timing_events) - len(filtered_events)} outlier points for plotting")
    
    rpms = np.array([event['rpm'] for event in filtered_events])
    advances = np.array([event['advance_degrees'] for event in filtered_events])
    
    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Generate theoretical curves
    rpm_range = np.linspace(0, 8000, 100)
    safe_curve, perf_curve = get_theoretical_curves(rpm_range)
    
    # Upper plot: Timing vs RPM with scatter points
    ax1.scatter(rpms, advances, alpha=0.3, s=20, c='lightblue', label='Individual measurements', zorder=1)
    
    # Calculate measured curve (binned averages with error bars)
    rpm_bins = np.arange(500, 8000, 100)
    bin_centers = rpm_bins[:-1] + 50
    bin_means = []
    bin_stds = []
    
    for i in range(len(rpm_bins)-1):
        mask = (rpms >= rpm_bins[i]) & (rpms < rpm_bins[i+1])
        if np.sum(mask) > 0:
            bin_means.append(np.mean(advances[mask]))
            bin_stds.append(np.std(advances[mask]))
        else:
            bin_means.append(np.nan)
            bin_stds.append(np.nan)
    
    # Remove NaN values for plotting
    valid_mask = ~np.isnan(bin_means)
    valid_centers = bin_centers[valid_mask]
    valid_means = np.array(bin_means)[valid_mask]
    valid_stds = np.array(bin_stds)[valid_mask]
    
    # Plot curves
    ax1.plot(rpm_range, safe_curve, '--', color='orange', linewidth=2, label='Safe curve')
    ax1.plot(rpm_range, perf_curve, '--', color='red', linewidth=2, label='Performance curve')
    ax1.errorbar(valid_centers, valid_means, yerr=valid_stds, fmt='o-', 
                color='blue', capsize=3, capthick=1, linewidth=2, 
                label='Measured (±1σ)', zorder=3)
    
    ax1.set_xlabel('RPM')
    ax1.set_ylabel('Ignition Advance (degrees BTDC)')
    ax1.set_title('Ignition Timing vs RPM')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_xlim(0, 8000)
    ax1.set_ylim(-5, 25)
    
    # Lower plot: Duty Cycle and Dwell Time vs RPM
    dwells = np.array([event['dwell_us'] for event in filtered_events])
    
    # Calculate duty cycle: dwell_time / period_time * 100
    periods_us = 60_000_000 / (rpms * 2)  # Period between triggers in microseconds
    duty_cycles = (dwells / periods_us) * 100
    
    # Create twin axis for dwell time
    ax2_twin = ax2.twinx()
    
    # Plot duty cycle (left axis)
    ax2.scatter(rpms, duty_cycles, alpha=0.3, s=20, c='lightgreen', label='Individual duty cycle', zorder=1)
    
    # Calculate duty cycle statistics by RPM bins
    duty_means = []
    duty_stds = []
    dwell_means = []
    dwell_stds = []
    
    for i in range(len(rpm_bins)-1):
        mask = (rpms >= rpm_bins[i]) & (rpms < rpm_bins[i+1])
        if np.sum(mask) > 0:
            duty_means.append(np.mean(duty_cycles[mask]))
            duty_stds.append(np.std(duty_cycles[mask]))
            dwell_means.append(np.mean(dwells[mask]))
            dwell_stds.append(np.std(dwells[mask]))
        else:
            duty_means.append(np.nan)
            duty_stds.append(np.nan)
            dwell_means.append(np.nan)
            dwell_stds.append(np.nan)
    
    # Remove NaN values for plotting
    valid_duty_mask = ~np.isnan(duty_means)
    valid_duty_centers = bin_centers[valid_duty_mask]
    valid_duty_means = np.array(duty_means)[valid_duty_mask]
    valid_duty_stds = np.array(duty_stds)[valid_duty_mask]
    valid_dwell_means = np.array(dwell_means)[valid_duty_mask]
    valid_dwell_stds = np.array(dwell_stds)[valid_duty_mask]
    
    # Plot duty cycle trend line (left axis)
    ax2.errorbar(valid_duty_centers, valid_duty_means, yerr=valid_duty_stds,
                fmt='o-', color='green', capsize=3, capthick=1, linewidth=2,
                label='Mean duty cycle ± std dev', zorder=3)
    
    # Plot dwell time scatter (right axis)
    ax2_twin.scatter(rpms, dwells/1000, alpha=0.3, s=20, c='lightcoral', label='Individual dwell time', zorder=1)
    
    # Plot dwell time trend line (right axis)
    ax2_twin.errorbar(valid_duty_centers, valid_dwell_means/1000, yerr=valid_dwell_stds/1000,
                     fmt='s-', color='red', capsize=3, capthick=1, linewidth=2,
                     label='Mean dwell time ± std dev', zorder=3)
    
    # Add reference lines
    ax2.axhline(y=40, color='orange', linestyle='--', alpha=0.7, label='40% max duty cycle')
    ax2_twin.axhline(y=3, color='blue', linestyle='--', alpha=0.7, label='Target 3ms dwell')
    
    # Configure axes
    ax2.set_xlabel('RPM')
    ax2.set_ylabel('Duty Cycle (%)', color='green')
    ax2_twin.set_ylabel('Dwell Time (ms)', color='red')
    ax2.set_title('Duty Cycle and Dwell Time vs RPM')
    ax2.grid(True, alpha=0.3)
    
    # Configure colors
    ax2.tick_params(axis='y', labelcolor='green')
    ax2_twin.tick_params(axis='y', labelcolor='red')
    
    # Legends
    ax2.legend(loc='upper left')
    ax2_twin.legend(loc='upper right')
    
    ax2.set_xlim(0, 8000)
    ax2.set_ylim(0, 50)  # 0-50% duty cycle
    ax2_twin.set_ylim(0, 5)  # 0-5ms dwell time
    
    plt.tight_layout()
    plt.savefig('timing_vs_rpm.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("Generated timing_vs_rpm.png")

def create_waveforms_plot(timing_events):
    """Create timing waveforms plot showing sample waveforms at different RPMs."""
    if not timing_events:
        print("No timing events to plot")
        return
    
    # Select representative RPM points
    rpms = np.array([event['rpm'] for event in timing_events])
    target_rpms = [1000, 2000, 4000, 6000]
    
    fig, axes = plt.subplots(len(target_rpms), 1, figsize=(12, 8))
    if len(target_rpms) == 1:
        axes = [axes]
    
    for i, target_rpm in enumerate(target_rpms):
        # Find closest measurement to target RPM
        closest_idx = np.argmin(np.abs(rpms - target_rpm))
        event = timing_events[closest_idx]
        actual_rpm = event['rpm']
        
        # Calculate period and timing
        period_us = 60_000_000 / (actual_rpm * 2)  # Period between triggers
        delay_us = event['delay_us']
        dwell_us = event['dwell_us'] if event['dwell_us'] else 3000
        trigger_pulse_width_us = event['trigger_pulse_width_us'] if event['trigger_pulse_width_us'] else 500
        
        # Create time axis (show ~2.5 periods)
        time_span_us = period_us * 2.5
        time_us = np.linspace(0, time_span_us, 1000)
        
        # Generate trigger signal (normally HIGH, brief LOW pulse at trigger times)
        trigger_signal = np.ones_like(time_us) * 3
        for trigger_time in [0, period_us, period_us * 2]:
            if trigger_time < time_span_us:
                # Add trigger falling edge (measured pulse width from VCD data)
                pulse_start = trigger_time
                pulse_end = trigger_time + trigger_pulse_width_us
                pulse_mask = (time_us >= pulse_start) & (time_us <= pulse_end)
                trigger_signal[pulse_mask] = 0
        
        # Generate spark signal (normally LOW, HIGH during dwell, brief LOW spike at spark)
        spark_signal = np.zeros_like(time_us)
        for trigger_time in [0, period_us, period_us * 2]:
            spark_time = trigger_time + delay_us
            dwell_start_time = spark_time - dwell_us
            
            if spark_time < time_span_us and dwell_start_time >= 0:
                # Add dwell period (HIGH = coil charging)
                dwell_mask = (time_us >= dwell_start_time) & (time_us < spark_time)
                spark_signal[dwell_mask] = 2
                
                # Spark event is the falling edge at spark_time (end of dwell)
                # The signal naturally drops to 0 after dwell ends
        
        # Plot signals
        ax = axes[i]
        ax.plot(time_us / 1000, trigger_signal, 'b-', linewidth=2, label='Trigger')
        ax.plot(time_us / 1000, spark_signal, 'r-', linewidth=2, label='Spark')
        
        # Add timing annotations
        advance_angle = event['advance_degrees']
        btdc_label = f"{advance_angle:.1f}° BTDC"
        dwell_label = f"{dwell_us/1000:.1f} ms dwell"
        
        ax.text(0.02, 0.95, btdc_label, transform=ax.transAxes, 
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7),
                verticalalignment='top')
        ax.text(0.02, 0.80, dwell_label, transform=ax.transAxes,
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7),
                verticalalignment='top')
        
        ax.set_ylabel('Signal')
        ax.set_title(f'{int(actual_rpm)} RPM (Same-lobe)' if not event['is_previous_lobe'] 
                    else f'{int(actual_rpm)} RPM (Previous-lobe) - {advance_angle:.1f}° advance - GOOD')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.5, 3.5)
        
        if i == 0:
            ax.legend()
        if i == len(target_rpms) - 1:
            ax.set_xlabel('Time (ms)')
    
    plt.suptitle('Ignition Timing Waveforms')
    plt.tight_layout()
    plt.savefig('timing_waveforms.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("Generated timing_waveforms.png")


if __name__ == "__main__":
    vcd_file = sys.argv[1] if len(sys.argv) > 1 else "wokwi-logic.vcd"
    timing_events, missing_spark_events = analyze_vcd(vcd_file)
    
    # Generate CSV output files
    generate_csv_output(timing_events, missing_spark_events)
    print("Generated CSV files: wokwi-logic-analysis.csv, wokwi-logic-missing-sparks.csv")
    
    # Generate plots
    create_timing_vs_rpm_plot(timing_events)
    create_waveforms_plot(timing_events)
    
    print("\nAnalysis complete! Generated:")
    print("- CSV files with timing data")
    print("- timing_vs_rpm.png (with scatter points)")
    print("- timing_waveforms.png")
    
    # Summary statistics
    if timing_events:
        rpms = [event['rpm'] for event in timing_events]
        advances = [event['advance_degrees'] for event in timing_events]
        print(f"\nSummary:")
        print(f"RPM range: {min(rpms):.0f} - {max(rpms):.0f}")
        print(f"Advance range: {min(advances):.1f}° - {max(advances):.1f}° BTDC")
        print(f"Timing measurements: {len(timing_events)}")
        print(f"Missing spark events: {len(missing_spark_events)}")