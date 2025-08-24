#!/usr/bin/env python3
"""
VCD Timing Analyzer - Optimized
Measures trigger falling edge to spark falling edge timing
"""
import sys
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

        timing_events.append({
            'time_sec': time / 1e9,
            'rpm': rpm,
            'delay_us': delay_us,
            'delay_degrees': delay_degrees,
            'advance_degrees': advance_degrees,
            'dwell_us': actual_dwell_us,
            'is_previous_lobe': is_previous_lobe
        })

    print(f"\nFound {len(timing_events)} timing measurements")
    print(f"Found {len(missing_spark_events)} missing spark events")

    return timing_events, missing_spark_events

if __name__ == "__main__":
    vcd_file = sys.argv[1] if len(sys.argv) > 1 else "wokwi-logic.vcd"
    timing_events, missing_spark_events = analyze_vcd(vcd_file)