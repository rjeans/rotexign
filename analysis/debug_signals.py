#!/usr/bin/env python3
"""
Debug script to check VCD signal parsing around trigger 49
"""

def parse_vcd_around_trigger_49():
    """Parse VCD signals around trigger 49 to verify correct identification."""

    # Signal mappings from the analyzer
    SIGNAL_MAPPINGS = {
        '0!': 'trigger_fall',   # D2 falling edge
        '1!': 'trigger_rise',   # D2 rising edge
        '0"': 'dwell_start',    # D3 falling edge (dwell start)
        '1"': 'spark_fire',     # D3 rising edge (spark fire)
    }

    # Parse around trigger 49 (3098ms = 3098000000ns)
    target_time = 3098000000
    window = 10000000  # 10ms window

    signal_changes = []
    current_time = 0

    with open('wokwi-logic.vcd', 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#'):
                current_time = int(line[1:])
                # Only collect data in our window of interest
                if target_time - window <= current_time <= target_time + window:
                    pass  # We're in the window
                elif current_time > target_time + window:
                    break  # Past our window
            elif line in SIGNAL_MAPPINGS and target_time - window <= current_time <= target_time + window:
                signal_changes.append((current_time, line, SIGNAL_MAPPINGS[line]))
            elif target_time - window <= current_time <= target_time + window:
                # Check for unmapped signals that might be confused
                if line in ['0$', '1$', '0#', '1#']:
                    pin = 'D5' if '$' in line else 'D4'
                    edge = 'fall' if '0' in line else 'rise'
                    signal_changes.append((current_time, line, f'{pin}_{edge}'))

    # Sort by time and display
    signal_changes.sort(key=lambda x: x[0])

    print("Signal changes around trigger 49:")
    print("Time (ns)     | Signal | Event")
    print("-" * 40)

    for time_ns, raw_signal, event in signal_changes:
        time_ms = time_ns / 1_000_000
        print(f"{time_ns:13d} | {raw_signal:6s} | {event}")
        print(f"{time_ms:10.3f}ms |        |")

    # Find trigger 49 specifically
    trigger_falls = [(t, s, e) for t, s, e in signal_changes if e == 'trigger_fall']
    if len(trigger_falls) >= 1:
        trigger_49_time = trigger_falls[0][0]  # First trigger fall in our window
        print(f"\nTrigger 49 at: {trigger_49_time} ns ({trigger_49_time/1_000_000:.3f} ms)")

        # Find corresponding spark
        sparks = [(t, s, e) for t, s, e in signal_changes if e == 'spark_fire' and t > trigger_49_time]
        if sparks:
            spark_time = sparks[0][0]
            delay_us = (spark_time - trigger_49_time) / 1000
            print(f"Next spark at: {spark_time} ns ({spark_time/1_000_000:.3f} ms)")
            print(f"Delay: {delay_us:.1f} μs")
        else:
            print("No spark found after trigger 49")

if __name__ == "__main__":
    parse_vcd_around_trigger_49()