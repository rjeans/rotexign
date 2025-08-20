#!/usr/bin/env python3
"""
Extract VCD data for plotting D0 (input) and D1 (output) signals
Outputs CSV format: time_seconds, D0_state, D1_state
"""

import sys
import re

def parse_vcd_to_csv(vcd_file, output_file=None):
    """Extract D0 and D1 signals from VCD file to CSV format"""
    
    if output_file is None:
        output_file = vcd_file.replace('.vcd', '_signals.csv')
    
    # Signal mappings (from VCD header)
    signals = {
        '!': 'D0',  # Trigger input
        '"': 'D1',  # Spark output  
        '#': 'D2'   # Dwell marker
    }
    
    # Track signal states
    signal_states = {'D0': 0, 'D1': 0, 'D2': 0}
    
    # Data storage
    timeline = []
    
    print(f"Parsing VCD file: {vcd_file}")
    
    with open(vcd_file, 'r') as f:
        in_data_section = False
        current_time = 0
        
        for line in f:
            line = line.strip()
            
            # Skip header until we reach data section
            if line == '$enddefinitions $end':
                in_data_section = True
                continue
                
            if not in_data_section:
                continue
            
            # Parse time stamps
            if line.startswith('#'):
                current_time = int(line[1:])  # Remove '#' and convert to int (nanoseconds)
                continue
            
            # Parse signal changes  
            if len(line) >= 2:
                value = line[0]  # '0' or '1'
                signal_id = line[1]  # '!', '"', or '#'
                
                if signal_id in signals:
                    signal_name = signals[signal_id]
                    signal_states[signal_name] = int(value)
                    
                    # Record this state change
                    time_seconds = current_time / 1e9  # Convert ns to seconds
                    timeline.append({
                        'time': time_seconds,
                        'D0': signal_states['D0'],
                        'D1': signal_states['D1'], 
                        'D2': signal_states['D2']
                    })
    
    # Write CSV output
    print(f"Writing CSV data to: {output_file}")
    with open(output_file, 'w') as f:
        f.write("time_seconds,D0,D1,D2\n")
        for point in timeline:
            f.write(f"{point['time']:.6f},{point['D0']},{point['D1']},{point['D2']}\n")
    
    print(f"Extracted {len(timeline)} signal transitions")
    print(f"Time range: {timeline[0]['time']:.3f}s to {timeline[-1]['time']:.3f}s")
    
    return output_file

def create_plot_ready_data(csv_file, time_start=0, time_end=None, output_file=None):
    """Convert state changes to plot-ready step function data"""
    
    if output_file is None:
        output_file = csv_file.replace('.csv', '_plot.csv')
    
    import csv
    
    # Read the signal transitions
    transitions = []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            time = float(row['time_seconds'])
            if time_end is None or (time >= time_start and time <= time_end):
                transitions.append({
                    'time': time,
                    'D0': int(row['D0']),
                    'D1': int(row['D1']),
                    'D2': int(row['D2'])
                })
    
    # Create step function data for plotting
    plot_data = []
    
    if not transitions:
        print("No data in specified time range")
        return output_file
    
    # Add initial state
    current_state = {'D0': transitions[0]['D0'], 'D1': transitions[0]['D1'], 'D2': transitions[0]['D2']}
    plot_data.append([time_start, current_state['D0'], current_state['D1'], current_state['D2']])
    
    for transition in transitions:
        time = transition['time']
        
        # Add point just before transition (to create step)
        plot_data.append([time - 1e-9, current_state['D0'], current_state['D1'], current_state['D2']])
        
        # Update state
        current_state['D0'] = transition['D0']
        current_state['D1'] = transition['D1'] 
        current_state['D2'] = transition['D2']
        
        # Add point at transition
        plot_data.append([time, current_state['D0'], current_state['D1'], current_state['D2']])
    
    # Write plot-ready CSV
    print(f"Writing plot-ready data to: {output_file}")
    with open(output_file, 'w') as f:
        f.write("time_seconds,D0,D1,D2\n")
        for point in plot_data:
            f.write(f"{point[0]:.9f},{point[1]},{point[2]},{point[3]}\n")
    
    print(f"Created {len(plot_data)} plot points for time range {time_start:.1f}s to {time_end or 'end'}s")
    
    return output_file

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 extract_vcd_data.py <vcd_file> [start_time] [end_time]")
        print("Example: python3 extract_vcd_data.py wokwi-logic.vcd 10 20")
        sys.exit(1)
    
    vcd_file = sys.argv[1]
    start_time = float(sys.argv[2]) if len(sys.argv) > 2 else 0
    end_time = float(sys.argv[3]) if len(sys.argv) > 3 else None
    
    # Extract signal transitions
    csv_file = parse_vcd_to_csv(vcd_file)
    
    # Create plot-ready data for specified time range
    plot_file = create_plot_ready_data(csv_file, start_time, end_time)
    
    print(f"\nFiles created:")
    print(f"  Transitions: {csv_file}")
    print(f"  Plot data:   {plot_file}")
    print(f"\nTo plot in Python:")
    print(f"  import pandas as pd")
    print(f"  import matplotlib.pyplot as plt")
    print(f"  df = pd.read_csv('{plot_file}')")
    print(f"  plt.plot(df['time_seconds'], df['D0'] + 2, label='D0 (Trigger)')")
    print(f"  plt.plot(df['time_seconds'], df['D1'], label='D1 (Spark)')")
    print(f"  plt.xlabel('Time (seconds)')")
    print(f"  plt.ylabel('Signal Level')")
    print(f"  plt.legend()")
    print(f"  plt.grid(True)")
    print(f"  plt.show()")