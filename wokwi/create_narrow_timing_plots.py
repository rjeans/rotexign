#!/usr/bin/env python3
"""
Create narrow timing plots showing 3-4 periods at different RPM ranges
Only displays Trigger (D0) and Spark (D1) signals
"""

import csv
import sys
from xml.dom.minidom import Document

def load_csv_data(csv_file):
    """Load CSV data"""
    data = {'time_seconds': [], 'D0': [], 'D1': []}
    
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data['time_seconds'].append(float(row['time_seconds']))
            data['D0'].append(int(row['D0']))
            data['D1'].append(int(row['D1']))
    
    return data

def find_rpm_window(data, target_time, window_duration=0.2):
    """Find a time window around target_time"""
    start_time = target_time
    end_time = target_time + window_duration
    
    window_data = {'time_seconds': [], 'D0': [], 'D1': []}
    
    for i, t in enumerate(data['time_seconds']):
        if start_time <= t <= end_time:
            window_data['time_seconds'].append(t)
            window_data['D0'].append(data['D0'][i])
            window_data['D1'].append(data['D1'][i])
    
    return window_data

def count_periods(signal_data):
    """Count falling edges (periods) in signal"""
    periods = 0
    for i in range(1, len(signal_data)):
        if signal_data[i-1] == 1 and signal_data[i] == 0:
            periods += 1
    return periods

def estimate_rpm_from_period(window_data):
    """Estimate RPM from trigger period"""
    trigger_times = []
    
    for i in range(1, len(window_data['D0'])):
        if window_data['D0'][i-1] == 1 and window_data['D0'][i] == 0:
            trigger_times.append(window_data['time_seconds'][i])
    
    if len(trigger_times) >= 2:
        avg_period = sum(trigger_times[i+1] - trigger_times[i] 
                        for i in range(len(trigger_times)-1)) / (len(trigger_times)-1)
        rpm = 30.0 / avg_period  # 2 pulses per rev, so 30s/period = RPM
        return int(rpm)
    return 0

def create_narrow_svg(window_data, output_svg, rpm_label):
    """Create SVG with just Trigger and Spark signals"""
    
    if len(window_data['time_seconds']) == 0:
        print(f"No data for {output_svg}")
        return None
    
    # SVG dimensions
    width = 600
    height = 250
    margin = 50
    plot_width = width - 2 * margin
    plot_height = height - 2 * margin
    
    # Time scaling
    time_min = min(window_data['time_seconds'])
    time_max = max(window_data['time_seconds'])
    time_range = time_max - time_min
    
    # Make time relative to start
    rel_time = [(t - time_min) * 1000 for t in window_data['time_seconds']]  # Convert to ms
    
    def time_to_x(t_ms):
        return margin + (t_ms / (time_range * 1000)) * plot_width
    
    def signal_to_y(signal_val, offset):
        # offset: 1 for trigger (top), 0 for spark (bottom)
        y_pos = margin + (1 - offset) * plot_height/2 + (1 - signal_val) * plot_height/3
        return y_pos
    
    # Create SVG document
    doc = Document()
    svg = doc.createElement('svg')
    svg.setAttribute('width', str(width))
    svg.setAttribute('height', str(height))
    svg.setAttribute('viewBox', f'0 0 {width} {height}')
    svg.setAttribute('xmlns', 'http://www.w3.org/2000/svg')
    doc.appendChild(svg)
    
    # Add styles
    style = doc.createElement('style')
    style.appendChild(doc.createTextNode('''
        .grid { stroke: #e0e0e0; stroke-width: 0.5; }
        .axis { stroke: #333; stroke-width: 1; }
        .trigger-line { stroke: #2E86C1; stroke-width: 2; fill: none; }
        .spark-line { stroke: #E74C3C; stroke-width: 2; fill: none; }
        .label { font-family: Arial, sans-serif; font-size: 11px; }
        .title { font-family: Arial, sans-serif; font-size: 14px; font-weight: bold; }
        .timing-arrow { stroke: #8B008B; stroke-width: 1.5; fill: none; marker-end: url(#arrowhead); }
        .timing-label { font-family: Arial, sans-serif; font-size: 10px; fill: #8B008B; }
    '''))
    svg.appendChild(style)
    
    # Define arrowhead marker
    defs = doc.createElement('defs')
    marker = doc.createElement('marker')
    marker.setAttribute('id', 'arrowhead')
    marker.setAttribute('markerWidth', '10')
    marker.setAttribute('markerHeight', '7')
    marker.setAttribute('refX', '9')
    marker.setAttribute('refY', '3.5')
    marker.setAttribute('orient', 'auto')
    polygon = doc.createElement('polygon')
    polygon.setAttribute('points', '0 0, 10 3.5, 0 7')
    polygon.setAttribute('fill', '#8B008B')
    marker.appendChild(polygon)
    defs.appendChild(marker)
    svg.appendChild(defs)
    
    # Background
    bg = doc.createElement('rect')
    bg.setAttribute('width', str(width))
    bg.setAttribute('height', str(height))
    bg.setAttribute('fill', 'white')
    svg.appendChild(bg)
    
    # Draw grid lines
    for i in range(5):
        x = margin + i * (plot_width / 4)
        line = doc.createElement('line')
        line.setAttribute('x1', str(x))
        line.setAttribute('y1', str(margin))
        line.setAttribute('x2', str(x))
        line.setAttribute('y2', str(height - margin))
        line.setAttribute('class', 'grid')
        svg.appendChild(line)
    
    # Draw axes
    # X-axis
    x_axis = doc.createElement('line')
    x_axis.setAttribute('x1', str(margin))
    x_axis.setAttribute('y1', str(height - margin))
    x_axis.setAttribute('x2', str(width - margin))
    x_axis.setAttribute('y2', str(height - margin))
    x_axis.setAttribute('class', 'axis')
    svg.appendChild(x_axis)
    
    # Y-axis
    y_axis = doc.createElement('line')
    y_axis.setAttribute('x1', str(margin))
    y_axis.setAttribute('y1', str(margin))
    y_axis.setAttribute('x2', str(margin))
    y_axis.setAttribute('y2', str(height - margin))
    y_axis.setAttribute('class', 'axis')
    svg.appendChild(y_axis)
    
    # Plot Trigger signal (top)
    path_data = []
    for i, (t, val) in enumerate(zip(rel_time, window_data['D0'])):
        x = time_to_x(t)
        y = signal_to_y(val, 1)
        if i == 0:
            path_data.append(f'M {x:.2f} {y:.2f}')
        else:
            # Create step function
            prev_y = signal_to_y(window_data['D0'][i-1], 1)
            path_data.append(f'L {x:.2f} {prev_y:.2f}')
            path_data.append(f'L {x:.2f} {y:.2f}')
    
    trigger_path = doc.createElement('path')
    trigger_path.setAttribute('d', ' '.join(path_data))
    trigger_path.setAttribute('class', 'trigger-line')
    svg.appendChild(trigger_path)
    
    # Plot Spark signal (bottom)
    path_data = []
    for i, (t, val) in enumerate(zip(rel_time, window_data['D1'])):
        x = time_to_x(t)
        y = signal_to_y(val, 0)
        if i == 0:
            path_data.append(f'M {x:.2f} {y:.2f}')
        else:
            # Create step function
            prev_y = signal_to_y(window_data['D1'][i-1], 0)
            path_data.append(f'L {x:.2f} {prev_y:.2f}')
            path_data.append(f'L {x:.2f} {y:.2f}')
    
    spark_path = doc.createElement('path')
    spark_path.setAttribute('d', ' '.join(path_data))
    spark_path.setAttribute('class', 'spark-line')
    svg.appendChild(spark_path)
    
    # Find timing points for annotation
    trigger_falls = []
    spark_falls = []
    
    for i in range(1, len(window_data['D0'])):
        if window_data['D0'][i-1] == 1 and window_data['D0'][i] == 0:
            trigger_falls.append(rel_time[i])
        if window_data['D1'][i-1] == 1 and window_data['D1'][i] == 0:
            spark_falls.append(rel_time[i])
    
    # Annotate first timing relationship
    if len(trigger_falls) > 0 and len(spark_falls) > 0:
        # Find first spark after first trigger
        trigger_time = trigger_falls[0]
        spark_time = None
        for st in spark_falls:
            if st > trigger_time:
                spark_time = st
                break
        
        if spark_time:
            delay_ms = spark_time - trigger_time
            
            # Draw timing arrow
            arrow = doc.createElement('line')
            arrow.setAttribute('x1', str(time_to_x(trigger_time)))
            arrow.setAttribute('y1', str(signal_to_y(0.5, 0.5)))
            arrow.setAttribute('x2', str(time_to_x(spark_time)))
            arrow.setAttribute('y2', str(signal_to_y(0.5, 0.5)))
            arrow.setAttribute('class', 'timing-arrow')
            svg.appendChild(arrow)
            
            # Add delay label
            mid_x = time_to_x((trigger_time + spark_time) / 2)
            text = doc.createElement('text')
            text.setAttribute('x', str(mid_x))
            text.setAttribute('y', str(signal_to_y(0.5, 0.5) - 5))
            text.setAttribute('text-anchor', 'middle')
            text.setAttribute('class', 'timing-label')
            text.appendChild(doc.createTextNode(f'{delay_ms:.1f}ms'))
            svg.appendChild(text)
    
    # Labels
    # Trigger label
    trigger_label = doc.createElement('text')
    trigger_label.setAttribute('x', '15')
    trigger_label.setAttribute('y', str(signal_to_y(0.5, 1)))
    trigger_label.setAttribute('class', 'label')
    trigger_label.setAttribute('fill', '#2E86C1')
    trigger_label.appendChild(doc.createTextNode('TRIGGER'))
    svg.appendChild(trigger_label)
    
    # Spark label
    spark_label = doc.createElement('text')
    spark_label.setAttribute('x', '15')
    spark_label.setAttribute('y', str(signal_to_y(0.5, 0)))
    spark_label.setAttribute('class', 'label')
    spark_label.setAttribute('fill', '#E74C3C')
    spark_label.appendChild(doc.createTextNode('SPARK'))
    svg.appendChild(spark_label)
    
    # Title
    title = doc.createElement('text')
    title.setAttribute('x', str(width / 2))
    title.setAttribute('y', '20')
    title.setAttribute('text-anchor', 'middle')
    title.setAttribute('class', 'title')
    title.appendChild(doc.createTextNode(f'{rpm_label} RPM - Ignition Timing'))
    svg.appendChild(title)
    
    # Time axis labels
    for i in range(5):
        x = margin + i * (plot_width / 4)
        t = i * (time_range * 1000 / 4)
        
        text = doc.createElement('text')
        text.setAttribute('x', str(x))
        text.setAttribute('y', str(height - margin + 15))
        text.setAttribute('text-anchor', 'middle')
        text.setAttribute('class', 'label')
        text.appendChild(doc.createTextNode(f'{t:.0f}'))
        svg.appendChild(text)
    
    # X-axis label
    x_label = doc.createElement('text')
    x_label.setAttribute('x', str(width / 2))
    x_label.setAttribute('y', str(height - 15))
    x_label.setAttribute('text-anchor', 'middle')
    x_label.setAttribute('class', 'label')
    x_label.appendChild(doc.createTextNode('Time (ms)'))
    svg.appendChild(x_label)
    
    # Info box
    period_count = count_periods(window_data['D0'])
    info_text = f'{period_count} cycles shown'
    
    info = doc.createElement('text')
    info.setAttribute('x', str(width - margin))
    info.setAttribute('y', '35')
    info.setAttribute('text-anchor', 'end')
    info.setAttribute('class', 'label')
    info.setAttribute('fill', '#666')
    info.appendChild(doc.createTextNode(info_text))
    svg.appendChild(info)
    
    # Write SVG file
    with open(output_svg, 'w') as f:
        doc.writexml(f, indent='  ', addindent='  ', newl='\n')
    
    return output_svg

def main():
    """Generate narrow timing plots for 3 RPM ranges"""
    
    if len(sys.argv) < 2:
        print("Generating plots from default data file...")
        csv_file = "wokwi-logic_signals.csv"
    else:
        csv_file = sys.argv[1]
    
    # Load full dataset
    print(f"Loading data from {csv_file}...")
    data = load_csv_data(csv_file)
    
    # Define RPM windows (time, duration, expected RPM)
    # These times are chosen based on the VCD analysis to get specific RPM ranges
    rpm_windows = [
        (10.0, 0.3, "~800"),   # Low RPM - around 10 seconds
        (30.0, 0.15, "~2000"),  # Mid RPM - around 30 seconds  
        (70.0, 0.08, "~5500")   # High RPM - around 70 seconds
    ]
    
    created_files = []
    
    for start_time, duration, rpm_label in rpm_windows:
        # Extract window
        window = find_rpm_window(data, start_time, duration)
        
        # Count periods
        periods = count_periods(window['D0'])
        
        # Estimate actual RPM
        actual_rpm = estimate_rpm_from_period(window)
        
        print(f"\n{rpm_label} RPM window:")
        print(f"  Time: {start_time:.1f}s - {start_time + duration:.1f}s")
        print(f"  Periods captured: {periods}")
        print(f"  Estimated RPM: {actual_rpm}")
        
        # Create SVG
        output_file = f"timing_{rpm_label.replace('~', '').replace(' ', '_')}_rpm.svg"
        svg_file = create_narrow_svg(window, output_file, rpm_label)
        
        if svg_file:
            created_files.append(svg_file)
            print(f"  Created: {svg_file}")
    
    print(f"\n=== Summary ===")
    print(f"Created {len(created_files)} narrow timing plots:")
    for f in created_files:
        print(f"  - {f}")
    
    print(f"\nTo include in README:")
    for f in created_files:
        print(f"![{f.replace('_', ' ').replace('.svg', '').title()}](wokwi/{f})")

if __name__ == "__main__":
    main()