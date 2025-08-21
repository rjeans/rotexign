#!/usr/bin/env python3
"""
Create SVG timing plot for README (pure Python, no dependencies)
"""

import csv
import sys
from xml.dom.minidom import Document

def load_csv_data(csv_file, max_points=2000):
    """Load CSV data and downsample if needed for SVG performance"""
    data = {'time_seconds': [], 'D0': [], 'D1': [], 'D2': []}
    
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        all_data = list(reader)
    
    # Downsample if too many points
    step = max(1, len(all_data) // max_points)
    
    for i in range(0, len(all_data), step):
        row = all_data[i]
        data['time_seconds'].append(float(row['time_seconds']))
        data['D0'].append(int(row['D0']))
        data['D1'].append(int(row['D1']))
        data['D2'].append(int(row['D2']))
    
    return data

def create_svg_timing_plot(csv_file, output_svg="ignition_timing_detail.svg"):
    """Create SVG timing diagram"""
    
    # Load and process data
    data = load_csv_data(csv_file)
    
    if len(data['time_seconds']) == 0:
        print("No data found")
        return
    
    # SVG dimensions
    width = 800
    height = 400
    margin = 60
    plot_width = width - 2 * margin
    plot_height = height - 2 * margin
    
    # Time scaling
    time_min = min(data['time_seconds'])
    time_max = max(data['time_seconds'])
    time_range = time_max - time_min
    
    def time_to_x(t):
        return margin + ((t - time_min) / time_range) * plot_width
    
    def signal_to_y(signal_val, offset):
        y_pos = height - margin - (offset + signal_val) * (plot_height / 5)
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
        .grid { stroke: #ddd; stroke-width: 0.5; }
        .axis { stroke: #333; stroke-width: 1; }
        .d0-line { stroke: #2E86C1; stroke-width: 2; fill: none; }
        .d1-line { stroke: #E74C3C; stroke-width: 2; fill: none; }
        .d2-line { stroke: #27AE60; stroke-width: 2; fill: none; }
        .label { font-family: Arial, sans-serif; font-size: 12px; }
        .title { font-family: Arial, sans-serif; font-size: 16px; font-weight: bold; }
        .annotation { font-family: Arial, sans-serif; font-size: 10px; }
    '''))
    svg.appendChild(style)
    
    # Background
    bg = doc.createElement('rect')
    bg.setAttribute('width', str(width))
    bg.setAttribute('height', str(height))
    bg.setAttribute('fill', 'white')
    svg.appendChild(bg)
    
    # Grid lines (vertical)
    for i in range(6):
        x = margin + i * (plot_width / 5)
        line = doc.createElement('line')
        line.setAttribute('x1', str(x))
        line.setAttribute('y1', str(margin))
        line.setAttribute('x2', str(x))
        line.setAttribute('y2', str(height - margin))
        line.setAttribute('class', 'grid')
        svg.appendChild(line)
    
    # Axes
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
    
    # Signal paths
    def create_signal_path(signal_data, offset, class_name):
        path_data = []
        for i, (t, val) in enumerate(zip(data['time_seconds'], signal_data)):
            x = time_to_x(t)
            y = signal_to_y(val, offset)
            if i == 0:
                path_data.append(f'M {x:.2f} {y:.2f}')
            else:
                path_data.append(f'L {x:.2f} {y:.2f}')
        
        path = doc.createElement('path')
        path.setAttribute('d', ' '.join(path_data))
        path.setAttribute('class', class_name)
        svg.appendChild(path)
    
    # Plot signals with offsets
    create_signal_path([d + 3.5 for d in data['D0']], 0, 'd0-line')  # Trigger (top)
    create_signal_path([d + 1.5 for d in data['D1']], 0, 'd1-line')  # Spark (middle)
    create_signal_path([d + 0 for d in data['D2']], 0, 'd2-line')     # Dwell (bottom)
    
    # Labels
    labels = [
        ('Trigger', 3.5, '#2E86C1'),
        ('Spark', 1.5, '#E74C3C'),
        ('Dwell', 0, '#27AE60')
    ]
    
    for label_text, offset, color in labels:
        y_pos = signal_to_y(1, offset)
        
        # Label text
        text = doc.createElement('text')
        text.setAttribute('x', str(margin - 10))
        text.setAttribute('y', str(y_pos))
        text.setAttribute('text-anchor', 'end')
        text.setAttribute('dominant-baseline', 'middle')
        text.setAttribute('class', 'label')
        text.setAttribute('fill', color)
        text.appendChild(doc.createTextNode(label_text))
        svg.appendChild(text)
    
    # Title
    title = doc.createElement('text')
    title.setAttribute('x', str(width / 2))
    title.setAttribute('y', '25')
    title.setAttribute('text-anchor', 'middle')
    title.setAttribute('class', 'title')
    title.appendChild(doc.createTextNode('Arduino Ignition Controller - Timing Signals'))
    svg.appendChild(title)
    
    # Time axis labels
    for i in range(6):
        x = margin + i * (plot_width / 5)
        t = time_min + i * (time_range / 5)
        
        text = doc.createElement('text')
        text.setAttribute('x', str(x))
        text.setAttribute('y', str(height - margin + 20))
        text.setAttribute('text-anchor', 'middle')
        text.setAttribute('class', 'annotation')
        text.appendChild(doc.createTextNode(f'{t:.2f}s'))
        svg.appendChild(text)
    
    # X-axis label
    x_label = doc.createElement('text')
    x_label.setAttribute('x', str(width / 2))
    x_label.setAttribute('y', str(height - 10))
    x_label.setAttribute('text-anchor', 'middle')
    x_label.setAttribute('class', 'label')
    x_label.appendChild(doc.createTextNode('Time (seconds)'))
    svg.appendChild(x_label)
    
    # Performance summary box
    summary_text = [
        f"Time Range: {time_min:.2f}s - {time_max:.2f}s",
        f"Data Points: {len(data['time_seconds'])}",
        "* Trigger Input (D0) - Blue",
        "* Spark Output (D1) - Red", 
        "* Dwell Marker (D2) - Green"
    ]
    
    # Summary box background
    box = doc.createElement('rect')
    box.setAttribute('x', str(width - 200))
    box.setAttribute('y', '50')
    box.setAttribute('width', '180')
    box.setAttribute('height', str(len(summary_text) * 15 + 10))
    box.setAttribute('fill', '#f0f8ff')
    box.setAttribute('stroke', '#ddd')
    box.setAttribute('stroke-width', '1')
    svg.appendChild(box)
    
    # Summary text
    for i, line in enumerate(summary_text):
        text = doc.createElement('text')
        text.setAttribute('x', str(width - 190))
        text.setAttribute('y', str(65 + i * 15))
        text.setAttribute('class', 'annotation')
        text.appendChild(doc.createTextNode(line))
        svg.appendChild(text)
    
    # Write SVG file
    with open(output_svg, 'w') as f:
        doc.writexml(f, indent='  ', addindent='  ', newl='\n')
    
    print(f"SVG timing plot created: {output_svg}")
    print(f"Time range: {time_min:.3f}s to {time_max:.3f}s")
    print(f"Data points plotted: {len(data['time_seconds'])}")
    
    return output_svg

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 create_svg_plot.py <csv_file>")
        print("Example: python3 create_svg_plot.py wokwi-logic_signals_plot.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    svg_file = create_svg_timing_plot(csv_file)
    
    print(f"\nSVG created for README:")
    print(f"  File: {svg_file}")
    print(f"\nTo include in README.md:")
    print(f"  ![Ignition Timing Signals]({svg_file})")