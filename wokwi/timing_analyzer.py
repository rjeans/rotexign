#!/usr/bin/env python3
"""
Complete VCD Timing Analyzer for rotexign
Analyzes timing data and creates focused timing plots in one script

Features:
- Comprehensive VCD analysis with mode detection
- Timing accuracy assessment vs programmed curves
- Automatic generation of focused timing plots
- Single script for complete analysis workflow
"""

import sys
import statistics
import csv
from xml.dom.minidom import Document

class TimingAnalyzer:
    def __init__(self, vcd_filename):
        self.vcd_filename = vcd_filename
        self.timing_data = []
        self.HIGH_RPM_THRESHOLD = 6500  # From Arduino code
        self.TRIGGER_ANGLE_BTDC = 47.0
        self.PULSES_PER_REV = 2
        
    def analyze_vcd(self):
        """Parse VCD file and extract timing data"""
        print(f"Analyzing: {self.vcd_filename}")
        print(f"Previous-lobe scheduling threshold: {self.HIGH_RPM_THRESHOLD} RPM")
        
        # Parse VCD file
        with open(self.vcd_filename, 'r') as f:
            lines = f.readlines()
        
        # Skip to data section
        data_start = 0
        for i, line in enumerate(lines):
            if line.strip() == '$enddefinitions $end':
                data_start = i + 1
                break
        
        # Parse timing data
        current_time = 0
        trigger_times = []
        spark_events = []
        
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
        
        print(f"Found {len(trigger_times)} triggers, {len(spark_events)} spark events")
        
        # Analyze timing relationships
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
                rpm = (60_000_000.0 / period_us) / self.PULSES_PER_REV
                
                trigger_to_spark_ns = spark_time - trigger_time
                trigger_to_spark_us = trigger_to_spark_ns / 1000.0
                
                dwell_duration_ns = spark_time - dwell_start_time
                dwell_duration_us = dwell_duration_ns / 1000.0
                
                # Calculate advance angle
                degrees_per_us = 180.0 / period_us
                trigger_to_spark_degrees = trigger_to_spark_us * degrees_per_us
                advance_angle = self.TRIGGER_ANGLE_BTDC - trigger_to_spark_degrees
                
                # Determine scheduling mode
                scheduling_mode = "PREV" if rpm > self.HIGH_RPM_THRESHOLD else "SAME"
                
                self.timing_data.append({
                    'time_sec': spark_time / 1_000_000_000.0,
                    'rpm': rpm,
                    'advance_angle': advance_angle,
                    'trigger_to_spark_us': trigger_to_spark_us,
                    'dwell_duration_us': dwell_duration_us,
                    'scheduling_mode': scheduling_mode
                })
    
    def generate_analysis_report(self, output_file):
        """Generate comprehensive analysis report"""
        if not self.timing_data:
            print("No timing data available for analysis")
            return
        
        with open(output_file, 'w') as f:
            f.write(f"Analyzing: {self.vcd_filename}\n")
            f.write(f"Found {len(self.timing_data)} complete timing events\n\n")
            
            # Detailed timing curve
            f.write("="*90 + "\n")
            f.write("EXTRACTED TIMING CURVE\n")
            f.write("="*90 + "\n")
            f.write(f"{'Time':>8} {'RPM':>6} {'Advance':>8} {'Trigger→Spark':>13} {'Dwell':>8} {'Mode':>5}\n")
            f.write(f"{'(sec)':>8} {'':>6} {'(°BTDC)':>8} {'(μs)':>13} {'(μs)':>8} {'':>5}\n")
            f.write("-"*90 + "\n")
            
            for data in self.timing_data:
                f.write(f"{data['time_sec']:8.3f} {data['rpm']:6.0f} {data['advance_angle']:8.1f} "
                       f"{data['trigger_to_spark_us']:13.1f} {data['dwell_duration_us']:8.1f} "
                       f"{data['scheduling_mode']:>5}\n")
            
            # Mode analysis
            same_lobe_data = [d for d in self.timing_data if d['scheduling_mode'] == 'SAME']
            prev_lobe_data = [d for d in self.timing_data if d['scheduling_mode'] == 'PREV']
            
            f.write(f"\n" + "="*70 + "\n")
            f.write("SCHEDULING MODE ANALYSIS\n")
            f.write("="*70 + "\n")
            f.write(f"Same-lobe events: {len(same_lobe_data)}\n")
            f.write(f"Previous-lobe events: {len(prev_lobe_data)}\n")
            
            if prev_lobe_data:
                prev_rpms = [d['rpm'] for d in prev_lobe_data]
                f.write(f"Previous-lobe RPM range: {min(prev_rpms):.0f} - {max(prev_rpms):.0f}\n")
            
            # RPM binned analysis
            rpm_groups = {}
            for data in self.timing_data:
                rpm_bin = int(data['rpm'] / 500) * 500
                if rpm_bin not in rpm_groups:
                    rpm_groups[rpm_bin] = []
                rpm_groups[rpm_bin].append(data)
            
            f.write(f"\n" + "="*85 + "\n")
            f.write("TIMING CURVE SUMMARY (500 RPM bins)\n")
            f.write("="*85 + "\n")
            f.write(f"{'RPM Range':>12} {'Count':>6} {'Avg RPM':>8} {'Avg Advance':>12} {'Avg Dwell':>10} {'Mode':>6}\n")
            f.write(f"{'':>12} {'':>6} {'':>8} {'(°BTDC)':>12} {'(μs)':>10} {'':>6}\n")
            f.write("-"*85 + "\n")
            
            for rpm_bin in sorted(rpm_groups.keys()):
                if rpm_bin == 0:
                    continue
                    
                events = rpm_groups[rpm_bin]
                avg_rpm = statistics.mean([e['rpm'] for e in events])
                avg_advance = statistics.mean([e['advance_angle'] for e in events])
                avg_dwell = statistics.mean([e['dwell_duration_us'] for e in events])
                
                # Determine predominant mode
                same_count = sum(1 for e in events if e['scheduling_mode'] == 'SAME')
                prev_count = len(events) - same_count
                mode = "SAME" if same_count > prev_count else "PREV"
                
                f.write(f"{rpm_bin:>7}-{rpm_bin+499:<4} {len(events):6d} {avg_rpm:8.0f} "
                       f"{avg_advance:12.1f} {avg_dwell:10.1f} {mode:>6}\n")
            
            # Compare with programmed curves
            self._write_curve_comparison(f, rpm_groups)
    
    def _write_curve_comparison(self, f, rpm_groups):
        """Write curve comparison analysis"""
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
        
        f.write(f"\n" + "="*100 + "\n")
        f.write("COMPARISON: ACTUAL vs PROGRAMMED CURVES\n")
        f.write("="*100 + "\n")
        f.write(f"{'RPM':>6} {'Actual':>8} {'Safe':>8} {'Perf':>8} {'Error':>8} {'Curve':>8} {'Mode':>6} {'Status':>10}\n")
        f.write(f"{'':>6} {'(°BTDC)':>8} {'(°BTDC)':>8} {'(°BTDC)':>8} {'(Act-Safe)':>8} {'Used':>8} {'':>6} {'':>10}\n")
        f.write("-"*100 + "\n")
        
        all_errors = []
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
            all_errors.append(abs(error_safe))
            
            # Determine which curve is closer
            curve_used = "SAFE" if abs(error_safe) < abs(error_perf) else "PERF"
            
            # Determine mode
            same_count = sum(1 for e in events if e['scheduling_mode'] == 'SAME')
            prev_count = len(events) - same_count
            mode = "SAME" if same_count > prev_count else "PREV"
            
            # Status assessment
            if abs(error_safe) < 1.0:
                status = "EXCELLENT"
            elif abs(error_safe) < 2.0:
                status = "GOOD"
            elif abs(error_safe) < 5.0:
                status = "FAIR"
            else:
                status = "POOR"
            
            f.write(f"{avg_rpm:6.0f} {actual_advance:8.1f} {safe_advance:8.1f} "
                   f"{perf_advance:8.1f} {error_safe:+8.1f} {curve_used:>8} "
                   f"{mode:>6} {status:>10}\n")
        
        # Summary
        if all_errors:
            f.write(f"\n" + "="*50 + "\n")
            f.write("ACCURACY SUMMARY\n")
            f.write("="*50 + "\n")
            f.write(f"Average error: {statistics.mean(all_errors):5.2f} degrees\n")
            f.write(f"Maximum error: {max(all_errors):5.2f} degrees\n")
            f.write(f"RMS error: {(sum(e**2 for e in all_errors)/len(all_errors))**0.5:5.2f} degrees\n")
    
    def create_focused_plots(self):
        """Create focused timing plots at key RPM points"""
        # First, extract CSV data for plotting
        csv_file = self.vcd_filename.replace('.vcd', '-signals.csv')
        self._extract_vcd_to_csv(csv_file)
        
        # Load CSV data
        data = self._load_csv_data(csv_file)
        
        # Create plots at specific RPM points
        plot_configs = [
            (11.0, 3, 800, "Low RPM (~800) - Ignition Timing"),
            (30.0, 3, 2000, "Medium RPM (~2000) - Ignition Timing"), 
            (70.0, 4, 5500, "High RPM (~5500) - Ignition Timing"),
            (75.0, 4, 6500, "Very High RPM (~6500) - Previous-Lobe Mode")
        ]
        
        created_files = []
        for time_center, periods, expected_rpm, title in plot_configs:
            window = self._find_period_window(data, time_center, periods)
            
            if window:
                actual_rpm = self._estimate_rpm(window)
                print(f"Creating plot for {actual_rpm} RPM...")
                
                full_title = title.replace(f"~{expected_rpm}", str(actual_rpm))
                output_file = f"timing_{expected_rpm}rpm.svg"
                
                svg_file = self._create_clean_svg(window, output_file, full_title)
                if svg_file:
                    created_files.append(svg_file)
        
        return created_files
    
    def _extract_vcd_to_csv(self, csv_file):
        """Extract VCD data to CSV format for plotting"""
        with open(self.vcd_filename, 'r') as f:
            lines = f.readlines()
        
        # Find data section
        data_start = 0
        for i, line in enumerate(lines):
            if line.strip() == '$enddefinitions $end':
                data_start = i + 1
                break
        
        # Parse signal changes
        current_time = 0
        signals = {'D0': 1, 'D1': 0, 'D2': 0}  # Initial states
        data_points = []
        
        for line in lines[data_start:]:
            line = line.strip()
            if not line:
                continue
                
            if line.startswith('#'):
                current_time = int(line[1:])
            elif line in ['0!', '1!']:  # D0 changes
                signals['D0'] = int(line[0])
                data_points.append((current_time / 1_000_000_000.0, signals['D0'], signals['D1'], signals['D2']))
            elif line in ['0"', '1"']:  # D1 changes
                signals['D1'] = int(line[0])
                data_points.append((current_time / 1_000_000_000.0, signals['D0'], signals['D1'], signals['D2']))
            elif line in ['0#', '1#']:  # D2 changes
                signals['D2'] = int(line[0])
                data_points.append((current_time / 1_000_000_000.0, signals['D0'], signals['D1'], signals['D2']))
        
        # Write CSV
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_seconds', 'D0', 'D1', 'D2'])
            for point in data_points:
                writer.writerow(point)
    
    def _load_csv_data(self, csv_file):
        """Load CSV data for plotting"""
        data = {'time_seconds': [], 'D0': [], 'D1': []}
        
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                data['time_seconds'].append(float(row['time_seconds']))
                data['D0'].append(int(row['D0']))
                data['D1'].append(int(row['D1']))
        
        return data
    
    def _find_period_window(self, data, target_time, target_periods=3):
        """Find a window containing target_periods trigger cycles"""
        # Find trigger falling edges near target time
        trigger_edges = []
        for i in range(1, len(data['D0'])):
            if data['D0'][i-1] == 1 and data['D0'][i] == 0:
                trigger_edges.append(i)
        
        # Find edge closest to target_time
        closest_idx = 0
        min_diff = float('inf')
        for idx in trigger_edges:
            diff = abs(data['time_seconds'][idx] - target_time)
            if diff < min_diff:
                min_diff = diff
                closest_idx = trigger_edges.index(idx)
        
        # Get window with target_periods cycles
        if closest_idx + target_periods < len(trigger_edges):
            start_idx = trigger_edges[closest_idx]
            end_idx = trigger_edges[closest_idx + target_periods]
            
            # Add padding
            start_idx = max(0, start_idx - 20)
            end_idx = min(len(data['time_seconds']) - 1, end_idx + 20)
            
            return {
                'time_seconds': data['time_seconds'][start_idx:end_idx],
                'D0': data['D0'][start_idx:end_idx],
                'D1': data['D1'][start_idx:end_idx]
            }
        
        return None
    
    def _estimate_rpm(self, window_data):
        """Calculate RPM from trigger period"""
        trigger_times = []
        for i in range(1, len(window_data['D0'])):
            if window_data['D0'][i-1] == 1 and window_data['D0'][i] == 0:
                trigger_times.append(window_data['time_seconds'][i])
        
        if len(trigger_times) >= 2:
            periods = [trigger_times[i+1] - trigger_times[i] for i in range(len(trigger_times)-1)]
            avg_period = sum(periods) / len(periods)
            rpm = 30.0 / avg_period  # 2 pulses per rev
            return int(rpm)
        return 0
    
    def _create_clean_svg(self, window_data, output_svg, title):
        """Create clean SVG with trigger and spark signals"""
        if not window_data or len(window_data['time_seconds']) == 0:
            return None
        
        # SVG dimensions
        width = 800
        height = 300
        margin_left = 80
        margin_right = 50
        margin_top = 50
        margin_bottom = 50
        plot_width = width - margin_left - margin_right
        plot_height = height - margin_top - margin_bottom
        
        # Time scaling
        time_min = min(window_data['time_seconds'])
        time_max = max(window_data['time_seconds'])
        time_range_ms = (time_max - time_min) * 1000
        
        rel_time = [(t - time_min) * 1000 for t in window_data['time_seconds']]
        
        def time_to_x(t_ms):
            return margin_left + (t_ms / time_range_ms) * plot_width
        
        def signal_to_y(signal_val, is_trigger):
            if is_trigger:
                base_y = margin_top + plot_height * 0.25
                return base_y - signal_val * plot_height * 0.2
            else:
                base_y = margin_top + plot_height * 0.75
                return base_y - signal_val * plot_height * 0.2
        
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
            .grid { stroke: #e8e8e8; stroke-width: 1; stroke-dasharray: 2,2; }
            .trigger-line { stroke: #2E86C1; stroke-width: 2.5; fill: none; }
            .spark-line { stroke: #E74C3C; stroke-width: 2.5; fill: none; }
            .label { font-family: Arial, sans-serif; font-size: 14px; font-weight: bold; }
            .axis-label { font-family: Arial, sans-serif; font-size: 12px; }
            .title { font-family: Arial, sans-serif; font-size: 18px; font-weight: bold; }
            .timing-arrow { stroke: #7B68EE; stroke-width: 1.5; fill: none; stroke-dasharray: 5,2; }
            .timing-label { font-family: Arial, sans-serif; font-size: 11px; fill: #7B68EE; font-weight: bold; }
        '''))
        svg.appendChild(style)
        
        # White background
        bg = doc.createElement('rect')
        bg.setAttribute('width', str(width))
        bg.setAttribute('height', str(height))
        bg.setAttribute('fill', 'white')
        svg.appendChild(bg)
        
        # Grid lines
        for i in range(5):
            x = margin_left + i * (plot_width / 4)
            line = doc.createElement('line')
            line.setAttribute('x1', str(x))
            line.setAttribute('y1', str(margin_top))
            line.setAttribute('x2', str(x))
            line.setAttribute('y2', str(height - margin_bottom))
            line.setAttribute('class', 'grid')
            svg.appendChild(line)
        
        # Horizontal separator
        separator_y = margin_top + plot_height / 2
        sep_line = doc.createElement('line')
        sep_line.setAttribute('x1', str(margin_left))
        sep_line.setAttribute('y1', str(separator_y))
        sep_line.setAttribute('x2', str(width - margin_right))
        sep_line.setAttribute('y2', str(separator_y))
        sep_line.setAttribute('class', 'grid')
        svg.appendChild(sep_line)
        
        # Plot signals
        self._add_signal_path(doc, svg, rel_time, window_data['D0'], time_to_x, 
                             lambda v: signal_to_y(v, True), 'trigger-line')
        self._add_signal_path(doc, svg, rel_time, window_data['D1'], time_to_x,
                             lambda v: signal_to_y(v, False), 'spark-line')
        
        # Add timing annotations
        self._add_timing_annotations(doc, svg, window_data, rel_time, time_to_x, signal_to_y, separator_y)
        
        # Add labels and title
        self._add_plot_labels(doc, svg, title, width, height, margin_left, margin_top, 
                             plot_height, plot_width, time_range_ms)
        
        # Write SVG
        with open(output_svg, 'w') as f:
            doc.writexml(f, indent='  ', addindent='  ', newl='\n')
        
        return output_svg
    
    def _add_signal_path(self, doc, svg, time_data, signal_data, time_to_x, signal_to_y, class_name):
        """Add signal path to SVG"""
        path_data = []
        for i, (t, val) in enumerate(zip(time_data, signal_data)):
            x = time_to_x(t)
            y = signal_to_y(val)
            if i == 0:
                path_data.append(f'M {x:.2f} {y:.2f}')
            else:
                prev_y = signal_to_y(signal_data[i-1])
                if abs(prev_y - y) > 1:
                    path_data.append(f'L {x:.2f} {prev_y:.2f}')
                    path_data.append(f'L {x:.2f} {y:.2f}')
                else:
                    path_data.append(f'L {x:.2f} {y:.2f}')
        
        path_elem = doc.createElement('path')
        path_elem.setAttribute('d', ' '.join(path_data))
        path_elem.setAttribute('class', class_name)
        svg.appendChild(path_elem)
    
    def _add_timing_annotations(self, doc, svg, window_data, rel_time, time_to_x, signal_to_y, separator_y):
        """Add timing annotations to SVG"""
        # Find trigger and spark edges
        trigger_falls = []
        spark_falls = []
        
        for i in range(1, len(window_data['D0'])):
            if window_data['D0'][i-1] == 1 and window_data['D0'][i] == 0:
                trigger_falls.append(rel_time[i])
            if window_data['D1'][i-1] == 1 and window_data['D1'][i] == 0:
                spark_falls.append(rel_time[i])
        
        # Calculate RPM from trigger period
        rpm = 0
        if len(trigger_falls) >= 2:
            periods = [trigger_falls[i+1] - trigger_falls[i] for i in range(len(trigger_falls)-1)]
            avg_period_ms = sum(periods) / len(periods)
            rpm = 30000.0 / avg_period_ms  # Convert ms to RPM
        
        # Annotate first timing relationship
        if len(trigger_falls) > 0 and len(spark_falls) > 0:
            trigger_time = trigger_falls[0]
            spark_time = None
            for st in spark_falls:
                if st > trigger_time:
                    spark_time = st
                    break
            
            if spark_time and rpm > 0:
                delay_ms = spark_time - trigger_time
                
                # Calculate advance angle
                degrees_per_ms = 180.0 / avg_period_ms  # Half revolution per trigger period
                trigger_to_spark_degrees = delay_ms * degrees_per_ms
                advance_angle = self.TRIGGER_ANGLE_BTDC - trigger_to_spark_degrees
                
                # Determine timing quality and color
                if advance_angle < 0:
                    timing_color = '#E74C3C'  # Red - very poor (after TDC)
                    quality = 'POOR'
                elif advance_angle < 5:
                    timing_color = '#F39C12'  # Orange - poor
                    quality = 'POOR'
                elif advance_angle < 10:
                    timing_color = '#F1C40F'  # Yellow - fair
                    quality = 'FAIR'
                else:
                    timing_color = '#27AE60'  # Green - good
                    quality = 'GOOD'
                
                # Draw timing lines
                for t_time in [trigger_time, spark_time]:
                    x = time_to_x(t_time)
                    line = doc.createElement('line')
                    line.setAttribute('x1', str(x))
                    line.setAttribute('y1', str(signal_to_y(0, True) + 15))
                    line.setAttribute('x2', str(x))
                    line.setAttribute('y2', str(signal_to_y(0, False) - 15))
                    line.setAttribute('class', 'timing-arrow')
                    line.setAttribute('stroke', timing_color)
                    svg.appendChild(line)
                
                # Add timing delay label
                mid_x = time_to_x((trigger_time + spark_time) / 2)
                delay_text = doc.createElement('text')
                delay_text.setAttribute('x', str(mid_x))
                delay_text.setAttribute('y', str(separator_y - 20))
                delay_text.setAttribute('text-anchor', 'middle')
                delay_text.setAttribute('class', 'timing-label')
                delay_text.setAttribute('fill', timing_color)
                delay_text.appendChild(doc.createTextNode(f'{delay_ms:.1f} ms'))
                svg.appendChild(delay_text)
                
                # Add advance angle label
                advance_text = doc.createElement('text')
                advance_text.setAttribute('x', str(mid_x))
                advance_text.setAttribute('y', str(separator_y - 5))
                advance_text.setAttribute('text-anchor', 'middle')
                advance_text.setAttribute('class', 'timing-label')
                advance_text.setAttribute('fill', timing_color)
                advance_text.appendChild(doc.createTextNode(f'{advance_angle:+.1f}° BTDC ({quality})'))
                svg.appendChild(advance_text)
    
    def _add_plot_labels(self, doc, svg, title, width, height, margin_left, margin_top, 
                        plot_height, plot_width, time_range_ms):
        """Add labels and title to SVG"""
        # Signal labels
        trigger_label = doc.createElement('text')
        trigger_label.setAttribute('x', '15')
        trigger_label.setAttribute('y', str(margin_top + plot_height * 0.25))
        trigger_label.setAttribute('class', 'label')
        trigger_label.setAttribute('fill', '#2E86C1')
        trigger_label.appendChild(doc.createTextNode('TRIGGER'))
        svg.appendChild(trigger_label)
        
        spark_label = doc.createElement('text')
        spark_label.setAttribute('x', '15')
        spark_label.setAttribute('y', str(margin_top + plot_height * 0.75))
        spark_label.setAttribute('class', 'label')
        spark_label.setAttribute('fill', '#E74C3C')
        spark_label.appendChild(doc.createTextNode('SPARK'))
        svg.appendChild(spark_label)
        
        # Title
        title_elem = doc.createElement('text')
        title_elem.setAttribute('x', str(width / 2))
        title_elem.setAttribute('y', '25')
        title_elem.setAttribute('text-anchor', 'middle')
        title_elem.setAttribute('class', 'title')
        title_elem.appendChild(doc.createTextNode(title))
        svg.appendChild(title_elem)
        
        # X-axis labels
        for i in range(5):
            x = margin_left + i * (plot_width / 4)
            t = i * (time_range_ms / 4)
            
            text = doc.createElement('text')
            text.setAttribute('x', str(x))
            text.setAttribute('y', str(height - 40))
            text.setAttribute('text-anchor', 'middle')
            text.setAttribute('class', 'axis-label')
            text.appendChild(doc.createTextNode(f'{t:.1f}'))
            svg.appendChild(text)
        
        # X-axis title
        x_label = doc.createElement('text')
        x_label.setAttribute('x', str(width / 2))
        x_label.setAttribute('y', str(height - 10))
        x_label.setAttribute('text-anchor', 'middle')
        x_label.setAttribute('class', 'axis-label')
        x_label.appendChild(doc.createTextNode('Time (milliseconds)'))
        svg.appendChild(x_label)

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 timing_analyzer.py <vcd_file>")
        print("Example: python3 timing_analyzer.py wokwi-logic.vcd")
        sys.exit(1)
    
    vcd_file = sys.argv[1]
    analyzer = TimingAnalyzer(vcd_file)
    
    print("=== VCD TIMING ANALYSIS ===")
    
    # Step 1: Analyze VCD data
    analyzer.analyze_vcd()
    
    # Step 2: Generate analysis report
    analysis_file = vcd_file.replace('.vcd', '-analysis.txt')
    print(f"\nGenerating analysis report: {analysis_file}")
    analyzer.generate_analysis_report(analysis_file)
    
    # Step 3: Create focused timing plots
    print("\nCreating focused timing plots...")
    created_plots = analyzer.create_focused_plots()
    
    # Summary
    print(f"\n=== ANALYSIS COMPLETE ===")
    print(f"Analysis report: {analysis_file}")
    print(f"Created {len(created_plots)} timing plots:")
    for plot in created_plots:
        print(f"  - {plot}")
    
    print(f"\nTo include plots in README.md:")
    for plot in created_plots:
        print(f"![Timing Plot]({plot})")

if __name__ == "__main__":
    main()