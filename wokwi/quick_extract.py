#!/usr/bin/env python3
"""
Quick VCD data extraction for common analysis scenarios
"""

import subprocess
import sys

def extract_and_plot(scenario_name, start_time, end_time, description):
    """Extract VCD data and create plot for a specific scenario"""
    
    print(f"\n=== {scenario_name} ===")
    print(f"Description: {description}")
    print(f"Time range: {start_time}s - {end_time}s")
    
    # Extract data
    cmd1 = f"python3 extract_vcd_data.py wokwi-logic.vcd {start_time} {end_time}"
    subprocess.run(cmd1, shell=True)
    
    # Create SVG
    output_svg = f"{scenario_name.lower().replace(' ', '_')}_timing.svg"
    cmd2 = f"python3 create_svg_plot.py wokwi-logic_signals_plot.csv"
    subprocess.run(cmd2, shell=True)
    
    # Rename SVG
    subprocess.run(f"mv ignition_timing_detail.svg {output_svg}", shell=True)
    
    print(f"Created: {output_svg}")
    return output_svg

def main():
    """Generate timing plots for key analysis scenarios"""
    
    scenarios = [
        ("Low_RPM_Validation", 3, 15, "Timer1 overflow fix validation - startup through 1000 RPM"),
        ("Mid_RPM_Sweep", 20, 35, "Active timing curve region - 1500-3500 RPM"),
        ("High_RPM_Operation", 70, 80, "Previous-lobe scheduling and rev limiter - 6500+ RPM"),
        ("Transition_Analysis", 15, 25, "RPM acceleration phase - rapid timing changes")
    ]
    
    created_files = []
    
    for name, start, end, desc in scenarios:
        svg_file = extract_and_plot(name, start, end, desc)
        created_files.append(svg_file)
    
    print(f"\n=== Summary ===")
    print(f"Created {len(created_files)} timing visualization files:")
    for f in created_files:
        print(f"  - {f}")
    
    print(f"\nTo include in documentation:")
    for f in created_files:
        print(f"  ![{f.replace('_', ' ').replace('.svg', '').title()}]({f})")

if __name__ == "__main__":
    main()