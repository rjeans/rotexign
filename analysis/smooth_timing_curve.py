#!/usr/bin/env python3
"""
Smooth and interpolate the ignition timing curve.
Takes the original safe curve points and produces a smoothed version with more points.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d, UnivariateSpline
from scipy.signal import savgol_filter

# Original safe curve data points
# RPM:     [0,    1000,  2000,  3000,  4000,  5000,  6000,  7000]
# Advance: [0,    6,     12,    15,    15,    14,    13,    12]
original_rpm = np.array([0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000])
original_advance = np.array([0, 6, 12, 15, 15, 14, 13, 12, 11])

def smooth_curve_spline(rpm_points, advance_points, num_points=100, smoothing_factor=0.5):
    """
    Smooth the curve using a cubic spline with adjustable smoothing.
    
    Args:
        rpm_points: Original RPM values
        advance_points: Original advance values
        num_points: Number of interpolation points to generate
        smoothing_factor: 0=interpolate exactly through points, higher=more smoothing
    
    Returns:
        Tuple of (interpolated_rpm, smoothed_advance)
    """
    # Create a spline with smoothing
    # s parameter controls smoothing (0 = interpolate exactly)
    spline = UnivariateSpline(rpm_points, advance_points, s=smoothing_factor, k=3)
    
    # Generate more points
    rpm_smooth = np.linspace(rpm_points.min(), rpm_points.max(), num_points)
    advance_smooth = spline(rpm_smooth)
    
    # Ensure advance stays within reasonable bounds
    advance_smooth = np.clip(advance_smooth, 0, 20)
    
    return rpm_smooth, advance_smooth

def smooth_curve_cubic(rpm_points, advance_points, num_points=100):
    """
    Smooth the curve using cubic interpolation.
    
    Args:
        rpm_points: Original RPM values
        advance_points: Original advance values
        num_points: Number of interpolation points to generate
    
    Returns:
        Tuple of (interpolated_rpm, smoothed_advance)
    """
    # Create cubic interpolation
    f_cubic = interp1d(rpm_points, advance_points, kind='cubic')
    
    # Generate more points
    rpm_smooth = np.linspace(rpm_points.min(), rpm_points.max(), num_points)
    advance_smooth = f_cubic(rpm_smooth)
    
    # Apply additional smoothing with Savitzky-Golay filter
    if len(advance_smooth) > 11:
        advance_smooth = savgol_filter(advance_smooth, window_length=11, polyorder=3)
    
    # Ensure advance stays within reasonable bounds
    advance_smooth = np.clip(advance_smooth, 0, 20)
    
    return rpm_smooth, advance_smooth

def generate_lookup_table(rpm_smooth, advance_smooth, step_size=100):
    """
    Generate a lookup table at regular RPM intervals.
    
    Args:
        rpm_smooth: Smoothed RPM values
        advance_smooth: Smoothed advance values
        step_size: RPM step size for lookup table
    
    Returns:
        Tuple of (rpm_table, advance_table)
    """
    # Generate RPM points at regular intervals
    rpm_table = np.arange(0, rpm_smooth.max() + step_size, step_size)
    
    # Interpolate advance values at these points
    f_interp = interp1d(rpm_smooth, advance_smooth, kind='linear', fill_value='extrapolate')
    advance_table = f_interp(rpm_table)
    
    # Ensure bounds
    advance_table = np.clip(advance_table, 0, 20)
    
    return rpm_table, advance_table

def plot_curve(original_rpm, original_advance, rpm_smooth, advance_smooth):
    """
    Plot original points and smoothed curve.
    
    Args:
        original_rpm: Original RPM points
        original_advance: Original advance points
        rpm_smooth: Smoothed RPM values
        advance_smooth: Smoothed advance values
    """
    plt.figure(figsize=(12, 8))
    
    # Plot original points
    plt.scatter(original_rpm, original_advance, s=100, c='red', 
                label='Original Points', zorder=5)
    
    # Plot original linear interpolation
    rpm_linear = np.linspace(0, 8000, 100)
    f_linear = interp1d(original_rpm, original_advance, kind='linear')
    plt.plot(rpm_linear, f_linear(rpm_linear), 'r--', alpha=0.5, 
             label='Original Linear', linewidth=1)
    
    # Plot smoothed curve
    plt.plot(rpm_smooth, advance_smooth, 'blue', 
            label='Cubic + SavGol Smoothed', linewidth=2.5)
    
    plt.xlabel('RPM')
    plt.ylabel('Ignition Advance (degrees BTDC)')
    plt.title('Original vs Smoothed Timing Curves')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.xlim(0, 8000)
    plt.ylim(-1, 20)
    
    plt.tight_layout()
    plt.savefig('smoothed_timing_curves.png', dpi=300, bbox_inches='tight')
    plt.close()
    print("Plot saved to smoothed_timing_curves.png")

def save_curves_to_csv(filename, rpm, advance, description=""):
    """
    Save curve data to CSV file.
    
    Args:
        filename: Output CSV filename
        rpm: RPM values
        advance: Advance values
        description: Optional description
    """
    import csv
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Write header with description
        if description:
            writer.writerow([f"# {description}"])
        writer.writerow(['rpm', 'advance_degrees'])
        
        # Write data
        for r, a in zip(rpm, advance):
            writer.writerow([f"{r:.0f}", f"{a:.2f}"])
    
    print(f"Saved {len(rpm)} points to {filename}")

def main():
    """Generate smoothed timing curve using Cubic + SavGol method."""
    
    print("Original timing curve:")
    print(f"RPM:     {original_rpm.tolist()}")
    print(f"Advance: {original_advance.tolist()}")
    print()
    
    # Generate smoothed curve using Cubic interpolation with Savitzky-Golay smoothing
    rpm_cubic, advance_cubic = smooth_curve_cubic(original_rpm, original_advance, num_points=200)
    print(f"Cubic + SavGol interpolation: Generated {len(rpm_cubic)} points")
    
    # Generate lookup table at 100 RPM intervals
    print("\nGenerating lookup table at 100 RPM intervals:")
    rpm_table, advance_table = generate_lookup_table(rpm_cubic, advance_cubic, step_size=100)
    print(f"Lookup table: {len(rpm_table)} points from {rpm_table[0]:.0f} to {rpm_table[-1]:.0f} RPM")
    
    # Save curves to CSV files
    save_curves_to_csv('timing_curve_cubic.csv', rpm_cubic, advance_cubic, 
                      "Cubic interpolation with Savitzky-Golay smoothing")
    save_curves_to_csv('timing_lookup_table.csv', rpm_table, advance_table,
                      "Lookup table at 100 RPM intervals")
    
    # Print lookup table for Arduino in the required format
    print("\n// Timing curve: RPM -> advance_degrees_x10 (Cubic + SavGol smoothed)")
    print("// Advance values are stored as tenths of degrees (multiply by 10)")
    print("static const uint16_t timing_rpm_curve[][2] PROGMEM = {")
    
    for i, (rpm, adv) in enumerate(zip(rpm_table, advance_table)):
        # Format: {RPM, advance_degrees_x10}
        rpm_int = int(rpm)
        adv_tenths = int(round(adv * 10))  # Store as tenths of degrees
        
        # Add appropriate comment showing actual degrees
        comment = f"  // {rpm_int} RPM -> {adv:.1f}°"
        
        # Add comma except for last entry
        if i < len(rpm_table) - 1:
            print(f"    {{{rpm_int}, {adv_tenths}}},{comment}")
        else:
            print(f"    {{{rpm_int}, {adv_tenths}}} {comment}")
    
    print("};")
    print(f"\nconst uint8_t TIMING_CURVE_SIZE = {len(rpm_table)};")
    
    # Also print a compact version with key points only
    print("\n// Compact version with key RPM points only:")
    print("// Advance values are stored as tenths of degrees (multiply by 10)")
    print("static const uint16_t timing_rpm_curve_compact[][2] PROGMEM = {")
    
    # Select key RPM points
    key_rpms = [0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000]
    
    for i, target_rpm in enumerate(key_rpms):
        # Find the advance value for this RPM
        if target_rpm in rpm_table:
            idx = list(rpm_table).index(target_rpm)
            adv = advance_table[idx]
        else:
            # Interpolate if needed
            adv = np.interp(target_rpm, rpm_table, advance_table)
        
        rpm_int = int(target_rpm)
        adv_tenths = int(round(adv * 10))  # Store as tenths
        
        comment = f"  // {rpm_int} RPM -> {adv:.1f}°"
        
        if i < len(key_rpms) - 1:
            print(f"    {{{rpm_int}, {adv_tenths}}},{comment}")
        else:
            print(f"    {{{rpm_int}, {adv_tenths}}} {comment}")
    
    print("};")
    print(f"\nconst uint8_t TIMING_CURVE_COMPACT_SIZE = {len(key_rpms)};")
    
    # Plot curve
    plot_curve(original_rpm, original_advance, rpm_cubic, advance_cubic)
    
    # Print some key points for verification
    print("\nKey advance values at specific RPMs:")
    test_rpms = [800, 1500, 2500, 3500, 4500, 5500, 6500, 7500]
    f_cubic = interp1d(rpm_cubic, advance_cubic)
    
    for test_rpm in test_rpms:
        if test_rpm <= rpm_cubic.max():
            advance = f_cubic(test_rpm)
            print(f"  {test_rpm:4d} RPM: {advance:.1f}° BTDC")

if __name__ == "__main__":
    main()