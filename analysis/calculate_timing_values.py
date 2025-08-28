#!/usr/bin/env python3
"""
Calculate ignition timing values across RPM range.
Outputs CSV with advance angles and timing delays for safe and performance curves.
"""

import csv

# Timing curve from firmware (rotexign.ino)
RPM_POINTS = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000]
ADVANCE_CURVE = [0, 6, 12, 15, 15, 14, 13, 12]

# Engine constants
PULSES_PER_REV = 2  # Two flywheel lobes (180° apart)
TRIGGER_BTDC_DEG = 47  # Trigger occurs 47° before TDC
NOMINAL_DWELL_MS = 3.0  # Nominal dwell time
NOMINAL_DWELL_US = NOMINAL_DWELL_MS * 1000
MAX_DUTY_CYCLE = 0.40  # Maximum 40% duty cycle

# Arduino Timer1 constants
TIMER_PRESCALER = 64  # Timer1 prescaler
ARDUINO_CLOCK = 16000000  # 16 MHz Arduino clock
US_PER_TICK = TIMER_PRESCALER / (ARDUINO_CLOCK / 1000000)  # 4μs per tick

def interpolate_advance(rpm, rpm_points, advance_curve):
    """Linear interpolation of advance angle from curve."""
    # Handle edge cases
    if rpm <= rpm_points[0]:
        return float(advance_curve[0])
    if rpm >= rpm_points[-1]:
        return float(advance_curve[-1])
    
    # Find interpolation points
    for i in range(1, len(rpm_points)):
        if rpm <= rpm_points[i]:
            rpm_low = rpm_points[i-1]
            rpm_high = rpm_points[i]
            adv_low = advance_curve[i-1]
            adv_high = advance_curve[i]
            
            # Linear interpolation
            ratio = (rpm - rpm_low) / (rpm_high - rpm_low)
            return adv_low + (adv_high - adv_low) * ratio
    
    # Should never reach here
    return float(advance_curve[-1])

def calculate_timing(rpm):
    """Calculate timing values for a given RPM."""
    if rpm == 0:
        return None  # Engine not running
    
    # Get advance angle from timing curve
    advance_deg = interpolate_advance(rpm, RPM_POINTS, ADVANCE_CURVE)
    
    # Calculate period between trigger pulses
    # Period (ms) = 60,000 / (RPM * pulses_per_rev)
    period_ms = 60000.0 / (rpm * PULSES_PER_REV)
    period_us = period_ms * 1000
    
    # Calculate degrees per microsecond
    # 360 degrees per revolution, PULSES_PER_REV pulses per revolution
    # So each pulse represents 180 degrees (360/2)
    degrees_per_pulse = 360.0 / PULSES_PER_REV  # 180 degrees
    degrees_per_us = degrees_per_pulse / period_us
    
    # Calculate actual dwell time (limited by duty cycle)
    # Dwell is either nominal 3ms or limited to 40% duty cycle
    max_dwell_us = period_us * MAX_DUTY_CYCLE
    actual_dwell_us = min(NOMINAL_DWELL_US, max_dwell_us)
    actual_dwell_ticks = int(round(actual_dwell_us / US_PER_TICK))
    
    # Calculate spark delay from trigger (microseconds)
    # Trigger occurs at 47° BTDC
    # Spark should occur at advance_deg BTDC
    # So spark delay in degrees = 47 - advance_deg
    # If advance > 47, spark would need to fire before trigger (negative delay)
    spark_delay_deg = TRIGGER_BTDC_DEG - advance_deg
    spark_delay_us = spark_delay_deg / degrees_per_us
    
    # Calculate dwell start delay using actual dwell time
    dwell_delay_us = spark_delay_us - actual_dwell_us
    
    # If dwell delay is negative, add a period (schedule from previous trigger)
    if dwell_delay_us < 0:
        dwell_delay_us += period_us
    
    # Convert to timer ticks (4μs per tick with prescaler 64)
    spark_delay_ticks = int(round(spark_delay_us / US_PER_TICK))
    dwell_delay_ticks = int(round(dwell_delay_us / US_PER_TICK))
    period_ticks = int(round(period_us / US_PER_TICK))
    
    # Calculate actual duty cycle (dwell time as percentage of period)
    # Duty cycle = (dwell_time / period) * 100
    duty_cycle_percent = (actual_dwell_us / period_us) * 100
    
    return {
        'rpm': rpm,
        'period_us': period_us,
        'period_ticks': period_ticks,
        'advance_deg': advance_deg,
        'dwell_us': actual_dwell_us,
        'dwell_ticks': actual_dwell_ticks,
        'spark_delay_us': spark_delay_us,
        'spark_delay_ticks': spark_delay_ticks,
        'dwell_delay_us': dwell_delay_us,
        'dwell_delay_ticks': dwell_delay_ticks,
        'duty_cycle_percent': duty_cycle_percent
    }

def main():
    """Generate CSV file with timing calculations."""
    # Open CSV file for writing
    with open('timing_calculations.csv', 'w', newline='') as csvfile:
        fieldnames = [
            'rpm',
            'period_us',
            'period_ticks',
            'advance_deg',
            'dwell_us',
            'dwell_ticks',
            'spark_delay_us',
            'spark_delay_ticks',
            'dwell_delay_us',
            'dwell_delay_ticks',
            'duty_cycle_percent'
        ]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        # Write header
        writer.writeheader()
        
        # Calculate and write values for RPM range
        for rpm in range(200, 8001, 100):  # 200 to 8000, step of 100
            timing = calculate_timing(rpm)
            if timing:
                # Round values for readability (ticks are already integers)
                timing['period_us'] = round(timing['period_us'], 1)
                timing['period_ticks'] = timing['period_ticks']  # Already integer
                timing['advance_deg'] = round(timing['advance_deg'], 2)
                timing['dwell_us'] = round(timing['dwell_us'], 1)
                timing['dwell_ticks'] = timing['dwell_ticks']  # Already integer
                timing['spark_delay_us'] = round(timing['spark_delay_us'], 1)
                timing['spark_delay_ticks'] = timing['spark_delay_ticks']  # Already integer
                timing['dwell_delay_us'] = round(timing['dwell_delay_us'], 1)
                timing['dwell_delay_ticks'] = timing['dwell_delay_ticks']  # Already integer
                timing['duty_cycle_percent'] = round(timing['duty_cycle_percent'], 2)
                
                writer.writerow(timing)
    
    print("Timing calculations saved to timing_calculations.csv")
    
    # Show spark delay ticks at interpolation points
    print("\n/64 prescaler timing at interpolation points:")
    print("RPM\tAdvance°\tPeriod (ticks)\tSpark Delay (ticks)")
    print("-" * 60)
    
    # Use the actual interpolation points from the timing curve
    for i, rpm in enumerate(RPM_POINTS):
        if rpm == 0:
            continue  # Skip 0 RPM
        timing = calculate_timing(rpm)
        if timing:
            print(f"{rpm}\t{ADVANCE_CURVE[i]:.1f}\t\t{timing['period_ticks']}\t\t{timing['spark_delay_ticks']}")

if __name__ == "__main__":
    main()