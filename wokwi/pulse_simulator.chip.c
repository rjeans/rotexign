// MIT License
// Copyright (c) 2023 MicroBeaut
// Modified for rotexign project - RPM sweep functionality added

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static float time_running = 0;
static float total_time_running = 0;  // Total time since start
static int interval = 100;  // 100 microseconds = 10kHz update rate
static int state = 1;
static uint32_t pulse_count = 0;     // Total pulses generated
static float last_diagnostic_time = 0;  // For periodic diagnostics

typedef struct {
  pin_t pin_out_pulse;
  uint32_t rpm_attr;
  uint32_t sweep_enable_attr;
  uint32_t sweep_duration_attr;
  uint32_t sweep_min_rpm_attr;
  uint32_t sweep_max_rpm_attr;
  uint32_t startup_delay_attr;
} chip_state_t;

static void chip_timer_event(void *user_data);
static void print_simulator_status(chip_state_t *chip, float rpm, bool sweep_enable, float uptime);

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));

  chip->pin_out_pulse = pin_init("PULSE", OUTPUT_HIGH);
  chip->rpm_attr = attr_init_float("rpm", 1000.0f);
  chip->sweep_enable_attr = attr_init_float("sweep_enable", 0.0f);  // 0=false, 1=true
  chip->sweep_duration_attr = attr_init_float("sweep_duration", 30.0f);  // 30 seconds default
  chip->sweep_min_rpm_attr = attr_init_float("sweep_min_rpm", 200.0f);
  chip->sweep_max_rpm_attr = attr_init_float("sweep_max_rpm", 8000.0f);
  chip->startup_delay_attr = attr_init_float("startup_delay", 3.0f);  // 3 second startup delay

  // Print startup diagnostics to verify parameter reading
  printf("[SIMULATOR] === INITIALIZATION DIAGNOSTICS ===\n");
  printf("[SIMULATOR] Fixed RPM: %.3f (raw)\n", attr_read_float(chip->rpm_attr));
  printf("[SIMULATOR] Sweep Enable: %.3f (raw, 0=OFF, 1=ON)\n", attr_read_float(chip->sweep_enable_attr));
  printf("[SIMULATOR] Sweep Duration: %.3fs (raw)\n", attr_read_float(chip->sweep_duration_attr));
  printf("[SIMULATOR] Min RPM: %.3f (raw)\n", attr_read_float(chip->sweep_min_rpm_attr));
  printf("[SIMULATOR] Max RPM: %.3f (raw)\n", attr_read_float(chip->sweep_max_rpm_attr));
  printf("[SIMULATOR] Startup Delay: %.3fs (raw)\n", attr_read_float(chip->startup_delay_attr));
  printf("[SIMULATOR] Timer Interval: %d microseconds\n", interval);
  
  float expected_sweep_rate = (attr_read_float(chip->sweep_max_rpm_attr) - attr_read_float(chip->sweep_min_rpm_attr)) / attr_read_float(chip->sweep_duration_attr);
  printf("[SIMULATOR] Expected Sweep Rate: %.1f RPM/second\n", expected_sweep_rate);
  printf("[SIMULATOR] =====================================\n");

  const timer_config_t timer_config = {
    .callback = chip_timer_event,
    .user_data = chip,
  };
  timer_t timer_id = timer_init(&timer_config);
  timer_start(timer_id, interval, true);
}

void chip_timer_event(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
  
  // Update total running time
  total_time_running += (float)interval / 1000000.0f;
  
  float rpm;
  float sweep_enable_val = attr_read_float(chip->sweep_enable_attr);
  bool sweep_enable = (sweep_enable_val > 0.5f);  // Treat > 0.5 as true
  
  if (sweep_enable) {
    // RPM sweep mode with startup delay
    float startup_delay = attr_read_float(chip->startup_delay_attr);
    float sweep_duration = attr_read_float(chip->sweep_duration_attr);
    float min_rpm = attr_read_float(chip->sweep_min_rpm_attr);
    float max_rpm = attr_read_float(chip->sweep_max_rpm_attr);
    
    if (total_time_running < startup_delay) {
      // During startup delay, run at minimum RPM to allow relay initialization
      rpm = min_rpm;
    } else {
      // Calculate sweep progress based on time elapsed since startup delay ended
      float time_since_sweep_start = total_time_running - startup_delay;
      
      // Linear RPM sweep with stop at maximum
      float sweep_progress = time_since_sweep_start / sweep_duration;
      if (sweep_progress >= 1.0f) {
        // Sweep completed - hold at maximum RPM
        rpm = max_rpm;
      } else {
        // Sweep in progress - smooth linear interpolation
        rpm = min_rpm + (max_rpm - min_rpm) * sweep_progress;
      }
      
      // Bounds checking to prevent invalid RPM values
      if (rpm < 0) rpm = 0;
      if (rpm > max_rpm) rpm = max_rpm;
    }
    
  } else {
    // Fixed RPM mode
    rpm = attr_read_float(chip->rpm_attr);
  }

  if (rpm == 0) {
    pin_write(chip->pin_out_pulse, HIGH);
    state = 1;
    return;
  }

  // Calculate period for two pulses per revolution (Rotax 787 has 2 flywheel lobes)
  float period = 30.0f / rpm;  // Time for half revolution (one pulse)
  
  // Sanity check: ensure period is reasonable (1ms to 10s range)
  if (period < 0.001f) period = 0.001f;  // Min period = 1ms (30000 RPM - well above our max)
  if (period > 10.0f) period = 10.0f;    // Max period = 10s (3 RPM)
  
  float width = period / 10;   // Pulse width = 10% of period
  if (width < 0.001f) width = 0.001f;  // Minimum 1ms pulse width
  
  time_running += (float)interval / 1000000.0f;

  // Rising edge - end of pulse
  if (time_running > period && state == 0) {
    time_running = 0;
    state = 1;
    pulse_count++;
    pin_write(chip->pin_out_pulse, HIGH);
    
    // Print diagnostics every 5 seconds to match Arduino
    if (total_time_running - last_diagnostic_time >= 5.0f) {
      last_diagnostic_time = total_time_running;
      print_simulator_status(chip, rpm, sweep_enable, total_time_running);
    }
    return;
  }

  // Falling edge - start of pulse
  if (time_running > period - width && state == 1) {
    state = 0;
    pin_write(chip->pin_out_pulse, LOW);
    return;
  }

  return;
}

// Print simulator diagnostics to match Arduino serial output format
static void print_simulator_status(chip_state_t *chip, float rpm, bool sweep_enable, float uptime) {
  // Calculate period in microseconds (time between triggers)
  float period_us = (rpm > 0) ? (30000000.0f / rpm) : 0;  // 30s/rpm * 1000000 us/s
  
  // Calculate simulated advance angle (assuming target of ~10-15° at current RPM)
  float target_advance = 0;
  if (rpm >= 1000 && rpm < 2000) {
    target_advance = 6.0f + (rpm - 1000.0f) * 6.0f / 1000.0f;  // 6-12°
  } else if (rpm >= 2000 && rpm < 3000) {
    target_advance = 12.0f + (rpm - 2000.0f) * 3.0f / 1000.0f; // 12-15°
  } else if (rpm >= 3000) {
    target_advance = 15.0f;  // Max advance
  } else {
    target_advance = 3.0f;   // Cranking advance
  }
  
  // Calculate delta angle (trigger to spark timing)
  float delta_angle = 47.0f - target_advance;
  
  // Print status in Arduino format only
  printf("[SIMULATOR] RPM: %.0f, Period: %.0fus, Advance: %.1f°, Delta: %.1f°, Dwell: 3000us, ", 
         rpm, period_us, target_advance, delta_angle);
  printf("Engine: %s, RevLim: %s, Errors: 0x0\n", 
         (rpm > 200) ? "RUN" : "STOP", 
         (rpm > 7000) ? "ON" : "OFF");
}