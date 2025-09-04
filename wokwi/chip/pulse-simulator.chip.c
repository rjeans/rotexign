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
static bool ignition_on = false;     // Current ignition state (latched)
static bool last_ignition_pin = false;  // Previous ignition pin state for edge detection

// Auto-sweep mode variables
static bool auto_sweep_active = false;  // Auto-sweep mode active
static float auto_sweep_time = 0;       // Time at current RPM step
static const float AUTO_SWEEP_DURATION = 2.0f;  // 0.5 seconds at each RPM
static const float AUTO_SWEEP_STEP = 100.0f;    // 100 RPM increment

 // RPM range: idle (800 RPM) to max (8000 RPM)
static  const float IDLE_RPM = 1500.0f;
static const float MAX_RPM = 8000.0f;


static float rpm = 0;
static int noise_pulse=1000;
static timer_t noise_timer_id;
    


typedef struct {
  pin_t pin_out_pulse;
  pin_t pin_ready;     // READY pin for sweep activation
  pin_t pin_spark;     
} chip_state_t;





static void chip_timer_event(void *user_data);
static void noise_timer_event(void *user_data);

static void spark(void * user_data, pin_t pin, uint32_t value);
static void ready_pin_change(void * user_data, pin_t pin, uint32_t value);

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));

  chip->pin_out_pulse = pin_init("PULSE", OUTPUT_HIGH);  // Start HIGH (+VCC, normal state)

  chip->pin_ready = pin_init("READY", INPUT);             // READY pin for sweep activation
  chip->pin_spark = pin_init("SPARK", INPUT);        

  const timer_config_t timer_config = {
    .callback = chip_timer_event,
    .user_data = chip,
  };

  const timer_config_t noise_timer_config = {
    .callback = noise_timer_event,
    .user_data = chip,
  };

  const pin_watch_config_t spark_config = {
   .edge = RISING,
   .pin_change = spark,
   .user_data = chip
};

  const pin_watch_config_t ready_config = {
    .edge = RISING,
    .pin_change = ready_pin_change,
    .user_data = chip
  };

  
  pin_watch(chip->pin_spark,&spark_config);
  pin_watch(chip->pin_ready, &ready_config);

  timer_t timer_id = timer_init(&timer_config);
  timer_start(timer_id, interval, true);

  noise_timer_id = timer_init(&noise_timer_config);
  
  printf("[PULSE_GEN] Initialized - Ignition simulator mode\n");
  
}

void spark(void * user_data, pin_t pin, uint32_t value) {

  chip_state_t *chip = (chip_state_t*)user_data;

   if((rand() % 10 )==0) {
    pin_write(chip->pin_out_pulse, LOW);  // LOW = GND (trigger pulse)
 
    timer_start(noise_timer_id,noise_pulse,false);
   }

}

void noise_timer_event(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;

 
  pin_write(chip->pin_out_pulse, HIGH);  // LOW = GND (trigger pulse)
 
}
 

void chip_timer_event(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
  
  // Update total running time
  total_time_running += (float)interval / 1000000.0f;
  
  // Handle auto-sweep mode
  if (auto_sweep_active && ignition_on) {
    auto_sweep_time += (float)interval / 1000000.0f;
    
    // Check if it's time to increment RPM
    if (auto_sweep_time >= AUTO_SWEEP_DURATION) {
      auto_sweep_time = 0;  // Reset timer
      rpm += AUTO_SWEEP_STEP;
      
      if (rpm > MAX_RPM) {
        // Reached max RPM, stop auto-sweep
        rpm = MAX_RPM;
        auto_sweep_active = false;
        printf("[PULSE_GEN] Auto-sweep completed at MAX RPM %.0f\n", rpm);
      } else {
        printf("[PULSE_GEN] Auto-sweep: RPM increased to %.0f\n", rpm);
      }
    }
  }
  
  // Always output HIGH when ignition is off
  if (!ignition_on) {
    pin_write(chip->pin_out_pulse, HIGH);
    state = 1;
    time_running = 0;
    return;
  }
  
  // Ignition is ON - generate trigger pulses
  // Rotax 787: 2 pulses per revolution, so period = 30/RPM seconds per pulse
  float total_period = 30.0f / rpm;  // seconds per pulse (half revolution)
  float pulse_width = 0.001f;   // Fixed 1.0ms pulse width (measured from real system)
  
  time_running += (float)interval / 1000000.0f;

  // State machine: trigger every total_period, fixed 1.0ms pulse width
  if (time_running >= total_period && state == 1) {
    // Start of pulse: go to GND (LOW) - this is the trigger edge
    state = 0;
    pin_write(chip->pin_out_pulse, LOW);  // LOW = GND (trigger pulse)
    time_running = 0;  // Reset timing for next period
  } else if (time_running > pulse_width && state == 0) {
    // End of pulse: return to +VCC (HIGH)  
    state = 1;
    pin_write(chip->pin_out_pulse, HIGH);  // HIGH = +VCC
    pulse_count++;
  }
  
  // Diagnostics
  if (total_time_running - last_diagnostic_time >= 2.0f) {
    const char* ign_str = ignition_on ? "ON" : "OFF";
    
    printf("[PULSE_GEN] Time: %.1fs, IGN: %s, RPM: %.0f, Period: %.3fms (%.0fus), Pulse: %.3fms\n", 
           total_time_running, ign_str, rpm, total_period*1000.0f, total_period*1000000.0f, pulse_width*1000.0f);
    last_diagnostic_time = total_time_running;
  }
}

static void ready_pin_change(void * user_data, pin_t pin, uint32_t value) {
  // On HIGH pulse, activate auto-sweep regardless of ignition state
  ignition_on = true;
  rpm = IDLE_RPM;
  time_running = 0;
  state = 1;
  auto_sweep_active = true;
  auto_sweep_time = 0;
  printf("[PULSE_GEN] READY pin: Auto-sweep started - Ignition ON, RPM starting at %.0f\n", rpm);
}