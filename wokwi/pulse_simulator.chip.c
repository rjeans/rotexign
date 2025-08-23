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

 // RPM range: idle (800 RPM) to max (8000 RPM)
static  const float IDLE_RPM = 800.0f;
static const float MAX_RPM = 8000.0f;
static const float RPM_CHANGE = 200.0f;

static float rpm = 0;
    


typedef struct {
  pin_t pin_out_pulse;
  pin_t pin_ignition_switch;
  pin_t pin_throttle_up;  
  pin_t pin_throttle_down;
} chip_state_t;





static void chip_timer_event(void *user_data);
static void ignition_change(void * user_data, pin_t pin, uint32_t value);
static void throttle_up(void * user_data, pin_t pin, uint32_t value);
static void throttle_down(void * user_data, pin_t pin, uint32_t value);

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));

  chip->pin_out_pulse = pin_init("PULSE", OUTPUT_HIGH);  // Start HIGH (+VCC, normal state)
  chip->pin_ignition_switch = pin_init("IGN", INPUT);    // Ignition switch input
  chip->pin_throttle_up = pin_init("UP", INPUT);     
  chip->pin_throttle_down = pin_init("DOWN", INPUT);     

  const timer_config_t timer_config = {
    .callback = chip_timer_event,
    .user_data = chip,
  };

  const pin_watch_config_t ign_config = {
   .edge = RISING,
   .pin_change = ignition_change,
   .user_data = chip
};

  const pin_watch_config_t throttle_up_config = {
   .edge = RISING,
   .pin_change = throttle_up,
   .user_data = chip
};

  const pin_watch_config_t throttle_down_config = {
   .edge = RISING,
   .pin_change = throttle_down,
   .user_data = chip
};

  pin_watch(chip->pin_ignition_switch,&ign_config);
 pin_watch(chip->pin_throttle_up,&throttle_up_config);
 pin_watch(chip->pin_throttle_down,&throttle_down_config);

  timer_t timer_id = timer_init(&timer_config);
  timer_start(timer_id, interval, true);
  
  printf("[PULSE_GEN] Initialized - Ignition simulator mode\n");
  printf("[PULSE_GEN] IGN pin: Send pulse to toggle ignition on/off\n");
  printf("[PULSE_GEN] THR pin: 0V=idle RPM, Vcc=max RPM\n");
}

void ignition_change(void * user_data, pin_t pin, uint32_t value) {
  ignition_on = !ignition_on;
  if (ignition_on) {
    rpm = IDLE_RPM;
    // Reset timing when ignition turns on to prevent timing artifacts
    time_running = 0;
    state = 1;  // Start in HIGH state
    printf("[PULSE_GEN] Ignition ON - RPM set to %.0f\n", rpm);
  } else {
    rpm = 0;
    time_running = 0;
    state = 1;  // Return to HIGH state
    printf("[PULSE_GEN] Ignition OFF\n");
  }
}
void throttle_up(void * user_data, pin_t pin, uint32_t value) {
  if (ignition_on) {
  printf("[PULSE_GEN] Throttle Up\n");
  rpm+=RPM_CHANGE;
  if (rpm >= MAX_RPM) rpm=MAX_RPM;
  }
}
void throttle_down(void * user_data, pin_t pin, uint32_t value) {
  if (ignition_on) {
  printf("[PULSE_GEN] Throttle Down\n");
  rpm-=RPM_CHANGE;
    if (rpm <= IDLE_RPM) rpm=IDLE_RPM;
  }
}

void chip_timer_event(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
  
  // Update total running time
  total_time_running += (float)interval / 1000000.0f;
  
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
  float pulse_width = total_period * 0.03f;   // LOW time = 3% of total period (realistic)
  
  time_running += (float)interval / 1000000.0f;

  // State machine: trigger every total_period, pulse width = 3% of period
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