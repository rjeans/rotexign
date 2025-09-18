// MIT License
// Copyright (c) 2023 MicroBeaut
// Modified for rotexign project - Physics-based angular position with one-shot timers

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Physical constants
static const float PULSE_WIDTH_DEGREES = 6.0f;  // Pulse spans 6 degrees of rotation
static const float DEGREES_PER_REVOLUTION = 360.0f;
static const float PULSES_PER_REVOLUTION = 2.0f;  // Two pulses per revolution

// Timing constants  
static const float IDLE_DURATION_S = 3.0f;
static const float IDLE_RPM = 250.0f;
static const float MAX_RPM = 6900.0f;
static const float ACCELERATION_TIME_S = 5.0f;
static const float MAX_RPM_DURATION_S = 0.0f;  

// State machine states
typedef enum {
    STATE_WAITING_FOR_READY,
    STATE_IDLING,
    STATE_ACCELERATING,
    STATE_AT_MAX_RPM,
    STATE_DECELERATING,
    STATE_FINAL_IDLE,
    STATE_STOPPED
} SimulatorState;

// Pin structure
typedef struct {
    pin_t pin_out_pulse;
    pin_t pin_ready;
    pin_t pin_spark;
} chip_state_t;

// Global state
static chip_state_t *global_chip = NULL;
static SimulatorState sim_state = STATE_WAITING_FOR_READY;
static float state_start_time = 0;
static float last_pulse_time = 0;
static uint32_t pulse_count = 0;
static uint32_t revolution_count = 0;
static bool is_ready = false;

// Angular motion parameters for current state
static float initial_omega = 0;  // Angular velocity at start of state (deg/s)
static float alpha = 0;          // Angular acceleration (deg/s²)

// Timer IDs
static timer_t pulse_start_timer_id;
static timer_t pulse_end_timer_id;
static timer_t monitoring_timer_id;
static timer_t noise_timer_id;

// Noise parameters
static int noise_duration_us = 1000;
static bool noise_active = false;

// Function prototypes
static void pulse_start_timer_event(void *user_data);
static void pulse_end_timer_event(void *user_data);
static void monitoring_timer_event(void *user_data);
static void noise_timer_event(void *user_data);
static void spark_handler(void *user_data, pin_t pin, uint32_t value);
static void ready_pin_change(void *user_data, pin_t pin, uint32_t value);
static void schedule_next_pulse(void);
static float get_current_time_ms(void);

// Get current time in milliseconds
static float get_current_time_ms(void) {
    return get_sim_nanos() / 1000000.0f;
}

// Convert RPM to degrees/second
static float rpm_to_deg_per_sec(float rpm) {
    return rpm * 6.0f; // RPM * 360/60 = RPM * 6
}

// Solve for time to reach angle: angle = omega0*t + 0.5*alpha*t^2
// Returns time in seconds from now to reach target angle
static float time_to_angle(float target_deg, float omega0_deg_s, float alpha_deg_s2) {
    if (fabs(alpha_deg_s2) < 0.001f) {
        // Constant velocity
        if (fabs(omega0_deg_s) < 0.001f) return -1;
        return target_deg / omega0_deg_s;
    }
    
    // Quadratic: 0.5*alpha*t^2 + omega0*t - target = 0
    float a = 0.5f * alpha_deg_s2;
    float b = omega0_deg_s;
    float c = -target_deg;
    
    float discriminant = b*b - 4*a*c;
    if (discriminant < 0) return -1;
    
    float sqrt_disc = sqrtf(discriminant);
    float t1 = (-b + sqrt_disc) / (2*a);
    float t2 = (-b - sqrt_disc) / (2*a);
    
    // Return smallest positive time
    if (t1 > 0 && t2 > 0) return fminf(t1, t2);
    if (t1 > 0) return t1;
    if (t2 > 0) return t2;
    return -1;
}

// Schedule the next pulse based on current angular motion
static void schedule_next_pulse(void) {
    if (sim_state == STATE_WAITING_FOR_READY || sim_state == STATE_STOPPED) return;
    
    float current_time = get_current_time_ms() / 1000.0f;
    float time_in_state = current_time - state_start_time;
    
    // Calculate current angular velocity
    float current_omega = initial_omega + alpha * time_in_state;
    
    // SAFETY: Clamp omega to never exceed MAX_RPM during acceleration
    // This prevents overshoot due to timing granularity
    if (sim_state == STATE_ACCELERATING) {
        float max_omega = rpm_to_deg_per_sec(MAX_RPM);
        if (current_omega > max_omega) {
            current_omega = max_omega;
        }
    }
    
    // We need to travel 180 degrees to next pulse (half revolution)
    float degrees_to_next_pulse = 180.0f;
    
    // Calculate time to next pulse start (at 0 degrees of next half-revolution)
    float time_to_pulse_start = time_to_angle(degrees_to_next_pulse, current_omega, alpha);
    
    if (time_to_pulse_start > 0) {
        // Schedule pulse start
        uint32_t delay_us = (uint32_t)(time_to_pulse_start * 1000000.0f);
        timer_start(pulse_start_timer_id, delay_us, false);
        
        // Calculate angular velocity at pulse start time
        float omega_at_pulse = current_omega + alpha * time_to_pulse_start;
        
        // SAFETY: Also clamp omega at pulse time to never exceed MAX_RPM
        if (sim_state == STATE_ACCELERATING) {
            float max_omega = rpm_to_deg_per_sec(MAX_RPM);
            if (omega_at_pulse > max_omega) {
                omega_at_pulse = max_omega;
            }
        }
        
        // Calculate time for pulse width (6 degrees)
        float pulse_duration = time_to_angle(PULSE_WIDTH_DEGREES, omega_at_pulse, alpha);
        
        if (pulse_duration > 0) {
            // Schedule pulse end relative to pulse start
            uint32_t pulse_duration_us = (uint32_t)(pulse_duration * 1000000.0f);
            uint32_t pulse_end_delay_us = delay_us + pulse_duration_us;
            timer_start(pulse_end_timer_id, pulse_end_delay_us, false);
        }
        
        last_pulse_time = current_time + time_to_pulse_start;
    }
}

// Pulse start event - output goes LOW
static void pulse_start_timer_event(void *user_data) {
    if (!noise_active && global_chip) {
        pin_write(global_chip->pin_out_pulse, LOW);
        pulse_count++;
        
        if (pulse_count % 2 == 0) {
            revolution_count++;
        }
    }
}

// Pulse end event - output goes HIGH and schedule next pulse
static void pulse_end_timer_event(void *user_data) {
    if (!noise_active && global_chip) {
        pin_write(global_chip->pin_out_pulse, HIGH);
    }
    
    // Schedule the next pulse
    schedule_next_pulse();
}

// Monitoring timer for state transitions and diagnostics
static void monitoring_timer_event(void *user_data) {
    float current_time = get_current_time_ms() / 1000.0f;
    float time_in_state = current_time - state_start_time;
    
    switch(sim_state) {
        case STATE_WAITING_FOR_READY:
            // Ensure output is HIGH
            if (global_chip) {
                pin_write(global_chip->pin_out_pulse, HIGH);
            }
            break;
            
        case STATE_IDLING:
            if (time_in_state >= IDLE_DURATION_S) {
                // Transition to acceleration
                sim_state = STATE_ACCELERATING;
                state_start_time = current_time;
                
                // Set up angular motion for acceleration
                initial_omega = rpm_to_deg_per_sec(IDLE_RPM);
                float final_omega = rpm_to_deg_per_sec(MAX_RPM);
                alpha = (final_omega - initial_omega) / ACCELERATION_TIME_S;
                
                printf("[PULSE_GEN] t=%.1fs Starting acceleration: %.0f to %.0f RPM over %.1fs\n",
                       current_time, IDLE_RPM, MAX_RPM, ACCELERATION_TIME_S);
                printf("[PULSE_GEN] t=%.1fs Angular: ω₀=%.1f°/s, α=%.1f°/s²\n",
                       current_time, initial_omega, alpha);
            }
            break;
            
        case STATE_ACCELERATING:
            if (time_in_state >= ACCELERATION_TIME_S) {
                // Transition to max RPM
                sim_state = STATE_AT_MAX_RPM;
                
                // Calculate the target angular velocity (what we wanted to reach)
                float target_omega = rpm_to_deg_per_sec(MAX_RPM);
                
                // Calculate what the actual angular velocity would be at current time
                float actual_omega = initial_omega + alpha * time_in_state;
                
                // IMPORTANT: Clamp to never exceed target MAX_RPM
                if (actual_omega > target_omega) {
                    actual_omega = target_omega;
                }
                
                float actual_rpm = actual_omega / 6.0f;
                
                // Set up angular motion for constant RPM at the clamped velocity
                state_start_time = current_time;
                initial_omega = actual_omega;  // Use clamped velocity
                alpha = 0;  // No more acceleration
                
                printf("[PULSE_GEN] t=%.2fs Reached MAX RPM %.0f (clamped to %.0f) at %.2fs\n", 
                       current_time, MAX_RPM, actual_rpm, current_time);}
            break;
            
        case STATE_AT_MAX_RPM:
            if (time_in_state >= MAX_RPM_DURATION_S) {
                // Transition to deceleration
                sim_state = STATE_DECELERATING;
                
                // Use the current actual angular velocity (which should be constant in this state)
                float current_omega = initial_omega;  // No acceleration in MAX_RPM state
                
                // Set up angular motion for deceleration to 0 RPM (simulating ignition off)
                state_start_time = current_time;
                initial_omega = current_omega;  // Start from actual current velocity
                float final_omega = 0;  // Decelerate to 0 RPM
                alpha = (final_omega - initial_omega) / ACCELERATION_TIME_S;
                
                printf("[PULSE_GEN] t=%.1fs Starting deceleration: %.0f to 0 RPM over %.1fs (ignition off)\n",
                       current_time, MAX_RPM, ACCELERATION_TIME_S);
                printf("[PULSE_GEN] t=%.1fs Angular: ω₀=%.1f°/s, α=%.1f°/s²\n",
                       current_time, initial_omega, alpha);
            }
            break;
            
        case STATE_DECELERATING:
            if (time_in_state >= ACCELERATION_TIME_S) {
                // Transition to stopped state (0 RPM) to simulate turning ignition off
                sim_state = STATE_STOPPED;
                
                // Calculate actual velocity at end of deceleration (should be ~0)
                float actual_omega = initial_omega + alpha * ACCELERATION_TIME_S;
                float actual_rpm = actual_omega / 6.0f;
                
                // Stop all angular motion
                state_start_time = current_time;
                initial_omega = 0;  // Force to exactly 0 for stopped state
                alpha = 0;
                
                // Cancel any pending pulse timers
                timer_stop(pulse_start_timer_id);
                timer_stop(pulse_end_timer_id);
                
                // Ensure output is HIGH (no pulse)
                if (global_chip) {
                    pin_write(global_chip->pin_out_pulse, HIGH);
                }
                
                printf("[PULSE_GEN] t=%.2fs Engine stopped (0 RPM) - simulating ignition off at %.2fs\n", 
                       current_time, current_time);
            }
            break;
            
        case STATE_FINAL_IDLE:
            // Stay in final idle state indefinitely
            break;
            
        case STATE_STOPPED:
            // Stay stopped - no more pulses
            break;
    }
    
    // Periodic diagnostics every 2 seconds
    static float last_diagnostic = 0;
    if (current_time - last_diagnostic >= 2.0f && sim_state != STATE_WAITING_FOR_READY) {
        float current_rpm = (initial_omega + alpha * time_in_state) / 6.0f;
        const char* state_names[] = {"WAITING", "IDLING", "ACCELERATING", "MAX_RPM", "DECELERATING", "FINAL_IDLE", "STOPPED"};
        
        printf("[PULSE_GEN] t=%.1fs State=%s RPM=%.0f Pulses=%u Revs=%u\n",
               current_time, state_names[sim_state], current_rpm, pulse_count, revolution_count);
        
        last_diagnostic = current_time;
    }
}

// Spark noise handler
static void spark_handler(void *user_data, pin_t pin, uint32_t value) {
    chip_state_t *chip = (chip_state_t*)user_data;
    if (!is_ready) return;  // Ignore sparks if not ready
    // 10% chance to inject noise
    if ((rand() % 10) == 0 && !noise_active) {
        noise_active = true;
        pin_write(chip->pin_out_pulse, LOW);
        timer_start(noise_timer_id, noise_duration_us, false);
    }
}

// Noise timer - restore pin to HIGH
static void noise_timer_event(void *user_data) {
    chip_state_t *chip = (chip_state_t*)user_data;
    noise_active = false;
    pin_write(chip->pin_out_pulse, HIGH);
}

// Ready pin handler - start simulation
static void ready_pin_change(void *user_data, pin_t pin, uint32_t value) {
    float current_time = get_current_time_ms() / 1000.0f;

    is_ready = (value == HIGH);

    // Start idling
    sim_state = STATE_IDLING;
    state_start_time = current_time;
    pulse_count = 0;
    revolution_count = 0;
    
    // Set up angular motion for idling (constant RPM)
    initial_omega = rpm_to_deg_per_sec(IDLE_RPM);
    alpha = 0;  // No acceleration during idle
    
    // Schedule first pulse
    schedule_next_pulse();
    
    printf("[PULSE_GEN] READY triggered - Starting idle at %.0f RPM\n", IDLE_RPM);
}

// Initialize the chip
void chip_init(void) {
    chip_state_t *chip = malloc(sizeof(chip_state_t));
    global_chip = chip;
    
    chip->pin_out_pulse = pin_init("PULSE", OUTPUT_HIGH);
    chip->pin_ready = pin_init("READY", INPUT);
    chip->pin_spark = pin_init("SPARK", INPUT);
    
    // Set up timers
    const timer_config_t pulse_start_config = {
        .callback = pulse_start_timer_event,
        .user_data = chip,
    };
    
    const timer_config_t pulse_end_config = {
        .callback = pulse_end_timer_event,
        .user_data = chip,
    };
    
    const timer_config_t monitoring_config = {
        .callback = monitoring_timer_event,
        .user_data = chip,
    };
    
    const timer_config_t noise_config = {
        .callback = noise_timer_event,
        .user_data = chip,
    };
    
    pulse_start_timer_id = timer_init(&pulse_start_config);
    pulse_end_timer_id = timer_init(&pulse_end_config);
    monitoring_timer_id = timer_init(&monitoring_config);
    noise_timer_id = timer_init(&noise_config);
    
    // Start monitoring timer (100ms interval)
    timer_start(monitoring_timer_id, 100000, true);
    
    // Set up pin watches
    const pin_watch_config_t spark_config = {
        .edge = RISING,
        .pin_change = spark_handler,
        .user_data = chip
    };
    
    const pin_watch_config_t ready_config = {
        .edge = RISING,
        .pin_change = ready_pin_change,
        .user_data = chip
    };
    
    pin_watch(chip->pin_spark, &spark_config);
    pin_watch(chip->pin_ready, &ready_config);
    
    printf("[PULSE_GEN] Initialized - Physics-based timing with one-shot timers\n");
    printf("[PULSE_GEN] Settings: Idle %.0f RPM for %.1fs, accelerate to %.0f RPM in %.1fs, then decelerate to 0\n",
           IDLE_RPM, IDLE_DURATION_S, MAX_RPM, ACCELERATION_TIME_S);
}