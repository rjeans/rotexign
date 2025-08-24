/*
 * Advanced Arduino Ignition Timing Controller
 * For High-Performance Jet Ski Applications (Rotax 787)
 * 
 * Features:
 * - Sub-microsecond timing precision using Timer1 Input Capture
 * - One-revolution-ahead scheduling for high advance angles
 * - Smart coil (1GN-1A) support with dwell control
 * - Comprehensive safety systems with rev limiting
 * - Serial tuning interface with real-time diagnostics
 * 
 * Hardware:
 * - Arduino Uno/Nano (ATmega328P)
 * - Optocoupler for magneto pickup isolation
 * - Smart coil driver output
 * - Relay-based startup protection
 */

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

// ============================================================================
// PIN CONFIGURATION
// ============================================================================
namespace Pins {
  const uint8_t TRIGGER_INPUT = 2;      // INT0: Magneto pickup (falling edge HIGH→LOW)
  const uint8_t SPARK_OUTPUT = 3;       // Coil control (HIGH=dwell, LOW=spark)
  const uint8_t STATUS_LED = 13;        // Status indicator
  const uint8_t OPTO_SHORT_RELAY = 4;   // Removes optocoupler short after boot
}

// ============================================================================
// ENGINE CONFIGURATION
// ============================================================================
namespace Engine {
  const uint8_t PULSES_PER_REV = 2;     // Two flywheel lobes
  const float TRIGGER_ANGLE_BTDC = 47.0f; // Trigger occurs 47° BTDC
  
  
  // RPM thresholds
  const uint16_t MIN_RPM = 200;
  const uint16_t CRANKING_RPM = 400;
  // Previous-lobe timing is determined dynamically based on available window
  // Typically switches around 1800-2000 RPM when window < dwell+margin
  const uint16_t REV_LIMIT = 7000;
  const uint16_t MAX_RPM = 8000;
  const uint16_t REV_LIMIT_HYSTERESIS = 100;
  
  // Timing limits
  const uint16_t TIMEOUT_MS = 1000;     // Engine stop detection
  const uint16_t STARTUP_DELAY_MS = 500;  // Relay release delay (reduced for testing)
}

// ============================================================================
// COIL CONFIGURATION (1GN-1A Smart Coil)
// ============================================================================
namespace Coil {
  const uint16_t DWELL_MS = 3;          // Nominal dwell time at 12V
  const uint16_t MAX_DWELL_MS = 9;      // Maximum safe dwell at 14V
  const uint8_t MAX_DUTY_PERCENT = 40;  // Maximum duty cycle
  const bool ACTIVE_HIGH = true;        // HIGH starts dwell, LOW fires spark
}

// ============================================================================
// TIMING CURVES (RPM vs Advance in degrees BTDC)
// ============================================================================
namespace TimingCurves {
  const uint16_t rpm_points[] PROGMEM = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000};
  const int8_t safe_advance[] PROGMEM = {0, 6, 12, 15, 15, 14, 13, 12};
  const int8_t perf_advance[] PROGMEM = {0, 6, 12, 20, 20, 19, 18, 17};
  const uint8_t num_points = sizeof(rpm_points) / sizeof(rpm_points[0]);
}

// ============================================================================
// SYSTEM STATE
// ============================================================================
struct SystemState {
  // Engine state
  volatile bool engine_running = false;
  volatile uint16_t current_rpm = 0;
  uint16_t filtered_rpm = 0;
  bool rev_limit_active = false;
  bool use_safe_curve = true;
  
  // Timing data
  volatile uint16_t last_trigger_capture = 0;
  volatile uint16_t prev_trigger_capture = 0;
  volatile uint32_t trigger_period_ticks = 0;
  volatile uint32_t prev_period_ticks = 0;
  volatile bool new_trigger_available = false;
  volatile bool have_previous_trigger = false;
  
  // Timer1 overflow tracking for periods > 32.768ms
  volatile uint16_t timer1_overflows = 0;
  volatile uint32_t last_trigger_timestamp = 0;  // Full 32-bit timestamp
  
  // Software timing for low RPM when Timer1 would wraparound
  bool using_software_timing = false;
  uint32_t software_dwell_start_us = 0;
  uint32_t software_spark_target_us = 0;
  bool software_spark_pending = false;
  bool software_dwell_active = false;
  
  // Diagnostics
  volatile uint32_t trigger_count = 0;
  volatile uint32_t spark_count = 0;
  volatile uint16_t glitch_count = 0;
  volatile uint16_t rev_limit_events = 0;
  volatile uint16_t overflow_protection_events = 0;
  volatile uint8_t missed_triggers = 0;
  
  // Error flags
  uint8_t error_flags = 0;
  static const uint8_t ERROR_OVERSPEED = 0x01;
  static const uint8_t ERROR_NO_SIGNAL = 0x04;
  static const uint8_t ERROR_INVALID_RPM = 0x08;
  
  // Timing tracking
  uint32_t last_trigger_millis = 0;
  uint32_t last_diagnostic_millis = 0;
  uint32_t scheduled_dwell_ticks = 0;
  volatile bool last_used_previous_lobe = false;  // Track actual mode used
  
  // Startup state
  bool relays_initialized = false;
};

SystemState sys;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================
namespace Utils {
  inline uint32_t us_to_ticks(uint32_t microseconds) {
    return microseconds * 2UL;  // 0.5µs per tick at prescaler 8
  }
  
  inline uint32_t ms_to_ticks(uint32_t milliseconds) {
    return milliseconds * 2000UL;
  }
  
  inline uint32_t ticks_to_us(uint32_t ticks) {
    return ticks / 2UL;
  }
  
  uint16_t calculate_rpm(uint32_t period_ticks) {
    if (period_ticks == 0) return 0;
    
    uint32_t period_us = ticks_to_us(period_ticks);
    if (period_us > 300000UL) return 0;  // < 200 RPM
    
    uint32_t rpm = (60000000UL / period_us) / Engine::PULSES_PER_REV;
    return (rpm > 65535) ? 0 : (uint16_t)rpm;
  }
  
  uint16_t filter_rpm(uint16_t new_rpm, uint16_t old_rpm) {
    if (old_rpm == 0) return new_rpm;
    
    int16_t diff = (int16_t)new_rpm - (int16_t)old_rpm;
    int16_t abs_diff = (diff < 0) ? -diff : diff;
    
    // Adaptive filtering: less filtering for rapid changes (sweep mode)
    if (abs_diff > 200) {
      // Large change - minimal filtering (alpha = 0.75)  
      return (uint16_t)(old_rpm + (diff * 3 >> 2));
    } else if (abs_diff > 50) {
      // Medium change - moderate filtering (alpha = 0.5)
      return (uint16_t)(old_rpm + (diff >> 1));
    } else {
      // Small change - normal filtering (alpha = 0.25)
      return (uint16_t)(old_rpm + (diff >> 2));
    }
  }
}

// ============================================================================
// TIMING CURVE INTERPOLATION
// ============================================================================
float get_advance_angle(uint16_t rpm) {
  const int8_t* curve = sys.use_safe_curve ? 
    TimingCurves::safe_advance : TimingCurves::perf_advance;
  
  // Handle edge cases
  uint16_t first_rpm = pgm_read_word_near(&TimingCurves::rpm_points[0]);
  if (rpm <= first_rpm) {
    return (float)pgm_read_byte_near(&curve[0]);
  }
  
  // Check if beyond last point FIRST
  uint16_t last_rpm = pgm_read_word_near(&TimingCurves::rpm_points[TimingCurves::num_points - 1]);
  if (rpm >= last_rpm) {
    // Return exact last advance value, no interpolation
    return (float)pgm_read_byte_near(&curve[TimingCurves::num_points - 1]);
  }
  
  // Linear interpolation for points within the curve
  for (uint8_t i = 1; i < TimingCurves::num_points; i++) {
    uint16_t rpm_high = pgm_read_word_near(&TimingCurves::rpm_points[i]);
    if (rpm <= rpm_high) {
      uint16_t rpm_low = pgm_read_word_near(&TimingCurves::rpm_points[i-1]);
      int8_t adv_low = pgm_read_byte_near(&curve[i-1]);
      int8_t adv_high = pgm_read_byte_near(&curve[i]);
      
      float ratio = (float)(rpm - rpm_low) / (rpm_high - rpm_low);
      return adv_low + (adv_high - adv_low) * ratio;
    }
  }
  
  // This should never be reached now
  return (float)pgm_read_byte_near(&curve[TimingCurves::num_points - 1]);
}

// ============================================================================
// COIL CONTROL
// ============================================================================
namespace CoilControl {
  // Pin 3 (PD3) control for spark output
  inline void set_output_high() { PORTD |= _BV(PD3); }
  inline void set_output_low() { PORTD &= ~_BV(PD3); }
  
  void start_dwell() {
    // For 1GN-1A smart coil: HIGH = start charging (dwell)
    if (Coil::ACTIVE_HIGH) set_output_high(); 
    else set_output_low();
    // Dwell marker removed - not required
  }
  
  void fire_spark() {
    // For 1GN-1A smart coil: LOW = fire spark (end dwell)
    if (Coil::ACTIVE_HIGH) set_output_low(); 
    else set_output_high();
    // Dwell marker removed - not required
  }
  
  void safe_idle() {
    // Ensure coil is not charging (LOW for 1GN-1A)
    if (Coil::ACTIVE_HIGH) set_output_low();
    else set_output_high();
    // Dwell marker removed - not required
  }
}

// ============================================================================
// TIMER AND INTERRUPT CONFIGURATION
// ============================================================================
namespace TimerControl {
  void initialize() {
    cli();
    
    // Reset Timer1
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    
    // Prescaler = 8 (0.5µs resolution at 16MHz)
    TCCR1B = _BV(CS11);
    
    // Enable Timer1 overflow interrupt for tracking periods > 32.768ms
    TIMSK1 = _BV(TOIE1);
    
    // Configure INT0 (D2) for falling edge (HIGH→LOW trigger)
    EICRA = (1 << ISC01);  // Falling edge
    EIFR = (1 << INTF0);   // Clear pending
    EIMSK = (1 << INT0);   // Enable
    
    sei();
  }
  
  void cancel_scheduled_events() {
    // Disable compare interrupt (keep overflow interrupt enabled)
    TIMSK1 = _BV(TOIE1);  // Only overflow interrupt remains
    // Clear pending compare flag
    TIFR1 = _BV(OCF1A);
  }
  
  void schedule_spark_event(uint16_t dwell_start, uint16_t spark_time) {
    noInterrupts();
    
    // Check if we have pending spark event that hasn't fired yet
    if (TIMSK1 & (_BV(OCIE1A) | _BV(OCIE1B))) {
      // Previous spark event still pending - skip this one to avoid corruption
      interrupts();
      return;
    }
    
    uint16_t current_time = TCNT1;
    const uint16_t MIN_MARGIN = 32;  // 16µs minimum scheduling margin
    
    // Calculate time until spark (handles wraparound correctly)
    uint16_t spark_delta = spark_time - current_time;
    
    // If spark is in the past or too far future (wraparound), skip this event
    if (spark_delta > 32768) {
      // Spark time wrapped around to past - skip
      interrupts();
      return;
    }
    
    // If spark is too soon to schedule reliably, skip
    if (spark_delta < MIN_MARGIN) {
      interrupts();
      return;
    }
    
    // Check if dwell start time has already passed
    uint16_t dwell_delta = dwell_start - current_time;
    bool dwell_in_past = (dwell_delta > 32768) || (dwell_delta < MIN_MARGIN);
    
    if (dwell_in_past) {
      // Previous-lobe mode: dwell start time has passed, start immediately
      CoilControl::start_dwell();
      
      // Calculate adjusted spark time to maintain full dwell duration
      uint32_t target_dwell_ticks = Utils::ms_to_ticks(Coil::DWELL_MS);
      uint16_t adjusted_spark_time = current_time + (uint16_t)target_dwell_ticks;
      
      // Use adjusted spark time to maintain proper dwell duration
      OCR1A = adjusted_spark_time;
      TIFR1 = _BV(OCF1A);  // Clear pending compare A flag
      TIMSK1 = _BV(TOIE1) | _BV(OCIE1A);  // Enable only Compare A
    } else {
      // Same-lobe mode: both dwell and spark are in future
      // Schedule both events with dual compare matches
      OCR1B = dwell_start;  // Compare B for dwell start
      OCR1A = spark_time;   // Compare A for spark
      
      // Clear any pending compare interrupt flags
      TIFR1 = _BV(OCF1A) | _BV(OCF1B);
      
      // Enable both compare interrupts
      TIMSK1 = _BV(TOIE1) | _BV(OCIE1A) | _BV(OCIE1B);
    }
    
    interrupts();
  }
}

// ============================================================================
// SOFTWARE TIMING FOR LOW RPM (when Timer1 would wraparound)
// ============================================================================
namespace SoftwareTiming {
  void cancel_pending_spark() {
    if (sys.software_dwell_active) {
      // Ensure coil is in safe idle state
      CoilControl::safe_idle();
    }
    sys.software_spark_pending = false;
    sys.software_dwell_active = false;
    sys.using_software_timing = false;
  }
  
  void schedule_spark(uint32_t dwell_delay_us, uint32_t spark_delay_us) {
    uint32_t current_us = micros();
    
    // Calculate absolute target times (handle micros() wraparound)
    sys.software_dwell_start_us = current_us + dwell_delay_us;
    sys.software_spark_target_us = current_us + spark_delay_us;
    
    // Safety check: ensure dwell duration is reasonable
    uint32_t dwell_duration_us = spark_delay_us - dwell_delay_us;
    if (dwell_duration_us > (Coil::MAX_DWELL_MS * 1000)) {
      // Dwell too long - start dwell later to limit duration
      sys.software_dwell_start_us = sys.software_spark_target_us - (Coil::DWELL_MS * 1000);
    }
    
    sys.software_spark_pending = true;
    sys.software_dwell_active = false;
    sys.using_software_timing = true;
  }
  
  void process_pending_events() {
    if (!sys.software_spark_pending) return;
    
    uint32_t current_us = micros();
    
    // Handle dwell start (with micros() wraparound consideration)
    if (!sys.software_dwell_active) {
      // Check if target time has been reached
      // Using signed comparison to handle wraparound correctly
      int32_t time_until_dwell = (int32_t)(sys.software_dwell_start_us - current_us);
      bool dwell_time_reached = (time_until_dwell <= 0);
      
      if (dwell_time_reached) {
        CoilControl::start_dwell();
        sys.software_dwell_active = true;
      }
    }
    
    // Handle spark event (with micros() wraparound consideration)
    if (sys.software_dwell_active) {
      // Check if target time has been reached
      // Using signed comparison to handle wraparound correctly
      int32_t time_until_spark = (int32_t)(sys.software_spark_target_us - current_us);
      bool spark_time_reached = (time_until_spark <= 0);
      
      if (spark_time_reached) {
        CoilControl::fire_spark();
        sys.spark_count++;
        cancel_pending_spark();
      }
    }
  }
}

// ============================================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================================

// Timer1 overflow interrupt - tracks overflows for periods > 32.768ms
ISR(TIMER1_OVF_vect) {
  sys.timer1_overflows++;
}

ISR(INT0_vect) {
  uint16_t capture_time = TCNT1;
  static bool first_trigger = true;
  
  // Build 32-bit timestamp from overflow counter + Timer1 value
  // Each overflow = 65536 ticks = 32.768ms
  uint32_t current_timestamp = ((uint32_t)sys.timer1_overflows << 16) | capture_time;
  
  if (first_trigger) {
    first_trigger = false;
    sys.last_trigger_timestamp = current_timestamp;
    sys.last_trigger_capture = capture_time;
    return;  // Skip first trigger (no valid period yet)
  }
  
  // Calculate period using full 32-bit timestamps
  uint32_t delta_ticks = current_timestamp - sys.last_trigger_timestamp;
  
  // Reject glitches (minimum 1ms = 2000 ticks between valid triggers) 
  // This allows up to 30,000 RPM theoretical maximum
  if (delta_ticks < 2000) {
    sys.glitch_count++;
    return;
  }
  
  // Reject unreasonably long periods (> 300ms = 200 RPM minimum)
  if (delta_ticks > 600000) {
    sys.glitch_count++;
    return;
  }
  
  // Store previous trigger data
  sys.prev_trigger_capture = sys.last_trigger_capture;
  sys.prev_period_ticks = sys.trigger_period_ticks;
  sys.have_previous_trigger = true;
  
  // Accept this trigger
  sys.last_trigger_capture = capture_time;
  sys.last_trigger_timestamp = current_timestamp;
  sys.trigger_period_ticks = delta_ticks;
  sys.new_trigger_available = true;
  sys.trigger_count++;
  sys.missed_triggers = 0;
  sys.engine_running = true;
}

ISR(TIMER1_COMPA_vect) {
  // Spark event: end dwell
  CoilControl::fire_spark();
  // Disable both compare interrupts, keep overflow interrupt
  TIMSK1 = _BV(TOIE1);
  sys.spark_count++;
}

ISR(TIMER1_COMPB_vect) {
  // Dwell start event: begin charging coil
  CoilControl::start_dwell();
  // Disable Compare B interrupt after use to prevent retriggering
  // Keep Compare A enabled for the spark event
  TIMSK1 &= ~_BV(OCIE1B);
}

// ============================================================================
// ENGINE MANAGEMENT
// ============================================================================
namespace EngineManager {
  void check_timeout() {
    uint32_t current_time = millis();
    
    if (sys.engine_running && 
        (current_time - sys.last_trigger_millis) > Engine::TIMEOUT_MS) {
      sys.engine_running = false;
      CoilControl::safe_idle();
      TimerControl::cancel_scheduled_events();
      sys.error_flags |= SystemState::ERROR_NO_SIGNAL;
      sys.missed_triggers++;
      
      // SAFETY: Re-engage protection when engine stops
      if (sys.relays_initialized) {
        digitalWrite(Pins::OPTO_SHORT_RELAY, LOW);  // D4 LOW = Short opto (disable)
        sys.relays_initialized = false;
        sys.trigger_count = 0;  // Reset trigger count for next start
        sys.timer1_overflows = 0;  // Reset overflow counter
        SoftwareTiming::cancel_pending_spark();  // Cancel any software timing
        Serial.println(F("Engine stopped - protection re-engaged"));
      }
    } else {
      sys.error_flags &= ~SystemState::ERROR_NO_SIGNAL;
    }
  }
  
  bool check_rev_limit(uint16_t rpm) {
    if (!sys.rev_limit_active && rpm > Engine::REV_LIMIT) {
      sys.rev_limit_active = true;
      sys.rev_limit_events++;
      sys.error_flags |= SystemState::ERROR_OVERSPEED;
      return true;
    } else if (sys.rev_limit_active && 
               rpm < (Engine::REV_LIMIT - Engine::REV_LIMIT_HYSTERESIS)) {
      sys.rev_limit_active = false;
      sys.error_flags &= ~SystemState::ERROR_OVERSPEED;
    }
    return sys.rev_limit_active;
  }
  
  
  void calculate_and_schedule_spark() {
    // CRITICAL SAFETY: Don't schedule sparks while protection relays are active
    if (!sys.relays_initialized) {
      // Protection is still active - coil is grounded, opto is shorted
      // Do NOT attempt to schedule any spark events
      return;
    }
    
    // This is the main timing calculation function
    if (!sys.have_previous_trigger || sys.trigger_period_ticks == 0 || sys.prev_period_ticks == 0) {
      sys.error_flags |= SystemState::ERROR_INVALID_RPM;
      return;
    }
    
    // STARTUP PROTECTION: Skip first 5 triggers to ensure stable data
    if (sys.trigger_count < 5) {
      sys.error_flags |= SystemState::ERROR_INVALID_RPM;
      return;
    }
    
    // Additional startup safety: ensure we have reasonable trigger data
    // Validate that both current and previous periods are within acceptable range
    uint32_t min_valid_period = Utils::ms_to_ticks(3);   // 3ms = 10000 RPM max
    uint32_t max_valid_period = Utils::ms_to_ticks(300); // 300ms = 200 RPM min
    if (sys.trigger_period_ticks < min_valid_period || sys.trigger_period_ticks > max_valid_period ||
        sys.prev_period_ticks < min_valid_period || sys.prev_period_ticks > max_valid_period) {
      sys.error_flags |= SystemState::ERROR_INVALID_RPM;
      return;
    }
    

    uint16_t rpm = Utils::calculate_rpm(sys.trigger_period_ticks);
    if (rpm < Engine::MIN_RPM || rpm > Engine::MAX_RPM) {
      sys.error_flags |= SystemState::ERROR_INVALID_RPM;
      return;
    }

    sys.error_flags &= ~SystemState::ERROR_INVALID_RPM;
    sys.current_rpm = rpm;
    sys.filtered_rpm = Utils::filter_rpm(rpm, sys.filtered_rpm);

    // Rev limiter
    if (check_rev_limit(sys.filtered_rpm)) {
      return;
    }

    // Get advance angle from curve
    float advance = get_advance_angle(sys.filtered_rpm);
    float delta_angle = Engine::TRIGGER_ANGLE_BTDC - advance;
    
    // Normalize angle
    while (delta_angle < 0) delta_angle += 360.0f;
    while (delta_angle >= 360.0f) delta_angle -= 360.0f;

    // Calculate timing - CRITICAL FIX: Always relative to current trigger
    uint32_t period = sys.trigger_period_ticks;
    uint32_t delay_ticks = (uint32_t)(period * (delta_angle / 180.0f));
    
    // CRITICAL: Check if Timer1 compare register would wraparound
    uint32_t trigger_32 = sys.last_trigger_capture;
    uint32_t spark_time_32 = trigger_32 + delay_ticks;
    
    // If spark time > 65535, OCR1A wraps and timing fails
    bool timer1_would_wrap = (spark_time_32 > 65535);
    
    uint16_t spark_time = (uint16_t)spark_time_32;

    // Dwell calculation
    uint32_t target_dwell_ticks = Utils::ms_to_ticks(Coil::DWELL_MS);
    uint32_t ticks_per_rev = period * Engine::PULSES_PER_REV;
    uint32_t max_dwell_ticks = (ticks_per_rev * Coil::MAX_DUTY_PERCENT) / 100UL;
    uint32_t dwell_ticks = (target_dwell_ticks < max_dwell_ticks) ? target_dwell_ticks : max_dwell_ticks;

    const uint16_t SAFETY_MARGIN_TICKS = 600; // ~0.30 ms
    
    // Check if same-lobe dwell fits - use 32-bit arithmetic to avoid wraparound issues
    uint32_t earliest_dwell_start_same_32 = trigger_32 + SAFETY_MARGIN_TICKS;
    uint16_t earliest_dwell_start_same = (uint16_t)earliest_dwell_start_same_32;
    
    // FIXED: Use 32-bit arithmetic for dwell timing to prevent wraparound issues
    bool same_lobe_fits;
    uint32_t required_dwell_start_32;
    uint16_t required_dwell_start;
    
    if (dwell_ticks > delay_ticks) {
      // Not enough time - dwell extends before spark time
      same_lobe_fits = false;
      required_dwell_start_32 = 0; // Will be overridden in previous-lobe mode
      required_dwell_start = 0;
    } else {
      required_dwell_start_32 = spark_time_32 - dwell_ticks;
      required_dwell_start = (uint16_t)required_dwell_start_32;
      // Check if dwell start time is achievable (handle wraparound in comparison)
      int16_t dwell_margin = (int16_t)(required_dwell_start - earliest_dwell_start_same);
      same_lobe_fits = (dwell_margin >= 0);
    }
    
    // HYBRID TIMING: If Timer1 would wrap, use software timing for same-lobe
    if (timer1_would_wrap) {
      // Timer1 cannot handle this delay - use software timing instead
      TimerControl::cancel_scheduled_events();
      
      // Calculate software timing delays in microseconds
      uint32_t delay_us = Utils::ticks_to_us(delay_ticks);
      uint32_t dwell_us = Utils::ticks_to_us(dwell_ticks);
      uint32_t dwell_delay_us = (delay_us > dwell_us) ? (delay_us - dwell_us) : 0;
      
      // Use software timing for same-lobe scheduling
      SoftwareTiming::schedule_spark(dwell_delay_us, delay_us);
      
      // Track that we're using same-lobe via software for diagnostics  
      sys.last_used_previous_lobe = false;
      return;
    }
    
    // Decide timing mode - use previous-lobe if same-lobe doesn't fit
    bool use_previous_lobe = !same_lobe_fits;
    
    // Validate previous trigger if needed  
    if (use_previous_lobe && sys.have_previous_trigger) {
      // Compare against the stored 32-bit period instead of Timer1 captures
      // Use the previous period for validation since that's what we're checking
      uint32_t prev_period = sys.prev_period_ticks;
      uint32_t current_period = sys.trigger_period_ticks;
      
      // Check if periods are reasonably similar (within 25%)
      uint32_t min_period = (current_period * 3) / 4;
      uint32_t max_period = (current_period * 5) / 4;
      if (prev_period < min_period || prev_period > max_period) {
        use_previous_lobe = false;
      }
    } else if (use_previous_lobe) {
      use_previous_lobe = false;
    }
    
    
    // Calculate dwell start time based on mode - use 32-bit arithmetic
    uint32_t dwell_start_32;
    if (use_previous_lobe) {
      // Previous-lobe mode: can start dwell as early as previous trigger
      uint32_t earliest_dwell_start_prev_32 = (uint32_t)sys.prev_trigger_capture + SAFETY_MARGIN_TICKS;
      
      // In previous-lobe mode, we have more time available
      // Start dwell as early as safely possible, or at ideal time if later
      if (dwell_ticks <= delay_ticks) {
        // Normal case - ideal dwell start time is achievable
        dwell_start_32 = (required_dwell_start_32 > earliest_dwell_start_prev_32) ? 
                        required_dwell_start_32 : earliest_dwell_start_prev_32;
      } else {
        // High RPM case - ideal dwell would start before spark_time
        // Start as early as safely possible from previous trigger
        dwell_start_32 = earliest_dwell_start_prev_32;
      }
    } else {
      // Same-lobe mode: may need to trim dwell if it doesn't fit
      if (same_lobe_fits) {
        dwell_start_32 = required_dwell_start_32;
      } else {
        // Trim dwell to fit in available window
        uint32_t available_ticks = spark_time_32 - earliest_dwell_start_same_32;
        uint32_t min_dwell_ticks = Utils::ms_to_ticks(1); // Minimum 1ms dwell
        if (available_ticks < min_dwell_ticks) available_ticks = min_dwell_ticks;
        dwell_ticks = available_ticks;
        dwell_start_32 = spark_time_32 - dwell_ticks;
      }
    }
    
    // Convert to 16-bit for Timer1 interface
    uint16_t dwell_start = (uint16_t)dwell_start_32;

    // Overflow protection - check for unreasonable values
    if (delay_ticks > 100000UL || dwell_ticks > 20000UL) {
      sys.overflow_protection_events++;
      return; // Skip this spark event
    }

    // Cancel any pending software timing - we're using hardware Timer1
    SoftwareTiming::cancel_pending_spark();
    
    sys.scheduled_dwell_ticks = dwell_ticks;
    sys.last_used_previous_lobe = use_previous_lobe;  // Track mode for diagnostics
    TimerControl::schedule_spark_event(dwell_start, spark_time);
  }
}

// ============================================================================
// SERIAL INTERFACE
// ============================================================================
namespace SerialInterface {
  void print_status() {
    float advance = get_advance_angle(sys.filtered_rpm);
    float delta = Engine::TRIGGER_ANGLE_BTDC - advance;
    
    // Show actual mode that was used (not a guess based on RPM)
    bool prev_mode = sys.last_used_previous_lobe;
    
    Serial.print(F("RPM: ")); Serial.print(sys.filtered_rpm);
    Serial.print(F(", Period: ")); Serial.print(Utils::ticks_to_us(sys.trigger_period_ticks));
    Serial.print(F("us, Advance: ")); Serial.print(advance);
    Serial.print(F("°, Delta: ")); Serial.print(delta);
    Serial.print(F("°, Dwell: ")); Serial.print(Utils::ticks_to_us(sys.scheduled_dwell_ticks));
    Serial.print(F("us, Engine: ")); Serial.print(sys.engine_running ? F("RUN") : F("STOP"));
    Serial.print(F(", RevLim: ")); Serial.print(sys.rev_limit_active ? F("ON") : F("OFF"));
    Serial.print(F(", Mode: ")); Serial.print(prev_mode ? F("PREV") : F("SAME"));
    if (sys.using_software_timing) Serial.print(F("-SW"));
    Serial.print(F(", OvfProt: ")); Serial.print(sys.overflow_protection_events);
    Serial.print(F(", Errors: 0x")); Serial.println(sys.error_flags, HEX);
  }
  
  void print_diagnostics() {
    Serial.println(F("=== DIAGNOSTICS ==="));
    Serial.print(F("Triggers: ")); Serial.println(sys.trigger_count);
    Serial.print(F("Sparks: ")); Serial.println(sys.spark_count);
    Serial.print(F("Glitches: ")); Serial.println(sys.glitch_count);
    Serial.print(F("Rev Limits: ")); Serial.println(sys.rev_limit_events);
    Serial.print(F("Missed: ")); Serial.println(sys.missed_triggers);
    Serial.print(F("Overflow Events: ")); Serial.println(sys.overflow_protection_events);
    Serial.print(F("Uptime: ")); Serial.print(millis() / 1000); Serial.println(F("s"));
    Serial.print(F("Curve: ")); Serial.println(sys.use_safe_curve ? F("SAFE") : F("PERF"));
    Serial.print(F("Last Mode: ")); Serial.println(sys.last_used_previous_lobe ? F("PREV-LOBE") : F("SAME-LOBE"));
  }
  
  void process_command(char* cmd) {
    // Convert to uppercase
    for (char* p = cmd; *p; ++p) {
      if (*p >= 'a' && *p <= 'z') *p = *p - 'a' + 'A';
    }
    
    if (strcmp(cmd, "STATUS") == 0) {
      print_status();
    } else if (strcmp(cmd, "DIAG") == 0) {
      print_diagnostics();
    } else if (strcmp(cmd, "RESET") == 0) {
      sys.glitch_count = 0;
      sys.rev_limit_events = 0;
      sys.error_flags = 0;
      Serial.println(F("Counters reset"));
    } else if (strcmp(cmd, "SAFE") == 0) {
      sys.use_safe_curve = true;
      Serial.println(F("Using SAFE curve"));
    } else if (strcmp(cmd, "PERF") == 0) {
      sys.use_safe_curve = false;
      Serial.println(F("Using PERF curve"));
    } else {
      Serial.print(F("Unknown: ")); Serial.println(cmd);
    }
  }
  
  void handle_commands() {
    static char buffer[48];
    static uint8_t index = 0;
    
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r') continue;
      
      if (c == '\n') {
        buffer[index] = '\0';
        if (index > 0) process_command(buffer);
        index = 0;
      } else if (index < sizeof(buffer) - 1) {
        buffer[index++] = c;
      } else {
        index = 0;  // Buffer overflow
      }
    }
  }
}

// ============================================================================
// STATUS LED
// ============================================================================
namespace StatusLED {
  enum Mode { OFF, SLOW_BLINK, FAST_BLINK, SOLID };
  
  void update() {
    static uint32_t last_toggle = 0;
    static bool led_state = false;
    uint32_t now = millis();
    
    Mode mode;
    if (sys.error_flags != 0) {
      mode = FAST_BLINK;
    } else if (sys.engine_running) {
      mode = SOLID;
    } else {
      mode = SLOW_BLINK;
    }
    
    uint16_t blink_period = (mode == FAST_BLINK) ? 100 : 500;
    
    switch (mode) {
      case SOLID:
        digitalWrite(Pins::STATUS_LED, HIGH);
        break;
      case OFF:
        digitalWrite(Pins::STATUS_LED, LOW);
        break;
      default:
        if (now - last_toggle > blink_period) {
          led_state = !led_state;
          digitalWrite(Pins::STATUS_LED, led_state);
          last_toggle = now;
        }
    }
  }
}

// ============================================================================
// MAIN SETUP
// ============================================================================


void setup() {
  // Initialize serial
  Serial.begin(115200);
  
  // Configure pins
  pinMode(Pins::TRIGGER_INPUT, INPUT_PULLUP);
  pinMode(Pins::STATUS_LED, OUTPUT);
  pinMode(Pins::SPARK_OUTPUT, OUTPUT);
  pinMode(Pins::OPTO_SHORT_RELAY, OUTPUT);
  
  // Safe initial state
  CoilControl::safe_idle();
  digitalWrite(Pins::STATUS_LED, LOW);
  
  // Initialize protection relay - KEEP PROTECTION ACTIVE AT STARTUP
  digitalWrite(Pins::OPTO_SHORT_RELAY, LOW);  // D4 LOW = Keep opto shorted (disabled)
  
  // Enable watchdog (2 second timeout)
  wdt_enable(WDTO_2S);
  
  // Initialize timer system
  TimerControl::initialize();
  
  // Startup message
  Serial.println(F("Rotax 787 Ignition Controller"));
  Serial.println(F("Commands: STATUS, DIAG, RESET, SAFE, PERF"));
  
  // LED startup sequence
  for (int i = 0; i < 3; i++) {
    digitalWrite(Pins::STATUS_LED, HIGH);
    delay(100);
    digitalWrite(Pins::STATUS_LED, LOW);
    delay(100);
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  wdt_reset();
  
  // SAFETY: Release protection relay only after stable triggers
  if (!sys.relays_initialized && sys.trigger_count >= 10 && sys.engine_running) {
    // We have stable triggers - safe to release protection
    digitalWrite(Pins::OPTO_SHORT_RELAY, HIGH); // D4 HIGH = Remove opto short (enable opto)
    sys.relays_initialized = true;
    Serial.println(F("Protection relay released - system armed"));
  }
  
  // Check for engine timeout
  EngineManager::check_timeout();
  
  // Process software timing events (for low RPM hybrid timing)
  SoftwareTiming::process_pending_events();
  
  // Handle serial commands
  SerialInterface::handle_commands();
  
  // Periodic status output
  if (millis() - sys.last_diagnostic_millis > 5000) {
    if (sys.engine_running) {
      SerialInterface::print_status();
    }
    sys.last_diagnostic_millis = millis();
  }
  
  // Update status LED
  StatusLED::update();
  
  // Process new trigger events
  if (sys.new_trigger_available) {
    sys.new_trigger_available = false;
    sys.last_trigger_millis = millis();
    EngineManager::calculate_and_schedule_spark();
  }
}

