/*
 * Arduino Ignition Timing Controller
 * For Rotax 787 Applications
 * 
 * Features:
 * - Sub-microsecond timing precision using Timer1 
 * - Smart coil (1GN-1A) support with dwell control
 * - Safety systems with rev limiting
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
  
  // Simple timing state
  bool last_was_previous_lobe = false;            // For diagnostics
  
  // Software timing for Timer1 wraparound cases
  bool using_software_timing = false;
  uint32_t software_dwell_scheduled_us = 0;  // When dwell is scheduled to start
  uint32_t software_dwell_actual_us = 0;     // When dwell actually started
  bool software_dwell_pending = false;       // Stage 1: waiting for dwell start time
  bool software_dwell_active = false;        // Stage 2: dwell started, spark pending
  
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
  volatile bool last_was_predictive = false;      // Track if last spark was predictive
  
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
      
      // Only schedule spark event with Compare A
      OCR1A = spark_time;
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
  void cancel_pending_events() {
    if (sys.software_dwell_active) {
      // Ensure coil is in safe idle state
      CoilControl::safe_idle();
    }
    sys.software_dwell_pending = false;
    sys.software_dwell_active = false;
    sys.using_software_timing = false;
  }
  
  void schedule_dwell_start(uint32_t dwell_delay_us) {
    uint32_t current_us = micros();
    
    // Calculate absolute dwell start time (handle micros() wraparound)
    sys.software_dwell_scheduled_us = current_us + dwell_delay_us;
    
    // Ensure clean state
    sys.software_dwell_pending = true;
    sys.software_dwell_active = false;
    sys.using_software_timing = true;
  }
  
  void process_pending_events() {
    if (!sys.using_software_timing) return;
    
    uint32_t current_us = micros();
    
    // Stage 1: Handle dwell start
    if (sys.software_dwell_pending && !sys.software_dwell_active) {
      // Check if dwell start time has been reached
      // Using signed comparison to handle wraparound correctly
      int32_t time_until_dwell = (int32_t)(sys.software_dwell_scheduled_us - current_us);
      bool dwell_time_reached = (time_until_dwell <= 0);
      
      if (dwell_time_reached) {
        // Stage 2: Start dwell and record ACTUAL start time
        CoilControl::start_dwell();
        sys.software_dwell_pending = false;
        sys.software_dwell_active = true;
        // Record when dwell ACTUALLY started
        sys.software_dwell_actual_us = current_us;
      }
    }
    
    // Stage 2: Handle spark event (always 3ms after ACTUAL dwell start)
    if (sys.software_dwell_active) {
      // Check if 3ms have passed since dwell ACTUALLY started
      int32_t time_since_dwell = (int32_t)(current_us - sys.software_dwell_actual_us);
      bool spark_time_reached = (time_since_dwell >= (Coil::DWELL_MS * 1000));
      
      if (spark_time_reached) {
        CoilControl::fire_spark();
        sys.spark_count++;
        
        // Clean up software timing state
        cancel_pending_events();
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
  
  // Clean up: disable all compare interrupts and clear flags
  TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));  // Disable both compare interrupts
  TIFR1 = _BV(OCF1A) | _BV(OCF1B);  // Clear both compare flags
  
  sys.spark_count++;
  // Spark fired successfully
}

ISR(TIMER1_COMPB_vect) {
  // Stage 2: Dwell start event - begin charging coil and schedule spark
  CoilControl::start_dwell();
  
  // Calculate spark time (we know it fits since both events were pre-validated)
  uint16_t spark_time = OCR1B + Utils::ms_to_ticks(Coil::DWELL_MS);
  
  // Setup Compare A for spark timing
  OCR1A = spark_time;
  TIFR1 = _BV(OCF1A);  // Clear Compare A flag
  TIMSK1 = _BV(TOIE1) | _BV(OCIE1A);  // Enable Compare A, disable Compare B
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
        SoftwareTiming::cancel_pending_events();  // Cancel any software timing
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
  
  
  bool will_need_previous_lobe(uint32_t period_ticks, float advance) {
    // Predict if the NEXT spark will need previous-lobe timing
    // This function looks ahead to determine if we should start dwell NOW
    
    float delta_angle = Engine::TRIGGER_ANGLE_BTDC - advance;
    while (delta_angle < 0) delta_angle += 360.0f;
    while (delta_angle >= 360.0f) delta_angle -= 360.0f;
    
    uint32_t delay_ticks = (uint32_t)(period_ticks * (delta_angle / 180.0f));
    uint32_t target_dwell_ticks = Utils::ms_to_ticks(Coil::DWELL_MS);
    const uint16_t SAFETY_MARGIN_TICKS = 600; // ~0.30 ms
    
    // Check if dwell would need to start before the safety margin
    // If delay_ticks < (dwell_ticks + margin), we need previous-lobe
    return (delay_ticks < (target_dwell_ticks + SAFETY_MARGIN_TICKS));
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
    
    // Calculate timing for this period
    float delta_angle = Engine::TRIGGER_ANGLE_BTDC - advance;
    while (delta_angle < 0) delta_angle += 360.0f;
    while (delta_angle >= 360.0f) delta_angle -= 360.0f;
    
    uint32_t period = sys.trigger_period_ticks;
    uint32_t delay_ticks = (uint32_t)(period * (delta_angle / 180.0f));
    uint32_t dwell_ticks = Utils::ms_to_ticks(Coil::DWELL_MS);  // Always 3ms
    
    // Simple decision: Can we fit 3ms dwell in this period?
    const uint16_t SAFETY_MARGIN_TICKS = 600; // ~0.3ms
    bool use_same_lobe = (delay_ticks >= (dwell_ticks + SAFETY_MARGIN_TICKS));
    
    if (!use_same_lobe) {
      // PREVIOUS-LOBE: Add one period to delay spark into next period
      delay_ticks += period;
    }
    
    // Calculate dwell start time (same logic for both modes)
    uint32_t dwell_start_32 = sys.last_trigger_capture + delay_ticks - dwell_ticks;
    
    // Check if BOTH dwell start AND spark will fit in current Timer1 cycle
    uint32_t spark_time_32 = dwell_start_32 + dwell_ticks;
    bool both_fit_in_timer1 = (dwell_start_32 <= 65535) && (spark_time_32 <= 65535);
    
    if (both_fit_in_timer1) {
      // Both events fit in Timer1 range - use hardware timing
      OCR1B = (uint16_t)dwell_start_32;
      // Clear ALL compare flags and disable ALL compare interrupts first
      TIFR1 = _BV(OCF1A) | _BV(OCF1B);
      TIMSK1 = _BV(TOIE1) | _BV(OCIE1B);  // Only Compare B enabled
      sys.scheduled_dwell_ticks = dwell_ticks;
      sys.last_used_previous_lobe = !use_same_lobe;
    } else {
      // Dwell start too far - use software timing (two-stage approach)
      // Clear ALL compare flags and disable ALL compare interrupts
      TIFR1 = _BV(OCF1A) | _BV(OCF1B);
      TIMSK1 = _BV(TOIE1);  // Only overflow interrupt
      
      uint32_t dwell_delay_us = Utils::ticks_to_us(delay_ticks - dwell_ticks);
      SoftwareTiming::schedule_dwell_start(dwell_delay_us);
      sys.scheduled_dwell_ticks = dwell_ticks;
      sys.last_used_previous_lobe = !use_same_lobe;
    }
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
    Serial.print(F(", Mode: ")); 
    Serial.print(sys.last_used_previous_lobe ? F("PREV") : F("SAME"));
    Serial.print(F(", OvfProt: ")); Serial.print(sys.overflow_protection_events);
    Serial.print(F(", Errors: 0x")); Serial.println(sys.error_flags, HEX);
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
  pinMode(Pins::SPARK_OUTPUT, OUTPUT);
  pinMode(Pins::OPTO_SHORT_RELAY, OUTPUT);
  
  // Safe initial state
  CoilControl::safe_idle();
  
  // Initialize protection relay - KEEP PROTECTION ACTIVE AT STARTUP
  digitalWrite(Pins::OPTO_SHORT_RELAY, LOW);  // D4 LOW = Keep opto shorted (disabled)
  
  // Enable watchdog (2 second timeout)
  wdt_enable(WDTO_2S);
  
  // Initialize timer system
  TimerControl::initialize();
  
  // Startup message
  Serial.println(F("Rotax 787 Ignition Controller"));
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
  }
  
  // Check for engine timeout
  EngineManager::check_timeout();
  
  // No complex state management needed with simplified approach
  
  // Process software timing events (for low RPM hybrid timing)
  SoftwareTiming::process_pending_events();
  
  // Periodic status output
  if (millis() - sys.last_diagnostic_millis > 5000) {
    if (sys.engine_running) {
      SerialInterface::print_status();
    }
    sys.last_diagnostic_millis = millis();
  }
  
  // Process new trigger events
  if (sys.new_trigger_available) {
    sys.new_trigger_available = false;
    sys.last_trigger_millis = millis();
    EngineManager::calculate_and_schedule_spark();
  }
}

