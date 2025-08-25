/*
 * Arduino Ignition Timing Controller for Rotax 787
 * 
 * This controller manages ignition timing for a Rotax 787 twin-cylinder engine.
 * It reads magneto trigger pulses and generates precisely-timed spark events
 * based on engine RPM and a configurable advance curve.
 * 
 * Hardware Requirements:
 * - Arduino Uno/Nano (ATmega328P @ 16MHz)
 * - Pin 2 (INT0): Trigger input from magneto via optocoupler
 * - Pin 3: Spark output to 1GN-1A smart coil
 * - Pin 4: Relay control for startup protection
 * 
 * Timing System:
 * - Timer1 with prescaler 8 provides 0.5µs resolution
 * - Hardware compare matches for precise spark timing
 * - Software timing fallback for very low RPM (< 900)
 * 
 * Operating Modes:
 * - SAME LOBE: Spark fires in same 180° period as trigger (high RPM)
 * - PREVIOUS LOBE: Spark fires in next 180° period (low/mid RPM)
 * - Mode automatically selected based on available timing window
 */

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

// ============================================================================
// HARDWARE PIN ASSIGNMENTS
// ============================================================================
namespace Pins {
  const uint8_t TRIGGER_INPUT = 2;      // INT0: Magneto trigger (falling edge)
  const uint8_t SPARK_OUTPUT = 3;       // Coil control (HIGH=dwell, LOW=spark)
  const uint8_t OPTO_SHORT_RELAY = 4;   // Startup protection relay
}

// ============================================================================
// ENGINE CONFIGURATION - Rotax 787 Specific
// ============================================================================
namespace Engine {
  // Physical engine parameters
  const uint8_t PULSES_PER_REV = 2;        // Two flywheel lobes (180° apart)
  const float TRIGGER_ANGLE_BTDC = 47.0f;  // Magneto trigger at 47° BTDC
  
  // Operating limits
  const uint16_t MIN_RPM = 200;            // Below this = engine stopped
  const uint16_t REV_LIMIT = 7000;         // Soft rev limiter
  const uint16_t MAX_RPM = 8000;           // Absolute maximum
  const uint16_t REV_LIMIT_HYSTERESIS = 100;
  
  // Safety timeouts
  const uint16_t TIMEOUT_MS = 1000;        // No triggers = engine stopped
  const uint16_t STARTUP_DELAY_MS = 500;   // Relay release after stable triggers
}

// ============================================================================
// IGNITION COIL PARAMETERS - 1GN-1A Smart Coil
// ============================================================================
namespace Coil {
  const uint16_t DWELL_MS = 3;          // Fixed 3ms dwell time
  const bool ACTIVE_HIGH = true;        // HIGH starts dwell, LOW fires spark
}

// ============================================================================
// IGNITION ADVANCE CURVES
// ============================================================================
namespace TimingCurves {
  // RPM breakpoints for timing curve
  const uint16_t rpm_points[] PROGMEM = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000};
  
  // Safe timing curve (conservative advance)
  const int8_t safe_advance[] PROGMEM = {0, 6, 12, 15, 15, 14, 13, 12};
  
  // Performance curve (aggressive advance) - currently same as safe
  const int8_t perf_advance[] PROGMEM = {0, 6, 12, 15, 15, 14, 13, 12};
  
  const uint8_t num_points = sizeof(rpm_points) / sizeof(rpm_points[0]);
}

// ============================================================================
// GLOBAL SYSTEM STATE
// ============================================================================
struct SystemState {
  // Engine status
  volatile bool engine_running = false;
  volatile uint16_t current_rpm = 0;
  uint16_t filtered_rpm = 0;
  bool rev_limit_active = false;
  bool use_safe_curve = true;  // Which advance curve to use
  
  // Trigger timing data
  volatile uint16_t last_trigger_capture = 0;    // Timer1 value at last trigger
  volatile uint32_t trigger_period_ticks = 0;    // Period between triggers
  volatile bool new_trigger_available = false;   // Flag for main loop
  
  // Timer1 overflow tracking (for periods > 32.768ms)
  volatile uint16_t timer1_overflows = 0;
  volatile uint32_t last_trigger_timestamp = 0;  // Full 32-bit timestamp
  
  // Software timing state (for low RPM when hardware timing would overflow)
  bool using_software_timing = false;
  uint32_t software_dwell_scheduled_us = 0;      // When to start dwell
  uint32_t software_dwell_actual_us = 0;         // When dwell actually started
  bool software_dwell_pending = false;           // Waiting to start dwell
  bool software_dwell_active = false;            // Dwell started, waiting for spark
  
  // Diagnostics and safety
  volatile uint32_t trigger_count = 0;
  volatile uint32_t spark_count = 0;
  volatile uint16_t glitch_count = 0;
  uint32_t last_trigger_millis = 0;
  uint32_t last_diagnostic_millis = 0;
  
  // Scheduling state
  uint32_t scheduled_dwell_ticks = 0;
  volatile bool last_used_previous_lobe = false;  // Which mode was actually used
  
  // Startup protection
  bool relays_initialized = false;
  
  // Error tracking
  uint8_t error_flags = 0;
  static const uint8_t ERROR_OVERSPEED = 0x01;
  static const uint8_t ERROR_NO_SIGNAL = 0x04;
  static const uint8_t ERROR_INVALID_RPM = 0x08;
};

SystemState sys;

// ============================================================================
// UTILITY FUNCTIONS - Timer conversions and calculations
// ============================================================================
namespace Utils {
  // Timer1 runs at 2MHz (0.5µs per tick)
  inline uint32_t us_to_ticks(uint32_t microseconds) {
    return microseconds * 2UL;
  }
  
  inline uint32_t ms_to_ticks(uint32_t milliseconds) {
    return milliseconds * 2000UL;
  }
  
  inline uint32_t ticks_to_us(uint32_t ticks) {
    return ticks / 2UL;
  }
  
  // Calculate RPM from trigger period
  uint16_t calculate_rpm(uint32_t period_ticks) {
    if (period_ticks == 0) return 0;
    
    uint32_t period_us = ticks_to_us(period_ticks);
    if (period_us > 300000UL) return 0;  // Below minimum RPM
    
    // RPM = 60,000,000 / (period_us * pulses_per_rev)
    uint32_t rpm = (60000000UL / period_us) / Engine::PULSES_PER_REV;
    return (rpm > 65535) ? 0 : (uint16_t)rpm;
  }
  
  // Low-pass filter for RPM to smooth out variations
  uint16_t filter_rpm(uint16_t new_rpm, uint16_t old_rpm) {
    if (old_rpm == 0) return new_rpm;
    
    int16_t diff = (int16_t)new_rpm - (int16_t)old_rpm;
    int16_t abs_diff = (diff < 0) ? -diff : diff;
    
    // Adaptive filtering based on change magnitude
    if (abs_diff > 200) {
      // Large change - minimal filtering (75% new)
      return (uint16_t)(old_rpm + (diff * 3 >> 2));
    } else if (abs_diff > 50) {
      // Medium change - moderate filtering (50% new)
      return (uint16_t)(old_rpm + (diff >> 1));
    } else {
      // Small change - heavy filtering (25% new)
      return (uint16_t)(old_rpm + (diff >> 2));
    }
  }
}

// ============================================================================
// TIMING CURVE INTERPOLATION
// ============================================================================
float get_advance_angle(uint16_t rpm) {
  // Select which curve to use (safe or performance)
  const int8_t* curve = sys.use_safe_curve ? 
    TimingCurves::safe_advance : TimingCurves::perf_advance;
  
  // Handle edge cases
  if (rpm <= pgm_read_word_near(&TimingCurves::rpm_points[0])) {
    return (float)pgm_read_byte_near(&curve[0]);
  }
  
  if (rpm >= pgm_read_word_near(&TimingCurves::rpm_points[TimingCurves::num_points - 1])) {
    return (float)pgm_read_byte_near(&curve[TimingCurves::num_points - 1]);
  }
  
  // Linear interpolation between curve points
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
  
  // Should never reach here
  return (float)pgm_read_byte_near(&curve[TimingCurves::num_points - 1]);
}

// ============================================================================
// COIL CONTROL - Direct port manipulation for speed
// ============================================================================
namespace CoilControl {
  // Pin 3 (PD3) direct control
  inline void set_output_high() { PORTD |= _BV(PD3); }
  inline void set_output_low() { PORTD &= ~_BV(PD3); }
  
  void start_dwell() {
    // Begin charging coil
    if (Coil::ACTIVE_HIGH) set_output_high(); 
    else set_output_low();
  }
  
  void fire_spark() {
    // End dwell, fire spark
    if (Coil::ACTIVE_HIGH) set_output_low(); 
    else set_output_high();
  }
  
  void safe_idle() {
    // Ensure coil is not charging
    if (Coil::ACTIVE_HIGH) set_output_low();
    else set_output_high();
  }
}

// ============================================================================
// SOFTWARE TIMING - Fallback for low RPM when Timer1 would overflow
// ============================================================================
namespace SoftwareTiming {
  void cancel_pending_events() {
    if (sys.software_dwell_active) {
      CoilControl::safe_idle();
    }
    sys.software_dwell_pending = false;
    sys.software_dwell_active = false;
    sys.using_software_timing = false;
  }
  
  void schedule_dwell_start(uint32_t dwell_delay_us) {
    // Schedule dwell to start after specified delay
    sys.software_dwell_scheduled_us = micros() + dwell_delay_us;
    sys.software_dwell_pending = true;
    sys.software_dwell_active = false;
    sys.using_software_timing = true;
  }
  
  void process_pending_events() {
    if (!sys.using_software_timing) return;
    
    uint32_t current_us = micros();
    
    // Stage 1: Wait for dwell start time
    if (sys.software_dwell_pending && !sys.software_dwell_active) {
      int32_t time_until_dwell = (int32_t)(sys.software_dwell_scheduled_us - current_us);
      if (time_until_dwell <= 0) {
        // Time to start dwell
        CoilControl::start_dwell();
        sys.software_dwell_pending = false;
        sys.software_dwell_active = true;
        sys.software_dwell_actual_us = current_us;  // Record actual start time
      }
    }
    
    // Stage 2: Wait for spark time (3ms after dwell started)
    if (sys.software_dwell_active) {
      int32_t time_since_dwell = (int32_t)(current_us - sys.software_dwell_actual_us);
      if (time_since_dwell >= (Coil::DWELL_MS * 1000)) {
        // Time to fire spark
        CoilControl::fire_spark();
        sys.spark_count++;
        cancel_pending_events();
      }
    }
  }
}

// ============================================================================
// TIMER1 OVERFLOW ISR - Tracks time beyond 16-bit limit
// ============================================================================
ISR(TIMER1_OVF_vect) {
  sys.timer1_overflows++;
}

// ============================================================================
// TRIGGER INPUT ISR - Processes magneto pulses
// ============================================================================
ISR(INT0_vect) {
  uint16_t capture_time = TCNT1;
  static bool first_trigger = true;
  
  // Build full 32-bit timestamp
  uint32_t current_timestamp = ((uint32_t)sys.timer1_overflows << 16) | capture_time;
  
  // Skip first trigger (no period to calculate)
  if (first_trigger) {
    first_trigger = false;
    sys.last_trigger_timestamp = current_timestamp;
    sys.last_trigger_capture = capture_time;
    return;
  }
  
  // Calculate period since last trigger
  uint32_t delta_ticks = current_timestamp - sys.last_trigger_timestamp;
  
  // Reject glitches (minimum 1ms between triggers = 30,000 RPM max)
  if (delta_ticks < 2000) {
    sys.glitch_count++;
    return;
  }
  
  // Reject too-long periods (> 300ms = below 200 RPM)
  if (delta_ticks > 600000) {
    sys.glitch_count++;
    return;
  }
  
  // Valid trigger - update state
  sys.last_trigger_capture = capture_time;
  sys.last_trigger_timestamp = current_timestamp;
  sys.trigger_period_ticks = delta_ticks;
  sys.new_trigger_available = true;
  sys.trigger_count++;
  sys.engine_running = true;
}

// ============================================================================
// TIMER1 COMPARE A ISR - Fires spark
// ============================================================================
ISR(TIMER1_COMPA_vect) {
  // Fire spark
  CoilControl::fire_spark();
  
  // Disable compare interrupts (job done)
  TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));
  TIFR1 = _BV(OCF1A) | _BV(OCF1B);  // Clear flags
  
  sys.spark_count++;
}

// ============================================================================
// TIMER1 COMPARE B ISR - Starts dwell
// ============================================================================
ISR(TIMER1_COMPB_vect) {
  // Start dwell
  CoilControl::start_dwell();
  
  // Schedule spark for 3ms later
  uint16_t spark_time = OCR1B + Utils::ms_to_ticks(Coil::DWELL_MS);
  OCR1A = spark_time;
  
  // Switch to Compare A only for spark
  TIFR1 = _BV(OCF1A);
  TIMSK1 = _BV(TOIE1) | _BV(OCIE1A);  // Enable Compare A, disable Compare B
}

// ============================================================================
// ENGINE MANAGEMENT
// ============================================================================
namespace EngineManager {
  void check_timeout() {
    // Check if engine has stopped (no triggers for 1 second)
    if (sys.engine_running && 
        (millis() - sys.last_trigger_millis) > Engine::TIMEOUT_MS) {
      
      // Engine stopped - safe shutdown
      sys.engine_running = false;
      CoilControl::safe_idle();
      sys.error_flags |= SystemState::ERROR_NO_SIGNAL;
      
      // Re-engage startup protection
      if (sys.relays_initialized) {
        digitalWrite(Pins::OPTO_SHORT_RELAY, LOW);  // Re-enable protection
        sys.relays_initialized = false;
        sys.trigger_count = 0;
        sys.timer1_overflows = 0;
        SoftwareTiming::cancel_pending_events();
      }
    } else {
      sys.error_flags &= ~SystemState::ERROR_NO_SIGNAL;
    }
  }
  
  bool check_rev_limit(uint16_t rpm) {
    // Implement rev limiter with hysteresis
    if (!sys.rev_limit_active && rpm > Engine::REV_LIMIT) {
      sys.rev_limit_active = true;
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
    // Don't schedule sparks until protection is released
    if (!sys.relays_initialized) return;
    
    // Need valid trigger data
    if (sys.trigger_period_ticks == 0) {
      sys.error_flags |= SystemState::ERROR_INVALID_RPM;
      return;
    }
    
    // Skip first few triggers for stability
    if (sys.trigger_count < 5) {
      sys.error_flags |= SystemState::ERROR_INVALID_RPM;
      return;
    }
    
    // Validate period is reasonable (200-10000 RPM range)
    uint32_t min_period = Utils::ms_to_ticks(3);    // 10000 RPM
    uint32_t max_period = Utils::ms_to_ticks(300);  // 200 RPM
    if (sys.trigger_period_ticks < min_period || sys.trigger_period_ticks > max_period) {
      sys.error_flags |= SystemState::ERROR_INVALID_RPM;
      return;
    }
    
    // Calculate RPM and check limits
    uint16_t rpm = Utils::calculate_rpm(sys.trigger_period_ticks);
    if (rpm < Engine::MIN_RPM || rpm > Engine::MAX_RPM) {
      sys.error_flags |= SystemState::ERROR_INVALID_RPM;
      return;
    }
    
    sys.error_flags &= ~SystemState::ERROR_INVALID_RPM;
    sys.current_rpm = rpm;
    sys.filtered_rpm = Utils::filter_rpm(rpm, sys.filtered_rpm);
    
    // Check rev limiter
    if (check_rev_limit(sys.filtered_rpm)) return;
    
    // Get advance angle from curve
    float advance = get_advance_angle(sys.filtered_rpm);
    
    // Calculate spark delay from trigger point
    // Spark delay = (Trigger angle - Advance angle) as fraction of 180°
    float delta_angle = Engine::TRIGGER_ANGLE_BTDC - advance;
    while (delta_angle < 0) delta_angle += 360.0f;
    while (delta_angle >= 360.0f) delta_angle -= 360.0f;
    
    uint32_t period = sys.trigger_period_ticks;
    uint32_t delay_ticks = (uint32_t)(period * (delta_angle / 180.0f));
    uint32_t dwell_ticks = Utils::ms_to_ticks(Coil::DWELL_MS);
    
    // Determine timing mode: can we fit dwell in current period?
    const uint16_t SAFETY_MARGIN_TICKS = 600;  // 0.3ms safety margin
    bool use_same_lobe = (delay_ticks >= (dwell_ticks + SAFETY_MARGIN_TICKS));
    
    if (!use_same_lobe) {
      // PREVIOUS-LOBE mode: delay spark to next period
      delay_ticks += period;
    }
    
    // Calculate absolute timing values
    uint32_t dwell_start_32 = sys.last_trigger_capture + delay_ticks - dwell_ticks;
    uint32_t spark_time_32 = dwell_start_32 + dwell_ticks;
    
    // Check if both events fit within Timer1's 16-bit range
    bool both_fit_in_timer1 = (dwell_start_32 <= 65535) && (spark_time_32 <= 65535);
    
    if (both_fit_in_timer1) {
      // Use hardware timing (precise)
      OCR1B = (uint16_t)dwell_start_32;
      TIFR1 = _BV(OCF1A) | _BV(OCF1B);      // Clear flags
      TIMSK1 = _BV(TOIE1) | _BV(OCIE1B);    // Enable Compare B for dwell
      sys.scheduled_dwell_ticks = dwell_ticks;
      sys.last_used_previous_lobe = !use_same_lobe;
    } else {
      // Use software timing (for very low RPM)
      TIFR1 = _BV(OCF1A) | _BV(OCF1B);      // Clear flags
      TIMSK1 = _BV(TOIE1);                  // Overflow only
      
      uint32_t dwell_delay_us = Utils::ticks_to_us(delay_ticks - dwell_ticks);
      SoftwareTiming::schedule_dwell_start(dwell_delay_us);
      sys.scheduled_dwell_ticks = dwell_ticks;
      sys.last_used_previous_lobe = !use_same_lobe;
    }
  }
}

// ============================================================================
// SERIAL DIAGNOSTICS
// ============================================================================
namespace SerialInterface {
  void print_status() {
    float advance = get_advance_angle(sys.filtered_rpm);
    float delta = Engine::TRIGGER_ANGLE_BTDC - advance;
    
    Serial.print(F("RPM: ")); Serial.print(sys.filtered_rpm);
    Serial.print(F(", Period: ")); Serial.print(Utils::ticks_to_us(sys.trigger_period_ticks));
    Serial.print(F("us, Advance: ")); Serial.print(advance);
    Serial.print(F("°, Delta: ")); Serial.print(delta);
    Serial.print(F("°, Dwell: ")); Serial.print(Utils::ticks_to_us(sys.scheduled_dwell_ticks));
    Serial.print(F("us, Engine: ")); Serial.print(sys.engine_running ? F("RUN") : F("STOP"));
    Serial.print(F(", RevLim: ")); Serial.print(sys.rev_limit_active ? F("ON") : F("OFF"));
    Serial.print(F(", Mode: ")); 
    Serial.print(sys.last_used_previous_lobe ? F("PREV") : F("SAME"));
    Serial.print(F(", Errors: 0x")); Serial.println(sys.error_flags, HEX);
  }
}

// ============================================================================
// TIMER1 INITIALIZATION
// ============================================================================
void initialize_timer() {
  cli();
  
  // Configure Timer1
  TCCR1A = 0;                    // Normal mode
  TCCR1B = 0;
  TCNT1 = 0;                      // Clear counter
  TCCR1B = _BV(CS11);            // Prescaler = 8 (2MHz, 0.5µs resolution)
  TIMSK1 = _BV(TOIE1);           // Enable overflow interrupt
  
  // Configure INT0 (Pin 2) for falling edge trigger
  EICRA = (1 << ISC01);          // Falling edge
  EIFR = (1 << INTF0);           // Clear pending
  EIMSK = (1 << INT0);           // Enable
  
  sei();
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Initialize serial for diagnostics
  Serial.begin(115200);
  Serial.println(F("Rotax 787 Ignition Controller"));
  
  // Configure I/O pins
  pinMode(Pins::TRIGGER_INPUT, INPUT_PULLUP);
  pinMode(Pins::SPARK_OUTPUT, OUTPUT);
  pinMode(Pins::OPTO_SHORT_RELAY, OUTPUT);
  
  // Safe initial state
  CoilControl::safe_idle();
  digitalWrite(Pins::OPTO_SHORT_RELAY, LOW);  // Keep protection active
  
  // Enable watchdog (2 second timeout)
  wdt_enable(WDTO_2S);
  
  // Initialize timer system
  initialize_timer();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  wdt_reset();  // Keep watchdog happy
  
  // Release startup protection after stable triggers
  if (!sys.relays_initialized && 
      sys.trigger_count >= 10 && 
      (millis() - sys.last_trigger_millis) < 100) {
    // Engine is running stably - release protection
    digitalWrite(Pins::OPTO_SHORT_RELAY, HIGH);  // Enable ignition
    sys.relays_initialized = true;
  }
  
  // Check for engine timeout
  EngineManager::check_timeout();
  
  // Process software timing events (low RPM)
  SoftwareTiming::process_pending_events();
  
  // Print diagnostics every 5 seconds
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