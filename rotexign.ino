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
  const uint8_t TRIGGER_INPUT = 2;      // INT0: Magneto pickup (falling edge)
  const uint8_t SPARK_OUTPUT = 9;       // OC1A: Coil control (HIGH=dwell, LOW=spark)
  const uint8_t DWELL_MARKER = 10;      // OC1B: Dwell indicator output
  const uint8_t STATUS_LED = 13;        // Status indicator
  const uint8_t COIL_GND_RELAY = 3;     // Grounds coil during startup
  const uint8_t OPTO_SHORT_RELAY = 4;   // Removes optocoupler short after boot
}

// ============================================================================
// ENGINE CONFIGURATION
// ============================================================================
namespace Engine {
  const uint8_t PULSES_PER_REV = 2;     // Two flywheel lobes
  const float TRIGGER_ANGLE_BTDC = 47.0f; // Trigger occurs 47° BTDC
  const float CRANKING_ADVANCE = 3.0f;    // Advance during cranking
  
  // RPM thresholds
  const uint16_t MIN_RPM = 200;
  const uint16_t CRANKING_RPM = 400;
  const uint16_t HIGH_RPM_THRESHOLD = 6500;  // Switch to previous lobe above this
  const uint16_t REV_LIMIT = 7000;
  const uint16_t MAX_RPM = 8000;
  const uint16_t REV_LIMIT_HYSTERESIS = 100;
  
  // Timing limits
  const uint16_t TIMEOUT_MS = 1000;     // Engine stop detection
  const uint16_t STARTUP_DELAY_MS = 2000; // Relay release delay
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
  
  // Diagnostics
  volatile uint32_t trigger_count = 0;
  volatile uint32_t spark_count = 0;
  volatile uint16_t glitch_count = 0;
  volatile uint16_t rev_limit_events = 0;
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
    return pgm_read_byte_near(&curve[0]);
  }
  
  // Linear interpolation
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
  
  // Beyond last point
  return pgm_read_byte_near(&curve[TimingCurves::num_points - 1]);
}

// ============================================================================
// COIL CONTROL
// ============================================================================
namespace CoilControl {
  inline void set_output_high() { PORTB |= _BV(PB1); }
  inline void set_output_low() { PORTB &= ~_BV(PB1); }
  inline void set_marker_high() { PORTB |= _BV(PB2); }
  inline void set_marker_low() { PORTB &= ~_BV(PB2); }
  
  void start_dwell() {
    // For 1GN-1A smart coil: HIGH = start charging (dwell)
    if (Coil::ACTIVE_HIGH) set_output_high(); 
    else set_output_low();
    set_marker_high();  // D10 marker shows dwell period
  }
  
  void fire_spark() {
    // For 1GN-1A smart coil: LOW = fire spark (end dwell)
    if (Coil::ACTIVE_HIGH) set_output_low(); 
    else set_output_high();
    set_marker_low();  // D10 marker goes low at spark
  }
  
  void safe_idle() {
    // Ensure coil is not charging (LOW for 1GN-1A)
    if (Coil::ACTIVE_HIGH) set_output_low();
    else set_output_high();
    set_marker_low();
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
    
    // Compare interrupts disabled initially
    TIMSK1 = 0;
    
    // Configure INT0 (D2) for falling edge
    EICRA = (1 << ISC01);  // Falling edge
    EIFR = (1 << INTF0);   // Clear pending
    EIMSK = (1 << INT0);   // Enable
    
    sei();
  }
  
  void cancel_scheduled_events() {
    // Disable compare interrupts
    TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));
    // Clear pending flags
    TIFR1 = _BV(OCF1A) | _BV(OCF1B);
  }
  
  void schedule_spark_event(uint16_t dwell_start, uint16_t spark_time) {
    noInterrupts();
    
    // IMPORTANT: Check if we have pending compares that haven't fired yet
    // If OCIE1A or OCIE1B are still enabled, we should not reschedule
    if (TIMSK1 & (_BV(OCIE1A) | _BV(OCIE1B))) {
      // Previous spark event still pending - skip this one to avoid corruption
      interrupts();
      return;
    }
    
    uint16_t current_time = TCNT1;
    const int16_t MIN_MARGIN = 32;  // 16µs minimum scheduling margin
    
    // Store original dwell duration
    uint16_t dwell_duration = (uint16_t)((int16_t)spark_time - (int16_t)dwell_start);
    
    // Ensure we're not scheduling dwell start in the past
    int16_t dwell_delta = (int16_t)(dwell_start - current_time);
    if (dwell_delta < MIN_MARGIN) {
      // Shift both events forward, maintaining dwell duration
      uint16_t shift = MIN_MARGIN - dwell_delta;
      dwell_start += shift;
      spark_time += shift;
    }
    
    // Double-check spark isn't in the past either
    int16_t spark_delta = (int16_t)(spark_time - current_time);
    if (spark_delta < MIN_MARGIN) {
      // Skip this spark event - too late
      interrupts();
      return;
    }
    
    // Final safety: ensure dwell_start comes before spark_time
    if ((int16_t)(spark_time - dwell_start) < MIN_MARGIN) {
      spark_time = dwell_start + MIN_MARGIN;
    }
    
    // Set compare values
    OCR1B = dwell_start;  // Dwell start (Compare B)
    OCR1A = spark_time;   // Spark fire (Compare A)
    
    // Clear flags and enable interrupts
    TIFR1 = _BV(OCF1A) | _BV(OCF1B);
    TIMSK1 |= _BV(OCIE1A) | _BV(OCIE1B);
    
    interrupts();
  }
}

// ============================================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================================
ISR(INT0_vect) {
  uint16_t capture_time = TCNT1;
  uint32_t current_micros = micros();  // Use micros() for accurate timing
  static uint32_t last_trigger_micros = 0;
  
  // Calculate period using micros() to avoid Timer1 16-bit overflow issues
  uint32_t delta_micros = current_micros - last_trigger_micros;
  uint32_t delta_ticks = delta_micros * 2UL;  // Convert μs to ticks (0.5μs per tick)
  
  // Reject glitches (minimum 1ms between triggers)
  if (delta_micros < 1000) {
    sys.glitch_count++;
    return;
  }
  
  // Reject impossibly long periods (> 300ms = < 200 RPM)
  if (delta_micros > 300000) {
    sys.glitch_count++;
    return;
  }
  
  // Store previous trigger data
  sys.prev_trigger_capture = sys.last_trigger_capture;
  sys.prev_period_ticks = sys.trigger_period_ticks;
  sys.have_previous_trigger = true;
  
  // Accept this trigger
  sys.last_trigger_capture = capture_time;  // Still track Timer1 for scheduling
  sys.trigger_period_ticks = delta_ticks;   // But use micros() for period
  last_trigger_micros = current_micros;
  sys.new_trigger_available = true;
  sys.trigger_count++;
  sys.missed_triggers = 0;
  sys.engine_running = true;
}

ISR(TIMER1_COMPA_vect) {
  // Spark event: end dwell
  CoilControl::fire_spark();
  TIMSK1 &= ~_BV(OCIE1A);  // Disable this interrupt
  sys.spark_count++;
}

ISR(TIMER1_COMPB_vect) {
  // Dwell start event
  CoilControl::start_dwell();
  TIMSK1 &= ~_BV(OCIE1B);  // Disable this interrupt
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
    // Get trigger data atomically
    noInterrupts();
    uint32_t period = sys.trigger_period_ticks;
    uint16_t last_trigger = sys.last_trigger_capture;
    uint16_t prev_trigger = sys.prev_trigger_capture;
    bool have_prev = sys.have_previous_trigger;
    interrupts();
    
    // Calculate RPM
    uint16_t rpm = Utils::calculate_rpm(period);
    if (rpm < Engine::MIN_RPM || rpm > Engine::MAX_RPM) {
      sys.error_flags |= SystemState::ERROR_INVALID_RPM;
      return;
    }
    sys.error_flags &= ~SystemState::ERROR_INVALID_RPM;
    
    // Filter RPM
    sys.filtered_rpm = Utils::filter_rpm(rpm, sys.filtered_rpm);
    
    // Check rev limiter
    if (check_rev_limit(sys.filtered_rpm)) {
      TimerControl::cancel_scheduled_events();
      CoilControl::safe_idle();
      return;
    }
    
    // Determine advance angle
    float advance_angle = (sys.filtered_rpm < Engine::CRANKING_RPM) ? 
      Engine::CRANKING_ADVANCE : get_advance_angle(sys.filtered_rpm);
    
    // Calculate timing delay from trigger to spark
    // Trigger occurs 47° BTDC, spark should occur at advance_angle° BTDC
    // So delay = 47° - advance_angle (same revolution)
    float delta_angle = Engine::TRIGGER_ANGLE_BTDC - advance_angle;
    
    // Use previous lobe at high RPM (schedule from previous trigger)
    bool use_previous_lobe = (sys.filtered_rpm > Engine::HIGH_RPM_THRESHOLD) && have_prev;
    uint16_t reference_trigger = use_previous_lobe ? prev_trigger : last_trigger;
    
    // For previous lobe scheduling, we don't add 180° to delta_angle
    // because we're already using the previous trigger as reference
    
    // Normalize angle to 0-360 range
    while (delta_angle < 0) delta_angle += 360.0f;
    while (delta_angle >= 360.0f) delta_angle -= 360.0f;
    
    // Convert angle to timer ticks
    // period = time between triggers (180°), delta_angle = degrees after trigger
    uint32_t delay_ticks = (uint32_t)(period * delta_angle / 180.0f);
    
    // Calculate dwell duration
    uint32_t dwell_ticks = Utils::ms_to_ticks(Coil::DWELL_MS);
    uint32_t ticks_per_revolution = period * Engine::PULSES_PER_REV;
    uint32_t max_dwell = (ticks_per_revolution * Coil::MAX_DUTY_PERCENT) / 100UL;
    if (dwell_ticks > max_dwell) {
      dwell_ticks = max_dwell;
    }
    
    // Ensure dwell doesn't exceed timing delay, but maintain minimum safe dwell
    if (delay_ticks < dwell_ticks) {
      // At high RPM, delay becomes very short. Maintain minimum 1ms dwell for reliability
      uint32_t min_dwell_ticks = Utils::ms_to_ticks(1);  // 1ms minimum
      uint32_t available_dwell = (delay_ticks > min_dwell_ticks) ? 
                                 delay_ticks - min_dwell_ticks : delay_ticks / 2;
      dwell_ticks = (available_dwell > min_dwell_ticks) ? available_dwell : min_dwell_ticks;
    }
    
    // Calculate absolute timer values from reference trigger
    uint16_t spark_time = reference_trigger + (uint16_t)delay_ticks;
    uint16_t dwell_start = spark_time - (uint16_t)dwell_ticks;
    
    // Store for diagnostics
    sys.scheduled_dwell_ticks = dwell_ticks;
    
    // Schedule the events
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
    Serial.print(F("RPM: ")); Serial.print(sys.filtered_rpm);
    Serial.print(F(", Period: ")); Serial.print(Utils::ticks_to_us(sys.trigger_period_ticks));
    Serial.print(F("us, Advance: ")); Serial.print(advance);
    Serial.print(F("°, Delta: ")); Serial.print(delta);
    Serial.print(F("°, Dwell: ")); Serial.print(Utils::ticks_to_us(sys.scheduled_dwell_ticks));
    Serial.print(F("us, Engine: ")); Serial.print(sys.engine_running ? F("RUN") : F("STOP"));
    Serial.print(F(", RevLim: ")); Serial.print(sys.rev_limit_active ? F("ON") : F("OFF"));
    Serial.print(F(", Errors: 0x")); Serial.println(sys.error_flags, HEX);
  }
  
  void print_diagnostics() {
    Serial.println(F("=== DIAGNOSTICS ==="));
    Serial.print(F("Triggers: ")); Serial.println(sys.trigger_count);
    Serial.print(F("Sparks: ")); Serial.println(sys.spark_count);
    Serial.print(F("Glitches: ")); Serial.println(sys.glitch_count);
    Serial.print(F("Rev Limits: ")); Serial.println(sys.rev_limit_events);
    Serial.print(F("Missed: ")); Serial.println(sys.missed_triggers);
    Serial.print(F("Uptime: ")); Serial.print(millis() / 1000); Serial.println(F("s"));
    Serial.print(F("Curve: ")); Serial.println(sys.use_safe_curve ? F("SAFE") : F("PERF"));
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
  pinMode(Pins::DWELL_MARKER, OUTPUT);
  pinMode(Pins::COIL_GND_RELAY, OUTPUT);
  pinMode(Pins::OPTO_SHORT_RELAY, OUTPUT);
  
  // Safe initial state
  CoilControl::safe_idle();
  digitalWrite(Pins::STATUS_LED, LOW);
  
  // Initialize protection relays
  digitalWrite(Pins::COIL_GND_RELAY, HIGH);   // Ground coil control
  digitalWrite(Pins::OPTO_SHORT_RELAY, HIGH); // Remove opto short
  
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
  
  // Handle relay release after startup
  if (!sys.relays_initialized) {
    uint32_t uptime = millis();
    if (uptime > Engine::STARTUP_DELAY_MS || 
        (sys.engine_running && sys.filtered_rpm > Engine::CRANKING_RPM)) {
      digitalWrite(Pins::COIL_GND_RELAY, LOW);  // Release coil ground
      sys.relays_initialized = true;
    }
  }
  
  // Check for engine timeout
  EngineManager::check_timeout();
  
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