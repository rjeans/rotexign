/*
 * Advanced Arduino Ignition Timing Controller
 * For High-Performance Jet Ski Applications
 * 
 * Features:
 * - Sub-microsecond timing precision using Timer1 Input Capture
 * - One-revolution-ahead scheduling for high advance angles
 * - CDI and Inductive coil support
 * - Comprehensive safety systems
 * - Serial tuning interface
 * - Real-time diagnostics
 * 
 * Hardware Requirements:
 * - Arduino Uno/Nano (ATmega328P)
 * - Optocoupler for magneto pickup isolation
 * - Output drivers for CDI/Coil control
 * - Kill switch input
 * 
 * Pin Assignments:
 * - D2 (INT0): Magneto pickup input (via optocoupler, falling edge)
 * - D9 (OC1A): Primary spark output
 * - D10 (OC1B): Secondary output (pulse end/dwell start)
 * - D7: Kill switch input (active low)
 * - D13: Status LED
 * - A0: Optional analog input for real-time tuning
 */

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

// ========== CONFIGURATION ==========
const bool CDI_MODE = false;             // true=CDI, false=Inductive (smart coil per design notes)
const uint8_t PPR = 2;                   // Pulses per revolution (two lobes)                   // Pulses per revolution
const float THETA_TRIGGER = 47.0;        // Trigger occurs 47° BTDC (TDC = 47° after trigger)        // Pickup angle (degrees BTDC)
const uint16_t REV_LIMIT = 7000;         // RPM rev limiter per design notes        // RPM rev limiter
const uint16_t CRANK_RPM = 400;          // Cranking RPM threshold
const float CRANK_ADV = 3.0;             // Cranking advance (degrees BTDC)
const uint16_t CDI_PULSE_US = 200;       // CDI trigger pulse width
const uint16_t DWELL_MS = 3;             // Inductive dwell time
const uint16_t MAX_RPM = 8000;           // Maximum valid RPM          // Maximum valid RPM
const uint16_t MIN_RPM = 200;            // Minimum valid RPM
const uint16_t TIMEOUT_MS = 1000;        // Engine stop timeout

// Pin definitions
const uint8_t TRIGGER_INPUT_PIN = 2;     // INT0 external interrupt (falling edge)
const uint8_t KILL_SWITCH_PIN = 7;
const uint8_t STATUS_LED_PIN = 13;
const uint8_t SPARK_OUTPUT_PIN = 9;      // OC1A (PB1)
const uint8_t AUX_OUTPUT_PIN = 10;       // OC1B (PB2)

// Timing curve - RPM vs Advance (degrees BTDC)
const uint16_t bp_rpm[] PROGMEM = {0, 800, 1500, 3000, 5000, 7000, 9000, 11000, 12000};
const int8_t  bp_adv_safe[] PROGMEM = {0, 6, 12, 15, 15, 14, 13, 12, 10};
const int8_t  bp_adv_perf[] PROGMEM = {2, 5, 12, 20, 24, 22, 18, 14, 10};
bool use_safe_curve = true;
const uint8_t N_BP = sizeof(bp_rpm) / sizeof(bp_rpm[0]);

// ========== GLOBAL VARIABLES ==========
// Volatile variables shared with ISRs
volatile uint16_t last_capture = 0;
volatile uint32_t period_ticks = 0;
volatile uint16_t rpm_filtered = 0;
volatile bool new_period_flag = false;
volatile bool kill_switch_active = false;
volatile uint32_t last_pulse_time = 0;
volatile bool engine_running = false;
volatile uint8_t missed_pulses = 0;

// Diagnostic counters
volatile uint32_t pulse_count = 0;
volatile uint32_t spark_count = 0;
volatile uint16_t glitch_count = 0;
volatile uint16_t rev_limit_count = 0;

// Status and error flags
volatile uint8_t error_flags = 0;
#define ERROR_OVERSPEED    0x01
#define ERROR_KILL_SWITCH  0x02
#define ERROR_NO_SIGNAL    0x04
#define ERROR_INVALID_RPM  0x08

// Serial communication
bool serial_enabled = false;
uint32_t last_diagnostic_time = 0;

// Declare runtime_theta_trigger as a global variable
float runtime_theta_trigger = THETA_TRIGGER; // Initialize with the default value

// ========== UTILITY FUNCTIONS ==========
inline uint32_t us_to_ticks(uint32_t microseconds) {
  return microseconds * 2UL;  // 0.5µs per tick at prescaler 8
}

inline uint32_t ms_to_ticks(uint32_t milliseconds) {
  return milliseconds * 2000UL;
}

inline uint32_t ticks_to_us(uint32_t ticks) {
  return ticks / 2UL;
}

// RPM calculation from period
uint16_t calculate_rpm(uint32_t period_ticks_) {
  if (period_ticks_ == 0) return 0;
  
  // Convert ticks to microseconds, then to RPM
  uint32_t period_us = period_ticks_ / 2UL;
  if (period_us > 300000UL) return 0;  // < 200 RPM
  
  uint32_t rpm = (60000000UL / period_us) / PPR;
  return (rpm > 65535) ? 0 : (uint16_t)rpm;
}

// Linear interpolation for timing curve (PROGMEM)
float get_advance_for_rpm(uint16_t rpm) {
  const int8_t* curve = use_safe_curve ? bp_adv_safe : bp_adv_perf;
  uint16_t x0 = pgm_read_word_near(&bp_rpm[0]);
  int8_t y0 = pgm_read_byte_near(&curve[0]);
  if (rpm <= x0) return y0;
  for (uint8_t i = 1; i < N_BP; i++) {
    uint16_t xi = pgm_read_word_near(&bp_rpm[i]);
    if (rpm <= xi) {
      x0 = pgm_read_word_near(&bp_rpm[i-1]);
      uint16_t x1 = xi;
      y0 = pgm_read_byte_near(&curve[i-1]);
      int8_t y1 = pgm_read_byte_near(&curve[i]);
      return y0 + (y1 - y0) * ((float)(rpm - x0) / (x1 - x0));
    }
  }
  return pgm_read_byte_near(&curve[N_BP-1]);
}

// Exponential filter for RPM smoothing (alpha = 0.25) using integer math
uint16_t filter_rpm(uint16_t new_rpm, uint16_t old_rpm) {
  if (old_rpm == 0) return new_rpm;
  int16_t diff = (int16_t)new_rpm - (int16_t)old_rpm;
  return (uint16_t)(old_rpm + (diff >> 2));
}

// Fast pin control for D9 (PB1) and D10 (PB2)
inline void spark_pin_high() { PORTB |= _BV(PB1); }
inline void spark_pin_low()  { PORTB &= (uint8_t)~_BV(PB1); }
inline void aux_pin_high()   { PORTB |= _BV(PB2); }
inline void aux_pin_low()    { PORTB &= (uint8_t)~_BV(PB2); }

// ========== TIMER1 + INT0 SETUP ==========
void setup_timer1() {
  // Disable interrupts during setup
  cli();
  
  // Reset Timer1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  // Configure Timer1 for timing base and compare events only
  // Prescaler = 8 (0.5µs resolution at 16MHz)
  TCCR1B = _BV(CS11);
  
  // Enable Compare Match interrupts
  TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);

  // Configure External Interrupt INT0 (D2) on falling edge (negative pulse first)
  EICRA = (1 << ISC01);  // ISC01=1, ISC00=0 -> falling edge
  EIFR  = (1 << INTF0);  // clear pending
  EIMSK = (1 << INT0);   // enable INT0
  
  // Configure OC1A and OC1B pins as outputs
  pinMode(SPARK_OUTPUT_PIN, OUTPUT);
  pinMode(AUX_OUTPUT_PIN, OUTPUT);
  digitalWrite(SPARK_OUTPUT_PIN, LOW);
  digitalWrite(AUX_OUTPUT_PIN, LOW);
  
  sei();
}

// ========== EXTERNAL TRIGGER ISR ==========
ISR(INT0_vect) {
  uint16_t current_capture = TCNT1;
  uint32_t current_time = millis();
  
  // Calculate period with 16-bit overflow handling
  uint16_t raw_period = current_capture - last_capture;
  last_capture = current_capture;
  
  // Reject impossible periods (< 0.5ms)
  if (raw_period < us_to_ticks(500)) {
    glitch_count++;
    return;
  }

  // Add debounce logic for input pulses
  if (current_time - last_pulse_time < 2) { // Reject pulses within 2 ms
    glitch_count++;
    return;
  }
  
  // Update period (no PPR scaling here) and set flag for main loop processing
  period_ticks = (uint32_t)raw_period;
  new_period_flag = true;
  last_pulse_time = current_time;
  pulse_count++;
  missed_pulses = 0;
  engine_running = true;
  // Defer heavy computation and GPIO reads to main loop
}

// ========== COMPARE MATCH A ISR (Spark Event) ==========
ISR(TIMER1_COMPA_vect) {
  // Primary event: For CDI, start pulse (HIGH). For Inductive, end dwell (LOW = spark)
  if (CDI_MODE) {
    spark_pin_high();
  } else {
    spark_pin_low();
  }
  spark_count++;
}

// ========== COMPARE MATCH B ISR (Auxiliary Event) ==========
ISR(TIMER1_COMPB_vect) {
  if (CDI_MODE) {
    // End of CDI pulse
    spark_pin_low();
  } else {
    // Inductive: Dwell start (coil ON)
    spark_pin_high();
  }
}

// ========== WATCHDOG AND TIMEOUT HANDLING ==========
void check_engine_timeout() {
  uint32_t current_time = millis();
  
  if (engine_running && (current_time - last_pulse_time) > TIMEOUT_MS) {
    // Engine stopped - disable outputs and reset state
    engine_running = false;
    spark_pin_low();
    aux_pin_low();
    error_flags |= ERROR_NO_SIGNAL;
    missed_pulses++;
  } else {
    error_flags &= ~ERROR_NO_SIGNAL;
  }
}

// ========== SERIAL COMMUNICATION ==========
void handle_serial_commands() {
  if (!Serial.available()) return;
  
  String command = Serial.readStringUntil('\n');
  command.trim();
  
  if (command == F("STATUS")) {
    print_status();
  } else if (command == F("DIAG")) {
    print_diagnostics();
  } else if (command == F("RESET")) {
    // Reset error counters
    glitch_count = 0;
    rev_limit_count = 0;
    error_flags = 0;
    Serial.println(F("Counters reset"));
  } else if (command.startsWith(F("ADVANCE "))) {
    // Real-time advance adjustment (for tuning)
    float new_advance = command.substring(8).toFloat();
    runtime_theta_trigger = new_advance; // Update runtime variable
    Serial.print(F("Advance set to: "));
    Serial.println(new_advance);
  }
}

void print_status() {
  Serial.print(F("RPM: ")); Serial.print(rpm_filtered);
  Serial.print(F(", Period: ")); Serial.print(ticks_to_us(period_ticks));
  Serial.print(F("us, Advance: ")); Serial.print(get_advance_for_rpm(rpm_filtered));
  Serial.print(F("°, Engine: ")); Serial.print(engine_running ? F("ON") : F("OFF"));
  Serial.print(F(", Errors: 0x")); Serial.println(error_flags, HEX);
}

void print_diagnostics() {
  Serial.println(F("=== DIAGNOSTICS ==="));
  Serial.print(F("Pulses: ")); Serial.println(pulse_count);
  Serial.print(F("Sparks: ")); Serial.println(spark_count);
  Serial.print(F("Glitches: ")); Serial.println(glitch_count);
  Serial.print(F("Rev Limits: ")); Serial.println(rev_limit_count);
  Serial.print(F("Missed Pulses: ")); Serial.println(missed_pulses);
  Serial.print(F("Kill Switch: ")); Serial.println(kill_switch_active ? F("ACTIVE") : F("OK"));
  Serial.print(F("Uptime: ")); Serial.print(millis() / 1000); Serial.println(F("s"));
}

// ========== STATUS LED MANAGEMENT ==========
void update_status_led() {
  static uint32_t last_blink = 0;
  static bool led_state = false;
  uint32_t current_time = millis();
  
  if (error_flags != 0) {
    // Fast blink for errors
    if (current_time - last_blink > 100) {
      led_state = !led_state;
      digitalWrite(STATUS_LED_PIN, led_state);
      last_blink = current_time;
    }
  } else if (engine_running) {
    // Solid on when running normally
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else {
    // Slow blink when waiting for signal
    if (current_time - last_blink > 500) {
      led_state = !led_state;
      digitalWrite(STATUS_LED_PIN, led_state);
      last_blink = current_time;
    }
  }
}

// ========== REFACTORED CODE WITH COMMENTS ==========

// Helper function to debounce input signals
bool debounce_signal(uint32_t current_time, uint32_t& last_time, uint32_t debounce_threshold) {
    if (current_time - last_time < debounce_threshold) {
        return false; // Signal is within debounce threshold
    }
    last_time = current_time;
    return true;
}

// Helper function to handle rev limiter logic
bool is_rev_limited(uint16_t rpm, uint16_t limit, uint16_t hysteresis, bool& rev_limit_active) {
    if (rpm > limit + hysteresis) {
        rev_limit_active = true;
    } else if (rpm < limit - hysteresis) {
        rev_limit_active = false;
    }
    return rev_limit_active;
}

// Helper function to initialize relay logic during startup
void initialize_relay() {
    digitalWrite(3, HIGH);   // Ground the output to the coil
    digitalWrite(4, HIGH);   // Remove short circuit from optocoupler
}

// ========== UPDATED SETUP FUNCTION ==========
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    serial_enabled = true;

    // Configure pins
    pinMode(TRIGGER_INPUT_PIN, INPUT_PULLUP); // optocoupler pulls low on trigger
    pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    pinMode(SPARK_OUTPUT_PIN, OUTPUT);
    digitalWrite(SPARK_OUTPUT_PIN, LOW);
    pinMode(AUX_OUTPUT_PIN, OUTPUT);
    digitalWrite(AUX_OUTPUT_PIN, LOW);

    // Initialize relay logic
    initialize_relay();

    // Enable watchdog timer (2 second timeout)
    wdt_enable(WDTO_2S);

    // Initialize Timer1
    setup_timer1();

    // Startup message
    Serial.println(F("Arduino Ignition Controller v1.0"));
    Serial.println(String(F("Mode: ")) + String(CDI_MODE ? F("CDI") : F("Inductive")));
    Serial.println(String(F("Pickup Angle: ")) + String(THETA_TRIGGER) + String(F("° BTDC")));
    Serial.println(String(F("Rev Limit: ")) + String(REV_LIMIT) + String(F(" RPM")));
    Serial.println(F("Ready for operation..."));

    // Initial LED pattern
    for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
    }
}

// ========== UPDATED MAIN LOOP ==========
void loop() {
    // Reset watchdog
    wdt_reset();

    // Check for engine timeout
    check_engine_timeout();

    // Handle serial commands
    if (serial_enabled) {
        handle_serial_commands();

        // Periodic diagnostic output
        if (millis() - last_diagnostic_time > 5000) {
            if (engine_running) {
                print_status();
            }
            last_diagnostic_time = millis();
        }
    }

    // Update status LED
    update_status_led();

    // Process new period data and schedule events
    if (new_period_flag) {
        noInterrupts();
        new_period_flag = false;
        uint32_t pt = period_ticks;
        uint16_t last_cap = last_capture;
        interrupts();

        // Calculate RPM
        uint16_t current_rpm = calculate_rpm(pt);
        if (current_rpm < MIN_RPM || current_rpm > MAX_RPM) {
            error_flags |= ERROR_INVALID_RPM;
            return;
        } else {
            error_flags &= ~ERROR_INVALID_RPM;
        }

        // Filter RPM
        rpm_filtered = filter_rpm(current_rpm, rpm_filtered);

        // Kill switch check with debounce
        static uint32_t last_kill_check = 0;
        if (!debounce_signal(millis(), last_kill_check, 50)) {
            digitalWrite(SPARK_OUTPUT_PIN, LOW);
            digitalWrite(AUX_OUTPUT_PIN, LOW);
            return;
        }

        // Desired advance
        float theta_spark = (rpm_filtered < CRANK_RPM) ? CRANK_ADV : get_advance_for_rpm(rpm_filtered);

        // Add logic to handle transition to the previous lobe at high RPMs
        if (rpm_filtered > 7000) {
            theta_spark = get_advance_for_rpm(rpm_filtered - 500);
        }

        // Rev limiting: hard cut above limit per design notes
        static bool rev_limit_active = false;
        if (is_rev_limited(rpm_filtered, REV_LIMIT, 100, rev_limit_active)) {
            return; // Skip spark event
        }

        // Angle after pickup to fire
        float delta_theta = runtime_theta_trigger - theta_spark;
        if (delta_theta < 0) delta_theta += 360.0f; // next revolution

        // Delay in ticks (float math only in main loop)
        uint32_t pt360 = pt * PPR; // scale trigger interval to 360°
        uint32_t delay_ticks = (uint32_t)((float)pt360 * delta_theta / 360.0f);
        uint16_t spark_time = last_cap + (uint16_t)delay_ticks;

        if (CDI_MODE) {
            // Pulse HIGH then LOW on SPARK pin
            OCR1A = spark_time;
            OCR1B = spark_time + (uint16_t)us_to_ticks(CDI_PULSE_US);
        } else {
            // Inductive: dwell then spark (same SPARK pin)
            uint32_t dwell_ticks = ms_to_ticks(DWELL_MS);
            uint32_t max_dwell_ticks = (pt360 * 40UL) / 100UL;
            if (dwell_ticks > max_dwell_ticks) dwell_ticks = max_dwell_ticks;
            if (delay_ticks <= dwell_ticks) {
                delay_ticks += pt360; // add 360° cycle
                spark_time = last_cap + (uint16_t)delay_ticks;
            }
            OCR1B = spark_time - (uint16_t)dwell_ticks; // coil ON
            OCR1A = spark_time;                          // coil OFF (spark)
        }
    }
}
