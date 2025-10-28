/*******************************************************************************
 * ROTAX 787 IGNITION CONTROLLER
 *
 * Description:
 *   Programmable ignition controller for the Rotax 787 2-stroke engine.
 *   Provides precise spark timing control based on engine RPM with configurable
 *   advance curves and safety features.
 *
 * Features:
 *   - Hardware timer-based precision timing (0.5μs resolution)
 *   - RPM-based advance curve with 201-point interpolation
 *   - Predictive timing using Holt's double exponential smoothing
 *   - Multiple timing modes (same-lobe, previous-lobe, immediate)
 *   - Engine state management (stopped, ready, cranking, running)
 *   - Safety features: rev limiter, duty cycle protection, relay control
 *   - Fixed-point arithmetic for deterministic ISR performance
 *
 * Hardware Requirements:
 *   - Arduino Uno/Nano (ATmega328P @ 16MHz)
 *   - Pin D2: Crank trigger input (falling edge, 2 pulses per revolution)
 *   - Pin D3: Ignition coil output (HIGH = fire spark, LOW = dwell)
 *   - Pin D4: Safety relay control (HIGH = armed, LOW = safe)
 *   - Pin D5: Starter signal input (HIGH = cranking, LOW = not cranking)
 *
 * Version: 2.1.0
 * Author:  Richard Jeans
 * Date:    January 2025
 * License: MIT
 *
 * SAFETY WARNING:
 *   This controller directly controls ignition timing. Incorrect operation
 *   could cause engine damage, fire, or injury. Use at your own risk.
 *   Always include appropriate safety mechanisms and kill switches.
 ******************************************************************************/

#include <avr/interrupt.h>

// =============================================================================
// COMPILE-TIME CONFIGURATION
// =============================================================================

// Uncomment to enable diagnostic logging (uses more memory)
//#define ENABLE_DIAGNOSTIC_LOGGING

// =============================================================================
// HARDWARE CONFIGURATION CONSTANTS
// =============================================================================

namespace Config {
  // Pin assignments
  constexpr uint8_t TRIGGER_PIN = 2;  // D2 - External interrupt 0
  constexpr uint8_t SPARK_PIN = 3;    // D3 - Timer1 output capable
  constexpr uint8_t RELAY_PIN = 4;    // D4 - Safety relay control
  constexpr uint8_t STARTER_PIN = 5;  // D5 - Starter signal input

  // Direct port manipulation for spark pin (D3 = PD3)
  #define SPARK_PORT PORTD
  #define SPARK_DDR  DDRD
  #define SPARK_BIT  PD3

  // Timer1 configuration
  constexpr uint16_t TIMER1_PRESCALER_BITS = _BV(CS11);  // Prescaler = 8
  constexpr uint32_t TIMER_TICKS_PER_SEC = F_CPU / 8UL;  // 2,000,000 ticks/sec
  constexpr float TIMER_RESOLUTION_US = 0.5f;            // 0.5 μs per tick

  // Engine configuration
  constexpr uint8_t PULSES_PER_REVOLUTION = 2;  // 2-stroke with dual pickups
  constexpr uint16_t TRIGGER_BTDC_TENTHS = 435; // Trigger at 43.5° BTDC

  // RPM thresholds
  constexpr uint16_t MIN_RPM = 200;              // Minimum detectable RPM
  constexpr uint16_t MAX_RPM = 8000;             // Maximum operational RPM
  constexpr uint16_t CRANK_MAX_RPM = 800;        // Maximum cranking RPM
  constexpr uint16_t IDLE_MIN_RPM = 1000;        // Minimum idle RPM
  constexpr uint16_t OVERREV_RPM = 7000;         // Rev limiter engagement
  constexpr uint16_t OVERREV_HYST_RPM = 250;     // Rev limiter hysteresis

  // Timing parameters (in tenths of milliseconds)
  constexpr uint16_t NOMINAL_DWELL_TS = 30;  // 3.0ms nominal dwell time
  constexpr uint16_t MIN_DWELL_TS = 29;      // 2.9ms minimum dwell
  constexpr uint16_t MAX_DWELL_TS = 31;      // 3.1ms maximum dwell
  constexpr uint16_t MAX_DUTY_CYCLE = 40;    // 40% maximum coil duty cycle

  // Timing parameters (converted to ticks)
  constexpr uint32_t NOMINAL_DWELL_TICKS = (uint16_t)(( (uint32_t)NOMINAL_DWELL_TS * (uint32_t)TIMER_TICKS_PER_SEC) / 10000UL);
  constexpr uint16_t MIN_DWELL_TICKS = (uint16_t)(( (uint32_t)MIN_DWELL_TS * (uint32_t)TIMER_TICKS_PER_SEC) / 10000UL);
  constexpr uint16_t MAX_DWELL_TICKS = (uint16_t)(( (uint32_t)MAX_DWELL_TS * (uint32_t)TIMER_TICKS_PER_SEC) / 10000UL);

  // Period limits (in ticks)
  constexpr uint32_t MIN_PERIOD_TICKS = (TIMER_TICKS_PER_SEC * 60UL) / (PULSES_PER_REVOLUTION * MAX_RPM);
  constexpr uint32_t MAX_PERIOD_TICKS = (TIMER_TICKS_PER_SEC * 60UL) / (PULSES_PER_REVOLUTION * MIN_RPM);

  // State machine timing
  constexpr uint32_t STALL_TIMEOUT_MS = 500;     // Time to detect stall
  constexpr uint32_t CRANK_DEBOUNCE_MS = 200;    // Cranking state debounce
  constexpr uint32_t RELAY_ARM_DELAY_MS = 1000;  // Safety relay arm delay

  // Advance angles
  constexpr uint16_t CRANKING_ADVANCE_TENTHS = 0;  // 0° advance while cranking

  // Holt's smoothing parameters (Q16 fixed-point: 1.0 = 65536)
  constexpr uint32_t Q16_ONE = 65536;
  constexpr uint32_t Q16_ALPHA = 45875;  // 0.7 * 65536
  constexpr uint32_t Q16_BETA = 26214;   // 0.4 * 65536
  constexpr uint8_t HOLT_INIT_SAMPLES = 2;

  // Diagnostic configuration
  constexpr uint32_t DIAGNOSTIC_INTERVAL_MS = 2000;
  #ifdef ENABLE_DIAGNOSTIC_LOGGING
    constexpr uint16_t TRIGGER_BUFFER_SIZE = 16;
  #endif
}

// =============================================================================
// HARDWARE ABSTRACTION LAYER
// =============================================================================

namespace Hardware {
  using namespace Config;

  // System state (non-volatile for main loop access)
  uint32_t startup_millis = 0;
  bool relay_armed = false;

  // --------------------------------------------------------------------------
  // Spark pin control (using direct port manipulation for speed)
  // --------------------------------------------------------------------------

  inline void spark_pin_high() {
    SPARK_PORT |= _BV(SPARK_BIT);
  }

  inline void spark_pin_low() {
    SPARK_PORT &= ~_BV(SPARK_BIT);
  }

  inline void set_spark_pin_output() {
    SPARK_DDR |= _BV(SPARK_BIT);
  }

  inline void set_spark_pin_safe() {
    spark_pin_high();  // HIGH = safe (no dwell)
  }

  // --------------------------------------------------------------------------
  // Timer1 control
  // --------------------------------------------------------------------------

  inline uint16_t get_timer_count() {
    return TCNT1;
  }

  inline void reset_timer_count() {
    TCNT1 = 0;
  }

  inline void clear_dwell_flag() {
    TIFR1 = _BV(OCF1A);
  }

  inline void clear_spark_flag() {
    TIFR1 = _BV(OCF1B);
  }

  inline void clear_all_timer_flags() {
    TIFR1 = _BV(TOV1) | _BV(OCF1A) | _BV(OCF1B);
  }

  inline void enable_dwell_interrupt() {
    TIMSK1 |= _BV(OCIE1A);
  }

  inline void enable_spark_interrupt() {
    TIMSK1 |= _BV(OCIE1B);
  }

  inline void disable_dwell_interrupt() {
    TIMSK1 &= ~_BV(OCIE1A);
  }

  inline void disable_spark_interrupt() {
    TIMSK1 &= ~_BV(OCIE1B);
  }

  inline void set_dwell_compare(uint16_t when) {
    OCR1A = when;
  }

  inline void set_spark_compare(uint16_t when) {
    OCR1B = when;
  }

  inline void setup_timer() {
    TCCR1A = 0;  // Normal mode
    TCCR1B = TIMER1_PRESCALER_BITS;
    reset_timer_count();
    clear_all_timer_flags();
  }

  // --------------------------------------------------------------------------
  // External interrupt setup
  // --------------------------------------------------------------------------

  inline void setup_trigger_interrupt() {
    pinMode(TRIGGER_PIN, INPUT_PULLUP);
    EICRA = (1 << ISC01);  // Falling edge on INT0
    EIFR = (1 << INTF0);   // Clear pending flag
    EIMSK = (1 << INT0);   // Enable INT0
  }

  // --------------------------------------------------------------------------
  // Relay control
  // --------------------------------------------------------------------------

  inline void setup_relay() {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);  // Start safe
  }

  inline void arm_relay() {
    digitalWrite(RELAY_PIN, HIGH);
  }

  inline void disarm_relay() {
    digitalWrite(RELAY_PIN, LOW);
  }

  // --------------------------------------------------------------------------
  // Starter input
  // --------------------------------------------------------------------------

  inline void setup_starter_input() {
    pinMode(STARTER_PIN, INPUT);
  }

  inline bool starter_active() {
   // return digitalRead(STARTER_PIN) == HIGH;
   return true;
  }

  // --------------------------------------------------------------------------
  // State detection
  // --------------------------------------------------------------------------

  inline bool is_dwelling() {
    return (SPARK_PORT & _BV(SPARK_BIT)) == 0;  // LOW = dwelling
  }

  inline bool is_dwell_scheduled() {
    return (TIMSK1 & _BV(OCIE1A)) != 0;
  }
}

// =============================================================================
// ENGINE STATE MANAGEMENT
// =============================================================================

namespace Engine {
  using namespace Config;

  // Engine state enumeration
  enum State {
    STOPPED = 0,
    READY = 1,
    CRANKING = 2,
    RUNNING = 3
  };

  // Timing mode enumeration
  enum TimingMode {
    TIMING_SAME_LOBE = 0,      // Fire in same 180° period
    TIMING_PREVIOUS_LOBE = 1,  // Fire in previous 180° period
    TIMING_IMMEDIATE = 2,       // Immediate dwell (emergency)
    TIMING_STARTUP = 3          // Initial state
  };

  // State variables (volatile for ISR access)
  volatile State engine_state = STOPPED;
  volatile TimingMode timing_mode = TIMING_STARTUP;
  volatile bool spark_enabled = false;
  volatile bool over_rev_active = false;

  // State timers
  volatile uint32_t stall_start_millis = 0;
  volatile uint32_t crank_timer = 0;
  volatile uint8_t n_ready_triggers = 0;

  // Forward declarations
  void set_engine_stopped();
  void set_engine_ready();
  void set_engine_cranking();
  void set_engine_running();
}

// =============================================================================
// TIMING CALCULATIONS AND PREDICTION
// =============================================================================

namespace Timing {
  using namespace Config;

  // --------------------------------------------------------------------------
  // Timing variables (volatile for ISR access)
  // --------------------------------------------------------------------------

  // Trigger measurements
  volatile uint32_t trigger_millis = 0;
  volatile uint16_t trigger_tcnt = 0;
  volatile uint32_t period_millis = 0;
  volatile uint32_t period_ticks = 0;

  // RPM and prediction
  volatile uint16_t rpm = 0;
  volatile uint16_t predicted_rpm = 0;
  volatile uint32_t predicted_period_ticks = 0;

  // Timing calculations
  volatile uint16_t dwell_ticks = 0;
  volatile uint32_t max_dwell_ticks = 0;
  volatile uint16_t advance_tenths_0 = 0;      // Current advance
  volatile uint16_t advance_tenths_1 = 0;      // Predicted advance
  volatile uint16_t delay_angle_tenths_0 = 0;  // Current delay angle
  volatile uint16_t delay_angle_tenths_1 = 0;  // Predicted delay angle
  volatile uint16_t spark_delay_ticks_0 = 0;   // Same lobe spark delay
  volatile uint16_t spark_delay_ticks_1 = 0;   // Previous lobe spark delay
  volatile uint16_t dwell_delay_ticks_0 = 0;   // Same lobe dwell delay
  volatile uint16_t dwell_delay_ticks_1 = 0;   // Previous lobe dwell delay

  // Timing tracking
  volatile uint16_t current_ticks_after_calculation = 0;
  volatile uint16_t dwell_actual_started_tcnt = 0;
  volatile uint16_t dwell_actual_ended_tcnt = 0;

  // Holt's method state (Q16 fixed-point)
  volatile int32_t holt_level_q16 = 0;
  volatile int32_t holt_trend_q16 = 0;
  volatile uint8_t holt_init_count = 0;

  // --------------------------------------------------------------------------
  // Timing advance curve (PROGMEM)
  // --------------------------------------------------------------------------

  // 201-point advance curve: {RPM, advance_tenths}
  const uint16_t timing_curve[][2] PROGMEM = {
    {0, 0}, {40, 2}, {80, 4}, {120, 6}, {160, 8}, {200, 10}, {240, 12}, {280, 15},
    {320, 17}, {360, 19}, {400, 22}, {440, 24}, {480, 26}, {520, 29}, {560, 31}, {600, 34},
    {640, 36}, {680, 39}, {720, 42}, {760, 44}, {800, 47}, {840, 49}, {880, 52}, {920, 55},
    {960, 57}, {1000, 60}, {1040, 63}, {1080, 65}, {1120, 68}, {1160, 71}, {1200, 73},
    {1240, 76}, {1280, 78}, {1320, 81}, {1360, 84}, {1400, 86}, {1440, 89}, {1480, 91},
    {1520, 94}, {1560, 96}, {1600, 98}, {1640, 101}, {1680, 103}, {1720, 105}, {1760, 108},
    {1800, 110}, {1840, 112}, {1880, 114}, {1920, 116}, {1960, 118}, {2000, 120}, {2040, 122},
    {2080, 124}, {2120, 125}, {2160, 127}, {2200, 129}, {2240, 130}, {2280, 132}, {2320, 133},
    {2360, 135}, {2400, 136}, {2440, 138}, {2480, 139}, {2520, 140}, {2560, 141}, {2600, 142},
    {2640, 143}, {2680, 144}, {2720, 145}, {2760, 146}, {2800, 147}, {2840, 148}, {2880, 148},
    {2920, 149}, {2960, 149}, {3000, 150}, {3040, 150}, {3080, 151}, {3120, 151}, {3160, 152},
    {3200, 152}, {3240, 152}, {3280, 152}, {3320, 152}, {3360, 153}, {3400, 153}, {3440, 153},
    {3480, 153}, {3520, 153}, {3560, 152}, {3600, 152}, {3640, 152}, {3680, 152}, {3720, 152},
    {3760, 152}, {3800, 151}, {3840, 151}, {3880, 151}, {3920, 151}, {3960, 150}, {4000, 150},
    {4040, 150}, {4080, 149}, {4120, 149}, {4160, 149}, {4200, 148}, {4240, 148}, {4280, 148},
    {4320, 147}, {4360, 147}, {4400, 146}, {4440, 146}, {4480, 146}, {4520, 145}, {4560, 145},
    {4600, 144}, {4640, 144}, {4680, 143}, {4720, 143}, {4760, 143}, {4800, 142}, {4840, 142},
    {4880, 141}, {4920, 141}, {4960, 140}, {5000, 140}, {5040, 140}, {5080, 139}, {5120, 139},
    {5160, 138}, {5200, 138}, {5240, 138}, {5280, 137}, {5320, 137}, {5360, 136}, {5400, 136},
    {5440, 136}, {5480, 135}, {5520, 135}, {5560, 134}, {5600, 134}, {5640, 134}, {5680, 133},
    {5720, 133}, {5760, 132}, {5800, 132}, {5840, 132}, {5880, 131}, {5920, 131}, {5960, 130},
    {6000, 130}, {6040, 130}, {6080, 129}, {6120, 129}, {6160, 128}, {6200, 128}, {6240, 128},
    {6280, 127}, {6320, 127}, {6360, 126}, {6400, 126}, {6440, 126}, {6480, 125}, {6520, 125},
    {6560, 124}, {6600, 124}, {6640, 124}, {6680, 123}, {6720, 123}, {6760, 122}, {6800, 122},
    {6840, 122}, {6880, 121}, {6920, 121}, {6960, 120}, {7000, 120}, {7040, 120}, {7080, 119},
    {7120, 119}, {7160, 118}, {7200, 118}, {7240, 118}, {7280, 117}, {7320, 117}, {7360, 116},
    {7400, 116}, {7440, 116}, {7480, 115}, {7520, 115}, {7560, 114}, {7600, 114}, {7640, 114},
    {7680, 113}, {7720, 113}, {7760, 112}, {7800, 112}, {7840, 112}, {7880, 111}, {7920, 111},
    {7960, 110}, {8000, 110}
  };

  constexpr uint8_t TIMING_CURVE_POINTS = 201;
  constexpr uint16_t MAX_TIMING_RPM = 8000;

  // --------------------------------------------------------------------------
  // Utility functions
  // --------------------------------------------------------------------------

  inline uint16_t ticks_to_us(uint16_t ticks) {
    return ticks / 2;  // 0.5μs per tick
  }

  inline uint32_t calculate_rpm_numerator() {
    return TIMER_TICKS_PER_SEC * 60UL / PULSES_PER_REVOLUTION;
  }

  // --------------------------------------------------------------------------
  // Holt's double exponential smoothing (fixed-point implementation)
  // --------------------------------------------------------------------------

  void holt_initialize(uint16_t new_value) {
    if (holt_init_count < HOLT_INIT_SAMPLES) {
      if (holt_init_count == 0) {
        holt_level_q16 = ((uint32_t)new_value) << 16;
      } else {
        int32_t new_value_q16 = ((uint32_t)new_value) << 16;
        holt_trend_q16 = new_value_q16 - holt_level_q16;
        holt_level_q16 = new_value_q16;
      }
      holt_init_count++;
    }
  }

  uint16_t holt_update_and_predict(uint16_t new_value) {
    if (holt_init_count < HOLT_INIT_SAMPLES) {
      holt_initialize(new_value);
      return new_value;
    }

    // Convert to Q16
    int32_t new_value_q16 = ((uint32_t)new_value) << 16;
    int32_t level_prev_q16 = holt_level_q16;

    // Update level
    int64_t level_part1 = ((int64_t)Q16_ALPHA * new_value_q16) >> 16;
    int64_t level_part2 = ((int64_t)(Q16_ONE - Q16_ALPHA) * (level_prev_q16 + holt_trend_q16)) >> 16;
    holt_level_q16 = level_part1 + level_part2;

    // Update trend
    int64_t trend_part1 = ((int64_t)Q16_BETA * (holt_level_q16 - level_prev_q16)) >> 16;
    int64_t trend_part2 = ((int64_t)(Q16_ONE - Q16_BETA) * holt_trend_q16) >> 16;
    holt_trend_q16 = trend_part1 + trend_part2;

    // Calculate forecast
    int32_t forecast_q16 = holt_level_q16 + holt_trend_q16;
    int32_t predicted = forecast_q16 >> 16;

    // Clamp to valid range
    if (predicted < 0) predicted = 0;
    if (predicted > 65535) predicted = 65535;

    return (uint16_t)predicted;
  }

  void reset_holt() {
    holt_level_q16 = 0;
    holt_trend_q16 = 0;
    holt_init_count = 0;
  }

  // --------------------------------------------------------------------------
  // RPM and period calculations
  // --------------------------------------------------------------------------

  void calculate_rpm() {
    const uint32_t rpm_numerator = calculate_rpm_numerator();

    if (!period_ticks) {
      rpm = 0;
    } else {
      rpm = (uint16_t)(rpm_numerator / period_ticks);
    }

    predicted_rpm = holt_update_and_predict(rpm);

    if (!predicted_rpm) {
      predicted_period_ticks = period_ticks;
      predicted_rpm = rpm;
    } else {
      predicted_period_ticks = rpm_numerator / predicted_rpm;
    }
  }

  void adjust_period_for_wraparound(uint32_t& period_ticks_candidate, uint32_t period_millis_candidate) {
    uint32_t expected_period_ticks = period_millis_candidate * (TIMER_TICKS_PER_SEC / 1000UL);
    uint32_t num_wraps = expected_period_ticks / 0x10000;
    period_ticks_candidate = period_ticks_candidate + num_wraps * 0x10000;

    int32_t error = (int32_t)period_ticks_candidate - (int32_t)expected_period_ticks;
    if (error < -20000 || error > 20000) {
      if (error > 0) {
        period_ticks_candidate -= 0x10000;
      } else {
        period_ticks_candidate += 0x10000;
      }
    }
  }

  // --------------------------------------------------------------------------
  // Advance angle calculation (linear interpolation)
  // --------------------------------------------------------------------------

  // Calculate timing advance angle from RPM using linear interpolation
  static inline uint16_t calculate_advance_angle_tenths(uint16_t therpm) {


    // Edge cases
    uint16_t rpm_min = pgm_read_word(&timing_curve[0][0]);
    uint16_t adv_min = pgm_read_word(&timing_curve[0][1]);
    if (therpm <= rpm_min) {
      return (uint16_t)(adv_min);
    }

    uint16_t rpm_max = pgm_read_word(&timing_curve[TIMING_CURVE_POINTS - 1][0]);
    uint16_t adv_max = pgm_read_word(&timing_curve[TIMING_CURVE_POINTS - 1][1]);
    if (therpm >= rpm_max) {
      return (uint16_t)(adv_max);
    }

    // Must avoid overflow
    uint16_t i = (uint32_t(therpm) * (TIMING_CURVE_POINTS - 1) + MAX_TIMING_RPM - 1) / MAX_TIMING_RPM;

    uint16_t rpm_low  = pgm_read_word(&timing_curve[i - 1][0]);
    uint16_t rpm_high = pgm_read_word(&timing_curve[i][0]);

    uint16_t adv_low_u  = pgm_read_word(&timing_curve[i - 1][1]);
    uint16_t adv_high_u = pgm_read_word(&timing_curve[i][1]);

    // Do math in signed to handle negative slope
    int16_t  adv_low  = (int16_t)adv_low_u;
    int16_t  adv_high = (int16_t)adv_high_u;
    int16_t  d_adv    = (int16_t)(adv_high - adv_low);

    int32_t  d_rpm    = (int32_t)(therpm - rpm_low);
    int32_t  den      = (int32_t)(rpm_high - rpm_low);

    // numerator = d_adv * d_rpm * 10   (tenths of a degree)
    int32_t  num = (int32_t)d_adv * d_rpm;

    // Round to nearest: add or subtract half-denominator based on sign
    int32_t  frac_tenths = (num >= 0) ? (num + den / 2) / den
                                      : (num - den / 2) / den;

    int32_t  result_tenths = (int32_t)adv_low + frac_tenths;
    if (result_tenths < 0) result_tenths = 0;                 // optional clamp
    if (result_tenths > 1000) result_tenths = 1000;           // optional clamp
    return (uint16_t)result_tenths;
  }

  // --------------------------------------------------------------------------
  // Timing calculations
  // --------------------------------------------------------------------------

  void calculate_spark_delay() {
    // Select advance angle based on engine state
    if (Engine::engine_state == Engine::CRANKING) {
      advance_tenths_0 = Config::CRANKING_ADVANCE_TENTHS;
      advance_tenths_1 = Config::CRANKING_ADVANCE_TENTHS;
    } else {
      advance_tenths_0 = calculate_advance_angle_tenths(rpm);
      advance_tenths_1 = calculate_advance_angle_tenths(predicted_rpm);
    }

    // Calculate delay angle from trigger to spark
    delay_angle_tenths_0 = Config::TRIGGER_BTDC_TENTHS - advance_tenths_0;
    delay_angle_tenths_1 = Config::TRIGGER_BTDC_TENTHS - advance_tenths_1;

    // Convert angle to ticks
    spark_delay_ticks_0 = ((uint32_t)delay_angle_tenths_0 * (uint32_t)period_ticks + 900UL) / 1800UL;
    spark_delay_ticks_1 = (((uint32_t)delay_angle_tenths_1 + 1800UL) * (uint32_t)predicted_period_ticks + 900UL) / 1800UL;
  }

  void calculate_dwell() {
    max_dwell_ticks = ((period_ticks + 50UL) / 100UL) * uint32_t(MAX_DUTY_CYCLE);
    dwell_ticks = min(NOMINAL_DWELL_TICKS, max_dwell_ticks);
  }

  void calculate_dwell_delay() {
    dwell_delay_ticks_0 = spark_delay_ticks_0 - dwell_ticks;
    dwell_delay_ticks_1 = spark_delay_ticks_1 - dwell_ticks;
  }

  // --------------------------------------------------------------------------
  // Complete timing reset
  // --------------------------------------------------------------------------

  void reset_all_timing() {
    trigger_millis = 0;
    trigger_tcnt = 0;
    period_millis = 0;
    period_ticks = 0;
    rpm = 0;
    predicted_rpm = 0;
    predicted_period_ticks = 0;
    dwell_ticks = 0;
    max_dwell_ticks = 0;
    advance_tenths_0 = 0;
    advance_tenths_1 = 0;
    delay_angle_tenths_0 = 0;
    delay_angle_tenths_1 = 0;
    spark_delay_ticks_0 = 0;
    spark_delay_ticks_1 = 0;
    dwell_delay_ticks_0 = 0;
    dwell_delay_ticks_1 = 0;
    current_ticks_after_calculation = 0;
    dwell_actual_started_tcnt = 0;
    dwell_actual_ended_tcnt = 0;
    reset_holt();
  }
}

// =============================================================================
// DIAGNOSTIC LOGGING
// =============================================================================

#ifdef ENABLE_DIAGNOSTIC_LOGGING
namespace Diagnostics {
  using namespace Config;

  struct TriggerEvent {
    uint32_t trigger_id;
    uint32_t trigger_millis;
    uint16_t trigger_ticks;
    bool dwelling_entry;
    bool dwell_scheduled_entry;
    bool dwelling_exit;
    bool dwell_scheduled_exit;
    bool spark_enabled;
    uint8_t timing_mode_entry;
    uint8_t timing_mode_exit;
    uint16_t rpm;
    uint32_t period_millis;
    uint32_t period_ticks;
    uint32_t predicted_period_ticks;
    uint16_t advance_tenths_0;
    uint16_t advance_tenths_1;
    uint16_t delay_angle_tenths_0;
    uint16_t delay_angle_tenths_1;
    uint16_t spark_delay_ticks_0;
    uint16_t spark_delay_ticks_1;
    uint16_t dwell_ticks;
    uint32_t max_dwell_ticks;
    uint16_t dwell_time_0;
    uint16_t dwell_time_1;
    uint16_t current_time;
    uint16_t ocr1a;
    uint16_t dwell_actual_started_tcnt;
    uint16_t dwell_actual_ended_tcnt;
  };

  volatile TriggerEvent trigger_buffer[TRIGGER_BUFFER_SIZE];
  volatile uint16_t trigger_head = 0;
  volatile uint16_t trigger_tail = 0;
  volatile bool trigger_buffer_full = false;
  volatile bool keep_rotating_buffer = false;
  volatile bool dump_buffer_on_full = true;
  volatile bool buffer_dumped = false;
  volatile uint32_t next_trigger_id = 0;
  volatile uint32_t capture_start_trigger = 46;
  volatile bool capture_active = false;
  volatile bool capture_complete = false;

  const char* bool_to_string(bool value) {
    return value ? "true" : "false";
  }

  const char* get_timing_mode_name(uint8_t mode) {
    switch (mode) {
      case Engine::TIMING_SAME_LOBE: return "SAME_LOBE";
      case Engine::TIMING_PREVIOUS_LOBE: return "PREVIOUS_LOBE";
      case Engine::TIMING_IMMEDIATE: return "IMMEDIATE";
      case Engine::TIMING_STARTUP: return "STARTUP";
      default: return "UNKNOWN";
    }
  }

    void dump_trigger_buffer() {
    if (buffer_dumped) return;

    Serial.println(F("{"));
    Serial.println(F("  \"trigger_events\": ["));

    buffer_dumped = true;

    uint16_t count = 0;
    uint16_t idx = trigger_head;

    if (trigger_buffer_full) {
      count = TRIGGER_BUFFER_SIZE;
    } else {
      count = (trigger_head >= trigger_tail) ?
              (trigger_head - trigger_tail) :
              (TRIGGER_BUFFER_SIZE - trigger_tail + trigger_head);
      idx -= count;
    }

    for (uint16_t i = 0; i < count; i++) {
      volatile const TriggerEvent& event = trigger_buffer[idx];

      Serial.println(F("    {"));
      Serial.print(F("      \"trigger_id\": ")); Serial.print(event.trigger_id); Serial.println(F(","));
      Serial.print(F("      \"millis\": ")); Serial.print(event.trigger_millis); Serial.println(F(","));
      Serial.print(F("      \"trigger_ticks\": ")); Serial.print(event.trigger_ticks); Serial.println(F(","));
      Serial.print(F("      \"dwelling_entry\": ")); Serial.print(bool_to_string(event.dwelling_entry)); Serial.println(F(","));
      Serial.print(F("      \"dwell_scheduled_entry\": ")); Serial.print(bool_to_string(event.dwell_scheduled_entry)); Serial.println(F(","));
      Serial.print(F("      \"dwelling_exit\": ")); Serial.print(bool_to_string(event.dwelling_exit)); Serial.println(F(","));
      Serial.print(F("      \"dwell_scheduled_exit\": ")); Serial.print(bool_to_string(event.dwell_scheduled_exit)); Serial.println(F(","));
      Serial.print(F("      \"spark_enabled\": ")); Serial.print(bool_to_string(event.spark_enabled)); Serial.println(F(","));
      Serial.print(F("      \"timing_mode_entry\": \"")); Serial.print(get_timing_mode_name(event.timing_mode_entry)); Serial.println(F("\","));
      Serial.print(F("      \"timing_mode_exit\": \"")); Serial.print(get_timing_mode_name(event.timing_mode_exit)); Serial.println(F("\","));
      Serial.print(F("      \"rpm\": ")); Serial.print(event.rpm); Serial.println(F(","));
      Serial.print(F("      \"period_millis\": ")); Serial.print(event.period_millis); Serial.println(F(","));
      Serial.print(F("      \"period_ticks\": ")); Serial.print(event.period_ticks); Serial.println(F(","));
      Serial.print(F("      \"predicted_period_ticks\": ")); Serial.print(event.predicted_period_ticks); Serial.println(F(","));
      Serial.print(F("      \"advance_tenths_0\": ")); Serial.print(event.advance_tenths_0); Serial.println(F(","));
      Serial.print(F("      \"advance_tenths_1\": ")); Serial.print(event.advance_tenths_1); Serial.println(F(","));
      Serial.print(F("      \"delay_angle_tenths_0\": ")); Serial.print(event.delay_angle_tenths_0); Serial.println(F(","));
      Serial.print(F("      \"delay_angle_tenths_1\": ")); Serial.print(event.delay_angle_tenths_1); Serial.println(F(","));
      Serial.print(F("      \"spark_delay_ticks_0\": ")); Serial.print(event.spark_delay_ticks_0); Serial.println(F(","));
      Serial.print(F("      \"spark_delay_ticks_1\": ")); Serial.print(event.spark_delay_ticks_1); Serial.println(F(","));
      Serial.print(F("      \"dwell_ticks\": ")); Serial.print(event.dwell_ticks); Serial.println(F(","));
      Serial.print(F("      \"max_dwell_ticks\": ")); Serial.print(event.max_dwell_ticks); Serial.println(F(","));
      Serial.print(F("      \"dwell_time_0\": ")); Serial.print(event.dwell_time_0); Serial.println(F(","));
      Serial.print(F("      \"dwell_time_1\": ")); Serial.print(event.dwell_time_1); Serial.println(F(","));
      Serial.print(F("      \"current_time\": ")); Serial.print(event.current_time); Serial.println(F(","));
      Serial.print(F("      \"ocr1a\": ")); Serial.print(event.ocr1a); Serial.println(F(","));
      Serial.print(F("      \"dwell_actual_started_tcnt\": ")); Serial.print(event.dwell_actual_started_tcnt); Serial.println(F(","));
      Serial.print(F("      \"dwell_actual_ended_tcnt\": ")); Serial.print(event.dwell_actual_ended_tcnt); Serial.println();
      Serial.print(F("    }"));

      if (i < count - 1) {
        Serial.println(F(","));
      } else {
        Serial.println();
      }

      idx = (idx + 1) % TRIGGER_BUFFER_SIZE;
    }

    Serial.println(F("  ],"));
    Serial.print(F("  \"total_triggers\": ")); Serial.print(count);
    Serial.println();
    Serial.println(F("}"));
  }


  void store_trigger_event(bool dwelling_entry, bool dwell_scheduled_entry,
                           uint16_t trigger_ticks, uint8_t timing_mode_entry) {
    uint32_t current_trigger_id = next_trigger_id;

    if ((current_trigger_id >= capture_start_trigger && !capture_complete && !buffer_dumped) || keep_rotating_buffer) {
      if (!capture_active) {
        capture_active = true;
        Serial.print(F("Trigger buffer capture started at trigger "));
        Serial.println(current_trigger_id);
      }

      trigger_buffer[trigger_head].trigger_id = current_trigger_id;
      trigger_buffer[trigger_head].trigger_millis = Timing::trigger_millis;
      trigger_buffer[trigger_head].trigger_ticks = trigger_ticks;
      trigger_buffer[trigger_head].dwelling_entry = dwelling_entry;
      trigger_buffer[trigger_head].dwell_scheduled_entry = dwell_scheduled_entry;
      trigger_buffer[trigger_head].dwelling_exit = Hardware::is_dwelling();
      trigger_buffer[trigger_head].dwell_scheduled_exit = Hardware::is_dwell_scheduled();
      trigger_buffer[trigger_head].spark_enabled = Engine::spark_enabled;
      trigger_buffer[trigger_head].timing_mode_entry = timing_mode_entry;
      trigger_buffer[trigger_head].timing_mode_exit = Engine::timing_mode;
      trigger_buffer[trigger_head].rpm = Timing::rpm;
      trigger_buffer[trigger_head].period_millis = Timing::period_millis;
      trigger_buffer[trigger_head].period_ticks = Timing::period_ticks;
      trigger_buffer[trigger_head].predicted_period_ticks = Timing::predicted_period_ticks;
      trigger_buffer[trigger_head].advance_tenths_0 = Timing::advance_tenths_0;
      trigger_buffer[trigger_head].advance_tenths_1 = Timing::advance_tenths_1;
      trigger_buffer[trigger_head].delay_angle_tenths_0 = Timing::delay_angle_tenths_0;
      trigger_buffer[trigger_head].delay_angle_tenths_1 = Timing::delay_angle_tenths_1;
      trigger_buffer[trigger_head].spark_delay_ticks_0 = Timing::spark_delay_ticks_0;
      trigger_buffer[trigger_head].spark_delay_ticks_1 = Timing::spark_delay_ticks_1;
      trigger_buffer[trigger_head].dwell_ticks = Timing::dwell_ticks;
      trigger_buffer[trigger_head].max_dwell_ticks = Timing::max_dwell_ticks;
      trigger_buffer[trigger_head].dwell_time_0 = Timing::trigger_tcnt + Timing::dwell_delay_ticks_0;
      trigger_buffer[trigger_head].dwell_time_1 = Timing::trigger_tcnt + Timing::dwell_delay_ticks_1;
      trigger_buffer[trigger_head].current_time = Timing::current_ticks_after_calculation;
      trigger_buffer[trigger_head].ocr1a = OCR1A;
      trigger_buffer[trigger_head].dwell_actual_started_tcnt = Timing::dwell_actual_started_tcnt;
      trigger_buffer[trigger_head].dwell_actual_ended_tcnt = Timing::dwell_actual_ended_tcnt;

      trigger_head = (trigger_head + 1) % TRIGGER_BUFFER_SIZE;

      if (trigger_head == trigger_tail) {
        trigger_buffer_full = true;
        trigger_tail = (trigger_tail + 1) % TRIGGER_BUFFER_SIZE;

        if (!keep_rotating_buffer) {
          capture_complete = true;
          capture_active = false;
          Serial.print(F("Trigger buffer capture complete (full) at trigger "));
          Serial.println(current_trigger_id);
        }

        if (dump_buffer_on_full) {
          Serial.print(F("Buffer full at trigger "));
          Serial.print(current_trigger_id);
          Serial.println(F(" - auto-dumping:"));
          dump_trigger_buffer();
          Serial.println(F("Auto-dump complete."));
        }
      }
    }
  }


  void reset_diagnostics() {
    trigger_head = 0;
    trigger_tail = 0;
    trigger_buffer_full = false;
    next_trigger_id = 0;
    capture_active = false;
    capture_complete = false;
    buffer_dumped = false;
  }
}
#else
// Stub implementations when diagnostics disabled
namespace Diagnostics {
  inline void store_trigger_event(bool, bool, uint16_t, uint8_t) {}
  inline void dump_trigger_buffer() {}
  inline void reset_diagnostics() {}
}
#endif

// =============================================================================
// ENGINE STATE TRANSITIONS
// =============================================================================

namespace Engine {
  void set_engine_stopped() {
    Hardware::disable_dwell_interrupt();
    Hardware::disable_spark_interrupt();
    Hardware::set_spark_pin_safe();

    engine_state = STOPPED;
    Engine::timing_mode = Engine::TIMING_STARTUP;
    spark_enabled = false;
    over_rev_active = false;
    stall_start_millis = 0;
    crank_timer = 0;
    n_ready_triggers = 0;

    Timing::reset_all_timing();
    Hardware::clear_all_timer_flags();

    #ifdef ENABLE_DIAGNOSTIC_LOGGING
    if (!Diagnostics::dump_buffer_on_full) {
      if (Diagnostics::capture_complete || Diagnostics::trigger_head != Diagnostics::trigger_tail) {
        Serial.println(F("Engine stopped - dumping trigger buffer"));
        Diagnostics::dump_trigger_buffer();
      }
    }
    Diagnostics::reset_diagnostics();
    #endif
  }

  void set_engine_ready() {
    engine_state = READY;
    stall_start_millis = 0;
    crank_timer = 0;
    n_ready_triggers = 0;
  }

  void set_engine_cranking() {
    spark_enabled = true;
    engine_state = CRANKING;
    stall_start_millis = 0;
    crank_timer = 0;
  }

  void set_engine_running() {
    spark_enabled = true;
    engine_state = RUNNING;
    stall_start_millis = 0;
    crank_timer = 0;
  }

  void update_engine_state() {
    uint16_t current_rpm = Timing::rpm;
    uint32_t current_millis = millis();
    bool starter_is_active = Hardware::starter_active();

    // Over-rev protection with hysteresis
    if (!over_rev_active) {
      if (current_rpm > Config::OVERREV_RPM) {
        over_rev_active = true;
        spark_enabled = false;
      }
    } else {
      if (current_rpm < (Config::OVERREV_RPM - Config::OVERREV_HYST_RPM)) {
        over_rev_active = false;
        spark_enabled = true;
      }
    }

    // State machine transitions
    switch (engine_state) {
      case RUNNING:
        if (current_rpm < Config::IDLE_MIN_RPM && !starter_is_active) {
          if (stall_start_millis != 0 && current_millis - stall_start_millis > Config::STALL_TIMEOUT_MS) {
            set_engine_stopped();
          } else if (stall_start_millis == 0) {
            stall_start_millis = current_millis;
          }
        } else if (current_rpm < Config::IDLE_MIN_RPM && starter_is_active) {
          set_engine_cranking();
        }
        break;

      case STOPPED:
        if ((n_ready_triggers < Config::HOLT_INIT_SAMPLES || current_rpm < Config::MIN_RPM) && starter_is_active) {
          n_ready_triggers++;
        } else if (n_ready_triggers >= Config::HOLT_INIT_SAMPLES && current_rpm >= Config::MIN_RPM && starter_is_active) {
          set_engine_ready();
        }
        break;

      case READY:
        if (starter_is_active && current_rpm <= Config::CRANK_MAX_RPM) {
          if (crank_timer != 0 && current_millis - crank_timer > Config::CRANK_DEBOUNCE_MS) {
            set_engine_cranking();
          } else if (crank_timer == 0) {
            crank_timer = current_millis;
          }
        } else if (current_rpm > Config::CRANK_MAX_RPM) {
          if (current_rpm >= Config::IDLE_MIN_RPM) {
            set_engine_running();
          }
        }
        break;

      case CRANKING:
        if (current_rpm >= Config::IDLE_MIN_RPM) {
          set_engine_running();
        } else if (!starter_is_active && current_rpm < Config::CRANK_MAX_RPM) {
          set_engine_stopped();
        }
        break;
    }
  }
}

// =============================================================================
// IGNITION CONTROL FUNCTIONS
// =============================================================================

// Fire spark (called from Timer1 Compare B ISR)
inline void fire_spark() {
  Hardware::spark_pin_high();
  Timing::dwell_actual_ended_tcnt = Hardware::get_timer_count();
}

// Start dwell (called from Timer1 Compare A ISR)
inline void start_dwell() {
  Hardware::spark_pin_low();

  // Schedule spark after dwell period
  uint16_t spark_time = Hardware::get_timer_count() + Timing::dwell_ticks;
  Hardware::clear_spark_flag();
  Hardware::set_spark_compare(spark_time);
  Hardware::enable_spark_interrupt();
}

// Schedule dwell start (called from trigger ISR)
void schedule_dwell() {
  using namespace Timing;
  using namespace Engine;

  // Calculate all timing values
  calculate_dwell();
  calculate_spark_delay();
  calculate_dwell_delay();

  // Handle already scheduled dwell
  if (Hardware::is_dwell_scheduled()) {
    Hardware::disable_dwell_interrupt();
    Engine::timing_mode = Engine::TIMING_IMMEDIATE;
  } else if (Hardware::is_dwelling()) {
    Hardware::set_spark_compare(spark_delay_ticks_0 + trigger_tcnt);
  }

  uint16_t when;
  current_ticks_after_calculation = Hardware::get_timer_count();

  // Select timing mode
  if (Engine::timing_mode == Engine::TIMING_STARTUP) {
    if (spark_delay_ticks_0 < dwell_ticks) {
      Engine::timing_mode = Engine::TIMING_PREVIOUS_LOBE;
      when = trigger_tcnt + dwell_delay_ticks_1;
    } else {
      Engine::timing_mode = Engine::TIMING_SAME_LOBE;
      when = trigger_tcnt + dwell_delay_ticks_0;
    }
  } else if (Engine::timing_mode == Engine::TIMING_SAME_LOBE) {
    when = trigger_tcnt + dwell_delay_ticks_0;
    uint16_t immediate_dwell_ticks = when - current_ticks_after_calculation + dwell_ticks;
    if (immediate_dwell_ticks < Config::MAX_DWELL_TICKS && immediate_dwell_ticks > Config::MIN_DWELL_TICKS) {
      Engine::timing_mode = Engine::TIMING_IMMEDIATE;
    }
  } else if (Engine::timing_mode == Engine::TIMING_PREVIOUS_LOBE) {
    when = trigger_tcnt + dwell_delay_ticks_0;
    uint16_t immediate_dwell_ticks = when - current_ticks_after_calculation + dwell_ticks;
    if (immediate_dwell_ticks < Config::MAX_DWELL_TICKS && immediate_dwell_ticks > Config::MIN_DWELL_TICKS) {
      Engine::timing_mode = Engine::TIMING_IMMEDIATE;
    } else {
      when = trigger_tcnt + dwell_delay_ticks_1;
    }
  }

  // Handle immediate dwell
  if (Engine::timing_mode == Engine::TIMING_IMMEDIATE && !Hardware::is_dwelling()) {
    when = trigger_tcnt + dwell_delay_ticks_0;
    Hardware::spark_pin_low();
    dwell_actual_started_tcnt = Hardware::get_timer_count();

    uint16_t original_spark_time = when + dwell_ticks;
    Hardware::clear_spark_flag();
    Hardware::set_spark_compare(original_spark_time);
    Hardware::enable_spark_interrupt();

    uint16_t immediate_dwell_ticks = when - current_ticks_after_calculation + dwell_ticks;
    if (immediate_dwell_ticks < Config::MIN_DWELL_TICKS) {
      when = trigger_tcnt + dwell_delay_ticks_1;
      Engine::timing_mode = Engine::TIMING_PREVIOUS_LOBE;
    } else if (immediate_dwell_ticks > Config::MAX_DWELL_TICKS) {
      when = trigger_tcnt + dwell_delay_ticks_0;
      Engine::timing_mode = Engine::TIMING_SAME_LOBE;
    }
  }

  // Schedule normal dwell
  if (Engine::timing_mode != Engine::TIMING_IMMEDIATE) {
    Hardware::clear_dwell_flag();
    Hardware::set_dwell_compare(when);
    Hardware::enable_dwell_interrupt();
  }
}

// =============================================================================
// INTERRUPT SERVICE ROUTINES
// =============================================================================

// External interrupt: crank trigger (INT0)
ISR(INT0_vect) {
  uint16_t tcnt_candidate = Hardware::get_timer_count();
  uint32_t millis_candidate = millis();

  // Capture entry state for diagnostics
  bool dwelling_entry = Hardware::is_dwelling();
  bool dwell_scheduled_entry = Hardware::is_dwell_scheduled();

  // Calculate period
  uint32_t period_ticks_candidate = (uint32_t)(tcnt_candidate - Timing::trigger_tcnt);
  uint32_t period_millis_candidate = millis_candidate - Timing::trigger_millis;

  // Adjust for Timer1 wraparound
  Timing::adjust_period_for_wraparound(period_ticks_candidate, period_millis_candidate);

  // Noise filter: reject triggers < 33% of previous period
  const uint32_t min_valid_ticks = max((Timing::period_ticks / 3), Config::MIN_PERIOD_TICKS);
  if (period_ticks_candidate < min_valid_ticks && Engine::spark_enabled) {
    return;
  }

  #ifdef ENABLE_DIAGNOSTIC_LOGGING
  Diagnostics::next_trigger_id++;
  #endif

  // Update timing measurements
  Timing::period_ticks = period_ticks_candidate;
  Timing::period_millis = period_millis_candidate;
  Timing::predicted_period_ticks = Timing::period_ticks;
  Timing::trigger_tcnt = tcnt_candidate;
  Timing::trigger_millis = millis_candidate;

  // Calculate RPM and update engine state
  Timing::calculate_rpm();
  Engine::update_engine_state();

  // Capture timing mode before scheduling
  uint8_t timing_mode_before = Engine::timing_mode;

  // Schedule ignition if enabled
  if (Engine::spark_enabled) {
    schedule_dwell();
  }

  // Store diagnostic event
  Diagnostics::store_trigger_event(dwelling_entry, dwell_scheduled_entry, tcnt_candidate, timing_mode_before);
}

// Timer1 Compare A interrupt: dwell start
ISR(TIMER1_COMPA_vect) {
  Timing::dwell_actual_started_tcnt = Hardware::get_timer_count();
  Hardware::disable_dwell_interrupt();
  start_dwell();
}

// Timer1 Compare B interrupt: spark fire
ISR(TIMER1_COMPB_vect) {
  Hardware::disable_spark_interrupt();
  fire_spark();
}

// =============================================================================
// MAIN PROGRAM
// =============================================================================

void setup() {
  Serial.begin(115200);

  // Print startup banner
  Serial.println();
  Serial.println(F("==========================================="));
  Serial.println(F("  ROTAX 787 IGNITION CONTROLLER v2.1"));
  Serial.println(F("  Arduino-based timing controller"));
  Serial.println(F("==========================================="));
  Serial.println();

  // System configuration
  Serial.println(F("[INIT] System Configuration:"));
  Serial.print(F("  Timer Resolution: "));
  Serial.print(Config::TIMER_RESOLUTION_US, 1);
  Serial.println(F(" μs/tick"));
  Serial.print(F("  Target Dwell: "));
  Serial.print(Config::NOMINAL_DWELL_TS / 10.0, 1);
  Serial.println(F(" ms"));
  Serial.print(F("  RPM Range: "));
  Serial.print(Config::MIN_RPM);
  Serial.print(F(" - "));
  Serial.print(Config::MAX_RPM);
  Serial.println(F(" RPM"));
  Serial.print(F("  Rev Limiter: "));
  Serial.print(Config::OVERREV_RPM);
  Serial.println(F(" RPM"));
  Serial.println();

  // Hardware initialization
  Serial.println(F("[INIT] Hardware Setup:"));

  Hardware::set_spark_pin_output();
  Hardware::set_spark_pin_safe();
  Serial.print(F("  Spark Output (D"));
  Serial.print(Config::SPARK_PIN);
  Serial.println(F("): SAFE (HIGH)"));

  Hardware::setup_relay();
  Serial.print(F("  Safety Relay (D"));
  Serial.print(Config::RELAY_PIN);
  Serial.println(F("): DISARMED (LOW)"));

  Hardware::setup_starter_input();
  Serial.print(F("  Starter Input (D"));
  Serial.print(Config::STARTER_PIN);
  Serial.println(F("): CONFIGURED"));

  Hardware::setup_trigger_interrupt();
  Serial.print(F("  Trigger Input (D"));
  Serial.print(Config::TRIGGER_PIN);
  Serial.println(F("): INT0 ENABLED"));

  Hardware::setup_timer();
  Serial.println(F("  Timer1: CONFIGURED (Prescaler /8)"));

  sei();
  Serial.println(F("  Interrupts: ENABLED"));
  Serial.println();

  // Engine state information
  Serial.println(F("[INIT] Engine State Machine:"));
  Serial.println(F("  Current State: STOPPED"));
  Serial.println(F("  Waiting for starter signal and triggers..."));
  Serial.println();

  // Safety warnings
  Serial.println(F("[SAFETY] Protection Systems Active:"));
  Serial.println(F("  ✓ Relay protection (1-second delay)"));
  Serial.println(F("  ✓ Rev limiter (7000 RPM)"));
  Serial.println(F("  ✓ Duty cycle protection (40% max)"));
  Serial.println(F("  ✓ Trigger timeout (500ms)"));
  Serial.println(F("  ✓ Startup protection (2-trigger minimum)"));
  Serial.println();

  Serial.println(F("[READY] Controller initialized"));
  Serial.println(F("==========================================="));
  Serial.println();
}

void loop() {
  uint32_t current_millis = millis();

  // Print diagnostics every 2 seconds
  static uint32_t last_diagnostic_millis = 0;
  if (current_millis - last_diagnostic_millis >= Config::DIAGNOSTIC_INTERVAL_MS) {
    last_diagnostic_millis = current_millis;

    const char* state_names[] = {"STOPPED", "READY", "CRANKING", "RUNNING"};
    const char* timing_mode_names[] = {"SAME_LOBE", "PREV_LOBE", "IMMEDIATE", "STARTUP"};

    Serial.print(F("State: "));
    Serial.print(state_names[Engine::engine_state]);
    Serial.print(F(" | RPM: "));
    Serial.print(Timing::rpm);
    Serial.print(F(" | Starter: "));
    Serial.print(Hardware::starter_active() ? F("ON") : F("OFF"));
    Serial.print(F(" | Spark: "));
    Serial.print(Engine::spark_enabled ? F("EN") : F("DIS"));

    if (Engine::spark_enabled) {
      Serial.print(F(" | Timing: "));
      Serial.print(timing_mode_names[Engine::timing_mode]);
      Serial.print(F(" | Adv: "));
      Serial.print(Timing::advance_tenths_0 / 10);
      Serial.print(F("."));
      Serial.print(Timing::advance_tenths_0 % 10);
      Serial.print(F("°"));

      if (Timing::period_ticks > 0) {
        Serial.print(F(" | Dwell: "));
        Serial.print(Timing::ticks_to_us(Timing::dwell_ticks) / 1000);
        Serial.print(F("."));
        Serial.print((Timing::ticks_to_us(Timing::dwell_ticks) % 1000) / 100);
        Serial.print(F("ms"));
      }
    }

    if (Engine::over_rev_active) {
      Serial.print(F(" | OVERREV!"));
    }

    Serial.println();
  }

  // Check for trigger timeout
  if (Timing::trigger_millis > 0) {
    uint32_t time_since_trigger = current_millis - Timing::trigger_millis;
    if (current_millis > Timing::trigger_millis &&
        time_since_trigger > Config::STALL_TIMEOUT_MS &&
        Engine::engine_state != Engine::STOPPED) {
      Serial.print(F("No trigger for "));
      Serial.print(time_since_trigger);
      Serial.println(F("ms - Setting engine to STOPPED"));
      Engine::set_engine_stopped();
    }
  }

  // Arm relay when trigger pin is HIGH (with startup delay)
  if (!Hardware::relay_armed && digitalRead(Config::TRIGGER_PIN)) {
    if (Hardware::startup_millis == 0) {
      Hardware::startup_millis = current_millis;
    } else if (current_millis - Hardware::startup_millis > Config::RELAY_ARM_DELAY_MS) {
      Hardware::arm_relay();
      Hardware::relay_armed = true;
      Serial.println(F("Relay armed and ready"));
    }
  }
}