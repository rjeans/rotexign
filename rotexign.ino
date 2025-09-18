/*
  Rotax 787 Ignition Controller
  -----------------------------
  This Arduino sketch implements a programmable ignition controller for the Rotax 787 engine.
  It uses Timer1 and external interrupts to precisely control ignition timing and dwell based on engine RPM.
  The timing advance curve is stored in PROGMEM and interpolated for smooth operation.
  Output is provided on FIRE_PIN, and diagnostic information is printed periodically via Serial.
  Author: Richard Jeans
  Date: 31 Aug 2025
*/

#include <avr/interrupt.h>

// Enable diagnostic logging and ring buffer - comment out to save memory
//#define ENABLE_DIAGNOSTIC_LOGGING

// Direct port control for SPARK_PIN (digital pin 3 = PD3 on ATmega328P)
#define SPARK_PORT  PORTD
#define SPARK_DDR   DDRD
#define SPARK_BIT   PD3



namespace System {


  volatile uint32_t last_diagnostic_millis = 0;
  volatile uint32_t startup_millis = 0;
  volatile bool relay_armed = false;


constexpr uint8_t SPARK_PIN = 3;         // Output pin for spark pulse
constexpr uint8_t RELAY_PIN = 4;         // D4 - Relay control (HIGH = open/armed, LOW = closed/safe)
constexpr uint8_t STARTER_PIN = 5;       // D5 - Starter input (HIGH = starter active, LOW = starter inactive)  
constexpr uint8_t TRIGGER_PIN = 2;       // D2 - Crank trigger input

  // Set the prescaler bits for a prescaler of 8.
  // The CS10 bit is NOT set.
constexpr uint16_t PRESCALE_BITS = _BV(CS11); 

  // Low-level spark pin control
  static inline void spark_pin_high() {
    SPARK_PORT |= _BV(SPARK_BIT);
  }
  
  static inline void spark_pin_low() {
    SPARK_PORT &= ~_BV(SPARK_BIT);
  }
  

  
  static inline void set_spark_pin_output() {
    SPARK_DDR |= _BV(SPARK_BIT);
  }
  
  static inline void set_spark_pin_safe() {
    spark_pin_high();  // HIGH = safe state (no dwell)
  }
  
  // Timer1 interrupt flag clearing
  static inline void clear_dwell_flag() {
    TIFR1 = _BV(OCF1A);
  }
  
  static inline void clear_spark_flag() {
    TIFR1 = _BV(OCF1B);
  }
  
  static inline void clear_all_timer_flags() {
    TIFR1 = _BV(TOV1) | _BV(OCF1A) | _BV(OCF1B);
  }
  
  // Timer1 interrupt enable/disable
  static inline void enable_dwell_interrupt() {
    TIMSK1 |= _BV(OCIE1A);
  }
  
  static inline void enable_spark_interrupt() {
    TIMSK1 |= _BV(OCIE1B);
  }
  
  static inline void disable_dwell_interrupt() {
    TIMSK1 &= ~_BV(OCIE1A);
  }
  
  static inline void disable_spark_interrupt() {
    TIMSK1 &= ~_BV(OCIE1B);
  }
  
  // Timer1 compare register setters
  static inline void set_dwell_compare(uint16_t when) {

    OCR1A = when;
  }
  
  static inline void set_spark_compare(uint16_t when) {

    OCR1B = when;
  }

  static inline uint16_t get_spark_compare() {

    return OCR1B;
  }
  
  // Timer1 counter getter/setter
  static inline uint16_t get_timer_count() {
    return TCNT1;
  }
  
  static inline void reset_timer_count() {
    TCNT1 = 0;
  }
  
  // Timer1 setup
  static inline void setup_timer() {
    TCCR1A = 0;  // Normal mode
    TCCR1B = PRESCALE_BITS;
    reset_timer_count();
    clear_all_timer_flags();
  }


  
  // External interrupt (INT0) setup for trigger input
  static inline void setup_trigger_interrupt() {
    // Configure pin as input with pullup
    pinMode(TRIGGER_PIN, INPUT_PULLUP);
    
    // Configure INT0 for falling edge
    EICRA = (1 << ISC01);  // ISC01 = 1, ISC00 = 0: falling edge
    
    // Clear any pending interrupt flag
    EIFR = (1 << INTF0);
    
    // Enable INT0
    EIMSK = (1 << INT0);
  }
  
  // Relay control
  static inline void setup_relay() {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);  // Start with relay closed (safe state)
  }
  
  static inline void arm_relay() {
    digitalWrite(RELAY_PIN, HIGH);  // Open relay to arm coil
  }
  
  static inline void disarm_relay() {
    digitalWrite(RELAY_PIN, LOW);  // Close relay to ground coil (safe)
  }
  
  // Starter input setup  
  static inline void setup_starter_input() {
    pinMode(STARTER_PIN, INPUT);  // Configure D5 as input for starter signal (LOW = inactive, HIGH = active)
  }

}



namespace Timing {

    enum TimingState {
    SPARK_FIRED,     // Spark has fired, waiting for next trigger
    DWELL_SCHEDULED, // Dwell start scheduled via timer interrupt
    DWELLING         // Currently dwelling (coil charging), spark scheduled
  };

  enum TimingMode {
    TIMING_SAME_LOBE = 0,      // Using spark_delay_ticks_0 (same 180° period)
    TIMING_PREVIOUS_LOBE = 1,   // Using spark_delay_ticks_1 (previous 180° period)
    TIMING_IMMEDIATE = 2,       // Immediate dwell (no scheduling possible)
    TIMING_STARTUP = 3,         // Startup triggers (no ignition yet)
  };

  enum EngineState {
    READY = 0,
    RUNNING = 1,
    CRANKING = 2,
    STOPPED= 3
  };

  // Current state variables
  // Note:
  volatile EngineState engine_state = STOPPED;
  const uint16_t IDLE_MIN_RPM = 1000;
  const uint16_t CRANK_MAX_RPM = 800;
  const uint16_t REENTRY_RPM = 1100;
  const uint16_t OVERREV_RPM = 7500;
  const uint16_t OVERREV_HYST_RPM = 250;
  const uint32_t STALL_TIMEOUT_MS = 500; 
  volatile uint32_t crank_timer=0;
  volatile uint32_t stall_start_millis = 0; // Time when RPM dropped below CRANK_MAX_RPM
  const uint32_t CRANK_DEBOUNCE_MS = 200;
  // Inline function to read starter state from D4 pin (HIGH = starter active, LOW = starter inactive)
  static inline bool starter_active() {
    return digitalRead(System::STARTER_PIN) == HIGH;
  }
  volatile bool over_rev_active = false; // Track if over-rev condition is active



 

  // State is now determined from hardware conditions - no state variable needed
  volatile uint32_t trigger_millis = 0; // Time of last trigger in milliseconds
  volatile bool spark_enabled = false; // Set true to enable spark output
  volatile uint8_t n_ready_triggers = 0; // Number of initial ticks collected before stable
  volatile uint32_t period_ticks = 0; // Period in ticks between last two triggers
  volatile uint32_t period_millis = 0; // Period in milliseconds between last two triggers
  volatile uint32_t predicted_period_ticks = 0; // Predicted next period using Holt's Method
  volatile uint16_t predicted_rpm = 0; // Candidate period from most recent trigger
  volatile uint16_t trigger_tcnt = 0;
  volatile uint16_t dwell_ticks = 0; // Dwell duration in ticks
  volatile uint32_t max_dwell_ticks = 0; // Maximum dwell duration based on duty cycle
  
  // Holt's Method (Double Exponential Smoothing) variables
  // Using simple integer arithmetic with minimal scaling  
  volatile float holt_alpha = 0.7f;         
  volatile float holt_beta = 0.4f;           
  volatile float holt_trend = 0.0f;           
  volatile float holt_level = 0.0f;          
  volatile uint8_t holt_init_count = 0;      // Counter for initialization samples
 volatile uint16_t current_ticks_after_calculation = 0; 
  volatile uint16_t dwell_delay_ticks_0 = 0; // Timer count when dwell started
  volatile uint16_t dwell_delay_ticks_1 = 0; // Timer count when dwell started
  volatile uint16_t spark_delay_ticks_0 = 0; // Delay from trigger to spark
  volatile uint16_t spark_delay_ticks_1 = 0; // Delay from trigger to spark plus 180 degrees
  volatile uint16_t rpm = 0; // Current RPM
  volatile uint16_t advance_tenths_0 = 0; // Advance angle in tenths of degrees
  volatile uint16_t advance_tenths_1 = 0; // Advance angle in tenths of degrees
  volatile uint16_t delay_angle_tenths_0 = 0; // Delay angle in tenths of degrees
  volatile uint16_t delay_angle_tenths_1 = 0; // Delay angle in tenths of degrees
  volatile uint16_t dwell_actual_started_tcnt = 0; // Timer count when dwell actually started
  volatile uint16_t dwell_actual_ended_tcnt = 0;   // Timer count when dwell actually ended
  volatile uint8_t current_timing_mode = TIMING_STARTUP; // Track which timing mode is being used

#ifdef ENABLE_DIAGNOSTIC_LOGGING
  // Trigger ring buffer for debugging
  struct TriggerEvent {
    uint32_t trigger_id;
    uint32_t trigger_millis;
    uint16_t trigger_ticks;      // trigger_tcnt value
    bool dwelling_entry;
    bool dwell_scheduled_entry;
    bool dwelling_exit;
    bool dwell_scheduled_exit;
    bool spark_enabled;          // whether spark is enabled for this trigger
    uint8_t timing_mode_entry;   // timing mode at entry to schedule_dwell
    uint8_t timing_mode_exit;    // timing mode at exit from schedule_dwell
    uint16_t rpm;
    uint32_t period_millis;      // period in milliseconds
    uint32_t period_ticks;       // actual period measured
    uint32_t predicted_period_ticks; // predicted period (from Holt's method)
    uint16_t spark_delay_ticks_0; // delay from trigger to spark (same lobe)
    uint16_t spark_delay_ticks_1; // delay from trigger to spark (previous lobe)
    uint16_t dwell_ticks;        // dwell duration in ticks
    uint32_t max_dwell_ticks;    // max dwell allowed based on duty cycle
    uint16_t dwell_time_0;     // trigger_tcnt + dwell_delay_ticks_0
    uint16_t dwell_time_1;     // trigger_tcnt + dwell_delay_ticks_1
    uint16_t current_time;     // current_ticks_after_calculation
  };

  static constexpr uint16_t TRIGGER_BUFFER_SIZE = 16;
  volatile TriggerEvent trigger_buffer[TRIGGER_BUFFER_SIZE];
  volatile uint16_t trigger_head = 0;
  volatile uint16_t trigger_tail = 0;
  volatile bool trigger_buffer_full = false;
  volatile bool keep_rotating_buffer = false; // Default: keep adding when full
  volatile bool dump_buffer_on_full = true;  // Default: auto-dump when buffer is full
  volatile bool buffer_dumped = false;  // Track if buffer has been dumped
  volatile uint32_t next_trigger_id = 0;
  
  // Capture control
  volatile uint32_t capture_start_trigger = 8654;  // Start capturing after trigger N
  volatile bool capture_active = false;
  volatile bool capture_complete = false;
#endif

  // Inline state detection functions
  static inline bool is_dwelling() {
    // Dwelling when spark pin is LOW (coil charging)
    return (SPARK_PORT & _BV(SPARK_BIT)) == 0;
  }
  
  static inline bool is_dwell_scheduled() {
    // Dwell scheduled when COMPA interrupt is enabled
    return (TIMSK1 & _BV(OCIE1A)) != 0;
  }
  
  // is_spark_scheduled removed - redundant with is_dwelling
  
  // No get_current_state needed - use individual boolean functions directly

  // Forward declarations
#ifdef ENABLE_DIAGNOSTIC_LOGGING
  static void dump_trigger_buffer();
  static const char* get_timing_mode_name(uint8_t mode);
#endif






  // Calculate the number of timer ticks per second with a prescaler of 8.
  // 16,000,000 Hz / 8 = 2,000,000 ticks/sec
  constexpr uint32_t TIMER_TICKS_PER_SEC = (F_CPU / 8UL);
  

  const uint8_t  PULSES_PER_REVOLUTION = 2;  // Number of pulses per engine revolution
  // The time per tick is the reciprocal of the ticks per second.
  // 1 / 2,000,000 = 0.5 microseconds per tick

  constexpr uint32_t RPM_NUMERATOR_TICKS = TIMER_TICKS_PER_SEC * 60UL / PULSES_PER_REVOLUTION;

  const uint16_t MAX_DUTY_CYCLE = 40; 
  const uint16_t NOMINAL_DWELL_TS = 30;
  const uint16_t MAX_DWELL_TS = 31;
  const uint16_t MIN_DWELL_TS = 29;
  constexpr uint32_t NOMINAL_DWELL_TICKS = (uint16_t)(( (uint32_t)NOMINAL_DWELL_TS * (uint32_t)TIMER_TICKS_PER_SEC) / 10000UL);
  constexpr uint16_t MAX_DWELL_TICKS = (uint16_t)(( (uint32_t)MAX_DWELL_TS * (uint32_t)TIMER_TICKS_PER_SEC) / 10000UL);
  constexpr uint16_t MIN_DWELL_TICKS = (uint16_t)(( (uint32_t)MIN_DWELL_TS * (uint32_t)TIMER_TICKS_PER_SEC) / 10000UL);
  const uint16_t MAX_RPM = 8000;
  const uint32_t MIN_PERIOD_TICKS =  (uint32_t)(RPM_NUMERATOR_TICKS / (uint32_t(MAX_RPM)));
  const uint16_t MIN_RPM = 200;
  const uint32_t MAX_PERIOD_TICKS =  (uint32_t)(RPM_NUMERATOR_TICKS / (uint32_t(MIN_RPM)));
  const uint16_t CUTOFF_RPM = 7000;
  const uint32_t CUTOFF_PERIOD_TICKS =  (uint32_t)(RPM_NUMERATOR_TICKS / (uint32_t(CUTOFF_RPM)));
  const uint16_t TRIGGER_BTDC_TENTHS = 470;  // 47.0 degrees in tenths
  const uint16_t CRANKING_ADVANCE_TENTHS = 0;   // 0.0 degrees in tenths

  const uint8_t HOLT_INIT_SAMPLES = 2;  // Number of samples for initialization



  // Convert Timer1 ticks at prescaler  back to microseconds (F_CPU = 16 MHz)
  static inline uint16_t ticks_to_us(uint16_t ticks) {
    return ticks/2 ;
  }

  // Initialize Holt's Method with first few samples
  // Keep it simple: just use first sample for level, zero trend
  static inline void holt_initialize(uint16_t new_value) {
    if (holt_init_count < HOLT_INIT_SAMPLES) {
      if (holt_init_count == 0) {
        // First sample: set initial level, zero trend
        holt_level = (float)new_value;
      } else  {
        holt_trend=(float)(new_value - holt_level);
        holt_level = (float)new_value;
      } 
      holt_init_count++;
    }
  }


  static inline uint16_t holt_update_and_predict(uint16_t new_value) {
    if (holt_init_count < HOLT_INIT_SAMPLES) {
      holt_initialize(new_value);
      return new_value; // Not enough data yet
    } else {

    float level_prev = holt_level;
    holt_level = holt_alpha * (float)new_value + (1.0f - holt_alpha) * (level_prev + holt_trend);
    holt_trend = holt_beta * (holt_level - level_prev) + (1.0f - holt_beta) * holt_trend;
    float forecast = holt_level + holt_trend;



    uint16_t predicted_uint = (uint16_t)constrain(forecast, 0,65535); // Clamp to valid range

    return predicted_uint;
  }

  }



  static inline void calculate_rpm() {


    if (!period_ticks) {
      rpm = 0;
    } else {
      rpm = (uint16_t)(RPM_NUMERATOR_TICKS / (uint32_t(period_ticks)));
    }

    predicted_rpm=holt_update_and_predict(rpm);

    if (!predicted_rpm) {
      predicted_period_ticks = period_ticks;
      predicted_rpm = rpm;
    } else {
      predicted_period_ticks = (uint32_t)(RPM_NUMERATOR_TICKS / (uint32_t(predicted_rpm)));
    }
  }

 
   


  // Timing advance curve: {RPM, advance in tenths of degrees} (201 points, 40 RPM resolution)
  static const uint16_t timing_rpm_curve[][2] PROGMEM = {
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

  const uint8_t TIMING_RPM_POINTS = 201;
  const uint16_t MAX_TIMING_RPM = 8000;

  // Calculate timing advance angle from RPM using linear interpolation
  static inline uint16_t calculate_advance_angle_tenths(uint16_t therpm) {


    // Edge cases
    uint16_t rpm_min = pgm_read_word(&timing_rpm_curve[0][0]);
    uint16_t adv_min = pgm_read_word(&timing_rpm_curve[0][1]);
    if (therpm <= rpm_min) {
      return (uint16_t)(adv_min);
    }

    uint16_t rpm_max = pgm_read_word(&timing_rpm_curve[TIMING_RPM_POINTS - 1][0]);
    uint16_t adv_max = pgm_read_word(&timing_rpm_curve[TIMING_RPM_POINTS - 1][1]);
    if (therpm >= rpm_max) {
      return (uint16_t)(adv_max);
    }

    // Must avoid overflow
    uint16_t i = (uint32_t(therpm) * (TIMING_RPM_POINTS - 1) + MAX_TIMING_RPM - 1) / MAX_TIMING_RPM;

    uint16_t rpm_low  = pgm_read_word(&timing_rpm_curve[i - 1][0]);
    uint16_t rpm_high = pgm_read_word(&timing_rpm_curve[i][0]);

    uint16_t adv_low_u  = pgm_read_word(&timing_rpm_curve[i - 1][1]);
    uint16_t adv_high_u = pgm_read_word(&timing_rpm_curve[i][1]);

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

  // Calculate spark delay from trigger to spark fire
  static inline void calculate_spark_delay() {
    // Calculate RPM and advance angle
    if (engine_state == CRANKING) {
      advance_tenths_0 = CRANKING_ADVANCE_TENTHS;
      advance_tenths_1 = CRANKING_ADVANCE_TENTHS;
    } else {
      advance_tenths_0 = calculate_advance_angle_tenths(rpm);
      advance_tenths_1 = calculate_advance_angle_tenths(predicted_rpm);
    }


    // Calculate delay angle: 47° - advance angle (in tenths)
    delay_angle_tenths_0 = TRIGGER_BTDC_TENTHS - advance_tenths_0;
    delay_angle_tenths_1 = TRIGGER_BTDC_TENTHS - advance_tenths_1;

    // Calculate delay in ticks using predicted period for next cycle accuracy
    // Delay_ticks = (delay_angle / 180°) * predicted_period_ticks
    // Using tenths: delay_ticks = (delay_angle_tenths * predicted_period_ticks) / 1800
    // Add 900 (which is half of 1800) to round the result to the nearest integer
    spark_delay_ticks_0 = ((uint32_t)delay_angle_tenths_0 * (uint32_t)period_ticks + 900UL) / 1800UL;
    spark_delay_ticks_1 = (((uint32_t)delay_angle_tenths_1 + 1800UL) * (uint32_t)predicted_period_ticks + 900UL) / 1800UL;



  }

  // Calculate dwell delay from trigger to dwell start
  static inline void calculate_dwell_delay() {
    // Always force a period (previous lobe)
    dwell_delay_ticks_0 = spark_delay_ticks_0 -  dwell_ticks;
    dwell_delay_ticks_1 = spark_delay_ticks_1 -  dwell_ticks;
  }

  // Calculate dwell duration based on predicted period and duty cycle limits  
  // Using predicted period provides better dwell control during RPM changes
  // Max dwell ticks can be bigger than 0xFFFF so use 32-bit
  static inline void calculate_dwell() {
    max_dwell_ticks = ((period_ticks + 50UL) / 100UL) * uint32_t(MAX_DUTY_CYCLE); // 40% max duty cycle with rounding
    dwell_ticks = min(NOMINAL_DWELL_TICKS, max_dwell_ticks);

  
  }

  static inline void set_engine_running() {
    spark_enabled = true;
    if (engine_state != RUNNING) {
      engine_state = RUNNING;
      Serial.print(F("Engine running at RPM: "));
      Serial.println(rpm);
    }
    stall_start_millis = 0; // Reset stall timer
    crank_timer = 0; // Update crank timer
  }

  static inline void set_engine_ready() {
    if (engine_state != READY) {
      engine_state = READY;
      Serial.print(F("Engine ready at RPM: "));
      Serial.println(rpm);
    }
    stall_start_millis = 0; // Reset stall timer
    crank_timer = 0; // Update crank timer
    n_ready_triggers = 0; // Clear ready triggers count
  }

  static inline void set_engine_cranking() {
    spark_enabled = true;
    if (engine_state != CRANKING) {
      engine_state = CRANKING;
      Serial.print(F("Engine cranking at RPM: "));
      Serial.println(rpm);
    }
    stall_start_millis = 0; // Reset stall timer
    crank_timer = 0; // Update crank timer
  }

  // Reset all timing variables when engine stops
  static inline void set_engine_stopped() {
    // Disable all timer interrupts
    System::disable_dwell_interrupt();
    System::disable_spark_interrupt();
    
    // Set spark pin to safe state
    System::set_spark_pin_safe();
    
    // Reset all timing state variables
    // State is now determined by hardware conditions - no variable to reset
    engine_state = STOPPED;
    over_rev_active = false;
    stall_start_millis = 0;
    crank_timer = 0;
    spark_enabled = false;
    n_ready_triggers = 0;
    period_ticks = 0;
    predicted_period_ticks = 0;
    period_millis = 0;
    predicted_rpm = 0;
    trigger_tcnt = 0;
    dwell_ticks = 0;
    
    // Reset Holt's Method variables
    holt_trend = 0.0f;
    holt_level = 0.0f;
    holt_init_count = 0;
    
    // Reset timing calculation variables
    dwell_delay_ticks_0 = 0;
    dwell_delay_ticks_1 = 0;
    spark_delay_ticks_0 = 0;
    spark_delay_ticks_1 = 0;
    current_ticks_after_calculation = 0;
    rpm = 0;
    advance_tenths_0 = 0;
    advance_tenths_1 = 0;
    delay_angle_tenths_0 = 0;
    delay_angle_tenths_1 = 0;
    dwell_actual_started_tcnt = 0;
    dwell_actual_ended_tcnt = 0;
    current_timing_mode = TIMING_STARTUP;
    trigger_millis = 0;
    
    // Clear all timer flags
    System::clear_all_timer_flags();
    
#ifdef ENABLE_DIAGNOSTIC_LOGGING
    // Dump trigger buffer if capture was active or we have data
    if (!dump_buffer_on_full) {
      if (capture_complete || trigger_head != trigger_tail) {
        Serial.println(F("Engine stopped - dumping trigger buffer"));
        dump_trigger_buffer();
      } else{
        Serial.print(" Number of triggers: "); Serial.print(next_trigger_id - 1);
        Serial.println(F(" Engine stopped - no trigger data to dump"));
      }
  }
    
    // Reset trigger buffer
    trigger_head = 0;
    trigger_tail = 0;
    trigger_buffer_full = false;
    next_trigger_id = 0;
    capture_active = false;
    capture_complete = false;
#endif
    
    Serial.println(F("Engine stopped - all timing variables reset"));
  }

#ifdef ENABLE_DIAGNOSTIC_LOGGING
  // Store trigger event in ring buffer (with capture control)
  static inline void store_trigger_event(bool dwelling_entry, bool dwell_scheduled_entry, 
                                         uint16_t trigger_ticks, uint8_t timing_mode_entry) {
    uint32_t current_trigger_id = next_trigger_id;
    
    // Check if we should start capturing
    if ((current_trigger_id >= capture_start_trigger && !capture_complete && !buffer_dumped) || keep_rotating_buffer) {

      if (!capture_active) {
        capture_active = true;
        Serial.print(F("Trigger buffer capture started at trigger "));
        Serial.println(current_trigger_id);
      }
      
      // Store trigger event
      trigger_buffer[trigger_head].trigger_id = current_trigger_id;
      trigger_buffer[trigger_head].trigger_millis = trigger_millis;
      trigger_buffer[trigger_head].trigger_ticks = trigger_ticks;
      trigger_buffer[trigger_head].dwelling_entry = dwelling_entry;
      trigger_buffer[trigger_head].dwell_scheduled_entry = dwell_scheduled_entry;
      trigger_buffer[trigger_head].dwelling_exit = is_dwelling();
      trigger_buffer[trigger_head].dwell_scheduled_exit = is_dwell_scheduled();
      trigger_buffer[trigger_head].spark_enabled = spark_enabled;
      trigger_buffer[trigger_head].timing_mode_entry = timing_mode_entry;
      trigger_buffer[trigger_head].timing_mode_exit = current_timing_mode;  // current mode after schedule_dwell
      trigger_buffer[trigger_head].rpm = rpm;
      trigger_buffer[trigger_head].period_millis = period_millis;
      trigger_buffer[trigger_head].period_ticks = period_ticks;
      trigger_buffer[trigger_head].predicted_period_ticks = predicted_period_ticks;
      trigger_buffer[trigger_head].spark_delay_ticks_0 = spark_delay_ticks_0;
      trigger_buffer[trigger_head].spark_delay_ticks_1 = spark_delay_ticks_1;
      trigger_buffer[trigger_head].dwell_ticks = dwell_ticks;
      trigger_buffer[trigger_head].max_dwell_ticks = max_dwell_ticks;
      trigger_buffer[trigger_head].dwell_time_0 = trigger_tcnt + dwell_delay_ticks_0;
      trigger_buffer[trigger_head].dwell_time_1 = trigger_tcnt + dwell_delay_ticks_1;
      trigger_buffer[trigger_head].current_time = current_ticks_after_calculation;
      
      // Advance head
      trigger_head = (trigger_head + 1) % TRIGGER_BUFFER_SIZE;
      
      // Check if buffer is full
      if (trigger_head == trigger_tail) {
        trigger_buffer_full = true;
        trigger_tail = (trigger_tail + 1) % TRIGGER_BUFFER_SIZE;
        if (!keep_rotating_buffer) {
          // Stop capturing if not rotating
          capture_complete = true;
          capture_active = false;
          Serial.print(F("Trigger buffer capture complete (full) at trigger "));
          Serial.println(current_trigger_id);
        }
      
        
        // Auto-dump if enabled and engine is running (spark enabled)
        if (dump_buffer_on_full) { {
          Serial.print(F("Buffer full at trigger "));
          Serial.print(current_trigger_id);
          Serial.println(F(" - auto-dumping:"));
          dump_trigger_buffer();
          Serial.println(F("Auto-dump complete."));
        } 
        
        // Buffer is full, stop capturing
        capture_complete = true;
        capture_active = false;
        Serial.print(F("Trigger buffer capture complete (full) at trigger "));
        Serial.println(current_trigger_id);
      }
    }
    }
  }
#else
  // Stub function when diagnostic logging is disabled
  static inline void store_trigger_event(bool, bool, uint16_t, uint8_t) {
    // Do nothing when logging disabled
  }
#endif

#ifdef ENABLE_DIAGNOSTIC_LOGGING
  // Helper function to print boolean as string
  static const char* bool_to_string(bool value) {
    return value ? "true" : "false";
  }
  
  // Get timing mode name from enum value
  static const char* get_timing_mode_name(uint8_t mode) {
    switch (mode) {
      case TIMING_SAME_LOBE: return "TIMING_SAME_LOBE";
      case TIMING_PREVIOUS_LOBE: return "TIMING_PREVIOUS_LOBE";
      case TIMING_IMMEDIATE: return "TIMING_IMMEDIATE";
      case TIMING_STARTUP: return "TIMING_STARTUP";
      default: return "UNKNOWN";
    }
  }

  // Dump trigger buffer in JSON format
  static void dump_trigger_buffer() {
    if (buffer_dumped) {
      return;
    }
    Serial.println(F("{"));
    Serial.println(F("  \"trigger_events\": ["));
    
    buffer_dumped = true;
    
    uint16_t count = 0;
    uint16_t idx = trigger_head;
    
    // Calculate number of entries
    if (trigger_buffer_full) {
      count = TRIGGER_BUFFER_SIZE;
    } else {
      count = (trigger_head >= trigger_tail) ? (trigger_head - trigger_tail) : 
              (TRIGGER_BUFFER_SIZE - trigger_tail + trigger_head);
      idx -= count;
    }
    
    for (uint16_t i = 0; i < count; i++) {
      volatile const TriggerEvent& event = trigger_buffer[idx];
      
      Serial.println(F("        {"));
      Serial.print(F("            \"trigger_id\": "));     Serial.print(event.trigger_id); Serial.println(F(","));
      Serial.print(F("            \"millis\": "));          Serial.print(event.trigger_millis); Serial.println(F(","));
      Serial.print(F("            \"trigger_ticks\": "));  Serial.print(event.trigger_ticks); Serial.println(F(","));
      Serial.print(F("            \"dwelling_entry\": "));  Serial.print(bool_to_string(event.dwelling_entry)); Serial.println(F(","));
      Serial.print(F("            \"dwell_scheduled_entry\": ")); Serial.print(bool_to_string(event.dwell_scheduled_entry)); Serial.println(F(","));
      Serial.print(F("            \"dwelling_exit\": "));   Serial.print(bool_to_string(event.dwelling_exit)); Serial.println(F(","));
      Serial.print(F("            \"dwell_scheduled_exit\": ")); Serial.print(bool_to_string(event.dwell_scheduled_exit)); Serial.println(F(","));
      Serial.print(F("            \"spark_enabled\": "));   Serial.print(bool_to_string(event.spark_enabled)); Serial.println(F(","));
      Serial.print(F("            \"timing_mode_entry\": \"")); Serial.print(get_timing_mode_name(event.timing_mode_entry)); Serial.println(F("\","));
      Serial.print(F("            \"timing_mode_exit\": \"")); Serial.print(get_timing_mode_name(event.timing_mode_exit)); Serial.println(F("\","));
      Serial.print(F("            \"rpm\": "));          Serial.print(event.rpm); Serial.println(F(","));
      Serial.print(F("            \"period_millis\": ")); Serial.print(event.period_millis); Serial.println(F(","));
      Serial.print(F("            \"period_ticks\": "));  Serial.print(event.period_ticks); Serial.println(F(","));
      Serial.print(F("            \"predicted_period_ticks\": ")); Serial.print(event.predicted_period_ticks); Serial.println(F(","));
      Serial.print(F("            \"spark_delay_ticks_0\": ")); Serial.print(event.spark_delay_ticks_0); Serial.println(F(","));
      Serial.print(F("            \"spark_delay_ticks_1\": ")); Serial.print(event.spark_delay_ticks_1); Serial.println(F(","));
      Serial.print(F("            \"dwell_ticks\": "));  Serial.print(event.dwell_ticks); Serial.println(F(","));
      Serial.print(F("            \"max_dwell_ticks\": "));  Serial.print(event.max_dwell_ticks); Serial.println(F(","));
      Serial.print(F("            \"dwell_time_0\": "));  Serial.print(event.dwell_time_0); Serial.println(F(","));
      Serial.print(F("            \"dwell_time_1\": "));  Serial.print(event.dwell_time_1); Serial.println(F(","));
      Serial.print(F("            \"current_time\": "));  Serial.print(event.current_time); Serial.println();
      Serial.print(F("        }"));
      
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
#endif





// Fire spark (called from COMPB ISR)
static inline void fire_spark() {
  // Fast pin set - HIGH to fire spark
  System::spark_pin_high();  // State becomes SPARK_FIRED when pin goes HIGH
  Timing::dwell_actual_ended_tcnt = System::get_timer_count();

}


// Schedule a COMPA one-shot at absolute tick 'when' (for dwell start)
static inline void schedule_dwell() {
  // Calculate all timing values
  calculate_dwell();
  calculate_spark_delay();
  calculate_dwell_delay();

  if (is_dwell_scheduled()) {
    // if the dwell is already scheduled then it has overshot - need to reschedule
    System::disable_dwell_interrupt();
    Timing::current_timing_mode = TIMING_IMMEDIATE; // Force immediate dwell
  } else if (is_dwelling()) {
      System::set_spark_compare(spark_delay_ticks_0 + trigger_tcnt);
  } 

  uint16_t when;
  current_ticks_after_calculation = System::get_timer_count();
  //lead_time_after_calculation = trigger_tcnt + dwell_delay_ticks_0 - current_ticks_after_calculation;  // Unsigned subtraction handles wraparound

  if (Timing::current_timing_mode == TIMING_STARTUP) {
     if (spark_delay_ticks_0 < dwell_ticks) {
       Timing::current_timing_mode = TIMING_PREVIOUS_LOBE;
       when = trigger_tcnt + dwell_delay_ticks_1;
     } else {
       Timing::current_timing_mode = TIMING_SAME_LOBE;
       when = trigger_tcnt + dwell_delay_ticks_0;
     }


  } else if (Timing::current_timing_mode == TIMING_SAME_LOBE) {
    when = trigger_tcnt + dwell_delay_ticks_0;
    uint16_t immediate_dwell_ticks = when - current_ticks_after_calculation + dwell_ticks;
     // Check if we are too close to the dwell time
    if (immediate_dwell_ticks < MAX_DWELL_TICKS && immediate_dwell_ticks > MIN_DWELL_TICKS) {
      // Too close or already past - switch to previous lobe timing
      Timing::current_timing_mode = TIMING_IMMEDIATE;

    }
  } else if (Timing::current_timing_mode == TIMING_PREVIOUS_LOBE) {
    when = trigger_tcnt + dwell_delay_ticks_0;
    uint16_t immediate_dwell_ticks = when - current_ticks_after_calculation + dwell_ticks;
     // Check if the next dwell time is likely to be a candidate for immediate dwell
    if (immediate_dwell_ticks < MAX_DWELL_TICKS && immediate_dwell_ticks > MIN_DWELL_TICKS) {
      // Too close or already past - switch to previous lobe timing
      Timing::current_timing_mode = TIMING_IMMEDIATE;

    } else {
      when = trigger_tcnt + dwell_delay_ticks_1;

    }
  } 
  



 if (Timing::current_timing_mode == TIMING_IMMEDIATE && !is_dwelling()) {
    when = trigger_tcnt + dwell_delay_ticks_0;

    // Too close or already past - start dwell immediately
    Timing::current_timing_mode = TIMING_IMMEDIATE;  // Mark as immediate dwell
    System::spark_pin_low();  // Start dwell immediately (coil charging)
    Timing::dwell_actual_started_tcnt = System::get_timer_count();
    // State is now DWELLING (spark pin is LOW)
    
    // Calculate the original planned spark time to maintain timing accuracy
    // Original spark time = when + dwell_ticks
    uint16_t original_spark_time = when + Timing::dwell_ticks;

    // Schedule spark timing at the original planned time (skip dwell interrupt)
    System::clear_spark_flag();
    System::set_spark_compare(original_spark_time);  // Maintain original spark timing
    System::enable_spark_interrupt();
    
    uint16_t immediate_dwell_ticks = when - current_ticks_after_calculation + dwell_ticks;
    if (immediate_dwell_ticks < MIN_DWELL_TICKS ) {
      // If we are already past the original spark time, fire spark immediately
      when = trigger_tcnt + dwell_delay_ticks_1;
      Timing::current_timing_mode = TIMING_PREVIOUS_LOBE;
    } else if (immediate_dwell_ticks > MAX_DWELL_TICKS) {
      // If we have enough time for a full dwell, schedule normally
      when = trigger_tcnt + dwell_delay_ticks_0;
      Timing::current_timing_mode = TIMING_SAME_LOBE;
    }

    
  }
  
  if (Timing::current_timing_mode != TIMING_IMMEDIATE) {
      // Safe to schedule normally
      System::clear_dwell_flag();   // clear stale flag
      System::set_dwell_compare(when);         // set absolute time
      System::enable_dwell_interrupt(); // enable dwell interrupt
      // State is now DWELL_SCHEDULED (COMPA interrupt enabled)
 }
}

// Schedule a COMPB one-shot for spark fire (dwell duration from now)
static inline void schedule_spark() {
  uint16_t spark_time = System::get_timer_count() + dwell_ticks;
  System::clear_spark_flag();
  System::set_spark_compare(spark_time);
  System::enable_spark_interrupt();
}

static inline void start_dwell() {
  // Fast pin clear - LOW to start dwell
  System::spark_pin_low();  // State becomes DWELLING when pin goes LOW

  // Schedule spark to fire after dwell period
  schedule_spark();
}

static inline void adjust_period_for_wraparound(uint32_t& period_ticks_candidate, uint32_t period_millis_candidate) {

  uint32_t expected_period_ticks = period_millis_candidate * (Timing::TIMER_TICKS_PER_SEC / 1000UL);
  // Adjust for possible Timer1 wraparound (every 65536 ticks)
  uint32_t num_wraps = (expected_period_ticks) / 0x10000; // Round to nearest)
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

  static inline void update_engine_state() {
    uint16_t current_rpm = Timing::rpm;
    uint32_t current_millis = millis();
    bool starter_is_active = starter_active();
   
/*
    // Print state diagnostics every 500ms
    static uint32_t last_state_print = 0;
    if (current_millis - last_state_print > 500) {
      last_state_print = current_millis;
      const char* state_names[] = {"READY", "RUNNING", "CRANKING", "STOPPED"};
      Serial.print(F("[STATE] Current: "));
      Serial.print(state_names[engine_state]);
      Serial.print(F(" RPM: "));
      Serial.print(current_rpm);
      Serial.print(F(" Starter: "));
      Serial.print(starter_is_active ? F("ON") : F("OFF"));
      Serial.print(F(" Spark: "));
      Serial.println(spark_enabled ? F("ENABLED") : F("DISABLED"));
    }
*/

    if (!over_rev_active) {
      if (current_rpm > OVERREV_RPM) {
        over_rev_active = true;
        Serial.println(F("Over-rev detected! Disabling spark."));
      }
    } else {
      // Currently in over-rev state
      if (current_rpm < (OVERREV_RPM - OVERREV_HYST_RPM)) {
        over_rev_active = false;
        Serial.println(F("RPM back to safe range. Re-enabling spark."));
      }
    }


    switch (engine_state) {
      case RUNNING:
        if (current_rpm < IDLE_MIN_RPM && !starter_is_active) {
          if (stall_start_millis !=0 && current_millis - stall_start_millis > STALL_TIMEOUT_MS) {
            Serial.print(F("[STATE] RUNNING->STOPPED: Stall timeout exceeded. RPM="));
            Serial.println(current_rpm);
            set_engine_stopped();
          } else if (stall_start_millis == 0) {
            stall_start_millis = current_millis;
            Serial.print(F("[STATE] RPM below idle - starting stall timer at RPM="));
            Serial.println(current_rpm);

          }
        } else if (current_rpm < IDLE_MIN_RPM && starter_is_active) {
          // If the starter is active, we consider the engine as cranking
          Serial.print(F("[STATE] RUNNING->CRANKING: Starter active, RPM below idle. RPM="));
          Serial.println(current_rpm);
          set_engine_cranking();
        } 
   
        break;

      case STOPPED:
            if ((n_ready_triggers < HOLT_INIT_SAMPLES || current_rpm < MIN_RPM) && starter_is_active) {
              n_ready_triggers++;
              if (n_ready_triggers == 1) {
                Serial.print(F("[STATE] STOPPED: Collecting triggers. Count="));
                Serial.print(n_ready_triggers);
                Serial.print(F(" RPM="));
                Serial.println(current_rpm);
              }
            } else if (n_ready_triggers >= HOLT_INIT_SAMPLES && current_rpm >= MIN_RPM && starter_is_active) {
              Serial.print(F("[STATE] STOPPED->READY: Enough triggers collected. RPM="));
              Serial.println(current_rpm);
              set_engine_ready();
            }
        break;

      case READY:
        if (starter_is_active && current_rpm <= CRANK_MAX_RPM) {
          if (crank_timer != 0 && current_millis - crank_timer > CRANK_DEBOUNCE_MS) {
            Serial.print(F("[STATE] READY->CRANKING: Starter active, debounce complete. RPM="));
            Serial.println(current_rpm);
            set_engine_cranking();
          } else if (crank_timer == 0) {
            crank_timer = current_millis;
            Serial.print(F("[STATE] READY: Starter active, starting debounce. RPM="));
            Serial.println(current_rpm);
          }
        } else if (current_rpm > CRANK_MAX_RPM) {
          // If RPM exceeds crank max while ready, might need to go to running
          if (current_rpm >= IDLE_MIN_RPM) {
            Serial.print(F("[STATE] READY->RUNNING: RPM exceeded idle minimum. RPM="));
            Serial.println(current_rpm);
            set_engine_running();
          }
        }
        break;

      case CRANKING:
        if (current_rpm >= IDLE_MIN_RPM) {
          Serial.print(F("[STATE] CRANKING->RUNNING: RPM reached idle minimum. RPM="));
          Serial.println(current_rpm);
          set_engine_running();
        } else if (!starter_is_active && current_rpm < CRANK_MAX_RPM) {
          Serial.print(F("[STATE] CRANKING->STOPPED: Starter off, RPM below crank max. RPM="));
          Serial.println(current_rpm);
          set_engine_stopped();
        } 

        break;


    }
  }

}
// External interrupt: crank trigger
ISR(INT0_vect) {
  uint16_t tcnt_candidate = System::get_timer_count();  // Snapshot Timer1 as early as possible
  uint32_t millis_candidate = millis(); // Snapshot millis for timestamp
  

  
  // Capture entry state
  bool dwelling_entry = Timing::is_dwelling();
  bool dwell_scheduled_entry = Timing::is_dwell_scheduled();




  // Calculate period in ticks for noise filtering (avoid expensive microsecond conversion)
  uint32_t period_ticks_candidate = (uint32_t)(tcnt_candidate - Timing::trigger_tcnt);
  uint32_t period_millis_candidate = millis_candidate - Timing::trigger_millis;

  // Adjust for possible Timer1 wraparound
  Timing::adjust_period_for_wraparound(period_ticks_candidate, period_millis_candidate);


  const uint32_t min_valid_ticks = max((Timing::period_ticks/3) , Timing::MIN_PERIOD_TICKS); // 33% or absolute min

  // 30% window filter: ignore triggers that are less than 30% of the previous period
  // Only apply if we have a valid previous period (after first pulse)
  if (period_ticks_candidate < min_valid_ticks && Timing::spark_enabled) {
    return;
  }

#ifdef ENABLE_DIAGNOSTIC_LOGGING
  Timing::next_trigger_id++;
#endif



  Timing::period_ticks = period_ticks_candidate;
  Timing::period_millis = period_millis_candidate;
  Timing::predicted_period_ticks = Timing::period_ticks;
  Timing::trigger_tcnt = tcnt_candidate;
  Timing::trigger_millis = millis_candidate;
  Timing::calculate_rpm();

  

  Timing::update_engine_state();
  


  // Capture timing mode before schedule_dwell modifies it
    uint8_t timing_mode_before = Timing::current_timing_mode;

  // Update engine timing state
  if (Timing::spark_enabled) {



    
    // Schedule dwell start (calculations done inside using predicted period)
    Timing::schedule_dwell();
  }
    
    // Store trigger event with final state (timing mode captured before and after)
    Timing::store_trigger_event(dwelling_entry, dwell_scheduled_entry, tcnt_candidate, timing_mode_before);
  
}



// Timer1 COMPA interrupt (dwell start)
ISR(TIMER1_COMPA_vect) {
    Timing::dwell_actual_started_tcnt = System::get_timer_count();
    System::disable_dwell_interrupt(); // Disable dwell interrupt
    Timing::start_dwell();
}

// Timer1 COMPB interrupt (spark fire)
ISR(TIMER1_COMPB_vect) {
    System::disable_spark_interrupt(); // Disable spark interrupt
    Timing::fire_spark();


}





void setup() {
  Serial.begin(115200);
  Serial.println(F("Rotax 787 Ignition Controller with timing curve"));
  Serial.print(F("Minimum period ticks: ")); Serial.println(Timing::MIN_PERIOD_TICKS);
  Serial.print(F("Initialized at ")); Serial.print(Timing::TIMER_TICKS_PER_SEC); Serial.println(F(" ticks per second"));
  Serial.print(F("Nominal dwell ticks: ")); Serial.println(Timing::NOMINAL_DWELL_TICKS);
  Serial.print(F("Max dwell ticks: ")); Serial.println(Timing::MAX_DWELL_TICKS);
  Serial.print(F("Min dwell ticks: ")); Serial.println(Timing::MIN_DWELL_TICKS);
  Serial.print(F("Max RPM: ")); Serial.println(Timing::MAX_RPM);
  Serial.print(F("Min RPM: ")); Serial.println(Timing::MIN_RPM);

  // Initialize spark pin
  System::set_spark_pin_output();
  System::set_spark_pin_safe();  // Initialize HIGH (safe state - no dwell)

  // Initialize relay control
  System::setup_relay();
  
  // Initialize starter input
  System::setup_starter_input();

  // Setup external interrupt for crank trigger
  System::setup_trigger_interrupt();

  // Setup Timer1 with prescaler /8 (0.5 us/tick)
  System::setup_timer();

  sei();
}





void loop() {
  uint32_t current_millis = millis();
  
  // Check if no triggers received for 500ms - engine must be stopped
  if (Timing::trigger_millis > 0) {  // Only check if we've received at least one trigger
    uint32_t time_since_trigger = current_millis - Timing::trigger_millis;

    if (current_millis > Timing::trigger_millis && time_since_trigger > 500 && Timing::engine_state != Timing::STOPPED) {
      Serial.print(F("No trigger for "));
      Serial.print(time_since_trigger);
      Serial.println(F("ms - Setting engine to STOPPED"));
      Timing::set_engine_stopped();
    }
  }
  
  // Arm relay when trigger pin is HIGH (with startup delay)
  if (!System::relay_armed && digitalRead(System::TRIGGER_PIN)) {
    if (System::startup_millis == 0) {
      System::startup_millis = current_millis;  // Record the time when trigger went HIGH
    } else if (current_millis - System::startup_millis > 1000) {  // 1-second delay
      System::arm_relay();  // Open relay to arm coil
      System::relay_armed = true;
      Serial.println(F("Relay armed and ready")); 
    }
  } 
}



