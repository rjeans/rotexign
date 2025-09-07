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

}



namespace Timing {

    enum TimingState {
    SPARK_FIRED,     // Spark has fired, waiting for next trigger
    DWELL_SCHEDULED, // Dwell start scheduled via timer interrupt
    DWELLING         // Currently dwelling (coil charging), spark scheduled
  };

  constexpr uint8_t NUMBER_STARTUP_TRIGGERS = 3; // Number of initial ticks to ignore for stable state

  volatile TimingState state = SPARK_FIRED;
  volatile bool ignition_on = false;
  volatile uint8_t n_starting_triggers = 0; // Number of initial ticks collected before stable
  volatile uint16_t period_ticks = 0; // Period in ticks between last two triggers
  volatile uint16_t last_trigger_tcnt = 0;
  volatile uint16_t dwell_ticks = 0; // Dwell duration in ticks
  volatile uint16_t dwell_delay_ticks = 0; // Delay from trigger to dwell start
  volatile uint16_t spark_delay_ticks = 0; // Delay from trigger to spark
  volatile uint16_t rpm = 0; // Current RPM
  volatile uint16_t advance_tenths = 0; // Advance angle in tenths of degrees
  volatile uint16_t delay_angle_tenths = 0; // Delay angle in tenths of degrees



struct TimingTriggerEvent {
    volatile uint16_t time;

  };

  
    static constexpr uint8_t BUFFER_SIZE = 128; // Size of the ring buffer
    TimingTriggerEvent buffer[BUFFER_SIZE];
    uint8_t head = 0;
    uint8_t tail = 0;

    // Add an event to the buffer
    static inline void add_event(uint16_t time) {
      buffer[head] = {time};
      head = (head + 1) % BUFFER_SIZE;
      if (head == tail) {
        tail = (tail + 1) % BUFFER_SIZE; // Overwrite oldest event
      }
    }








  


  // Calculate the number of timer ticks per second with a prescaler of 8.
  // 16,000,000 Hz / 8 = 2,000,000 ticks/sec
  constexpr uint32_t TIMER_TICKS_PER_SEC = (F_CPU / 8UL);
  

  const uint8_t  PULSES_PER_REVOLUTION = 2;  // Number of pulses per engine revolution
  // The time per tick is the reciprocal of the ticks per second.
  // 1 / 2,000,000 = 0.5 microseconds per tick

  constexpr uint32_t RPM_NUMERATOR_TICKS = TIMER_TICKS_PER_SEC * 60UL / PULSES_PER_REVOLUTION;

  const uint16_t MAX_DUTY_CYCLE = 40; 
  const uint16_t NOMINAL_DWELL_MS = 3;
  constexpr uint16_t NOMINAL_DWELL_TICKS = (uint16_t)(( (uint32_t)NOMINAL_DWELL_MS * TIMER_TICKS_PER_SEC) / 1000UL);
  const uint16_t MAX_RPM = 8000;
  const uint16_t MIN_PERIOD_TICKS =  (uint16_t)(RPM_NUMERATOR_TICKS / (uint32_t(MAX_RPM)));
  const uint16_t CUTOFF_RPM = 7000;
  const uint16_t CUTOFF_PERIOD_TICKS =  (uint16_t)(RPM_NUMERATOR_TICKS / (uint32_t(CUTOFF_RPM)));
  const uint16_t TRIGGER_BTDC_TENTHS = 470;  // 47.0 degrees in tenths



  // Convert Timer1 ticks at prescaler  back to microseconds (F_CPU = 16 MHz)
  static inline uint16_t ticks_to_us(uint16_t ticks) {
    return ticks/2 ;
  }



  static inline void calculate_rpm() {
    if (!period_ticks) {
      rpm = 0;
    } else {
      rpm = (uint16_t)(RPM_NUMERATOR_TICKS / (uint32_t(period_ticks)));
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
  static inline void calculate_advance_angle_tenths() {
    // Edge cases
    uint16_t rpm_min = pgm_read_word(&timing_rpm_curve[0][0]);
    uint16_t adv_min = pgm_read_word(&timing_rpm_curve[0][1]);
    if (rpm <= rpm_min) {
      advance_tenths = (uint16_t)(adv_min);
      return;
    }

    uint16_t rpm_max = pgm_read_word(&timing_rpm_curve[TIMING_RPM_POINTS - 1][0]);
    uint16_t adv_max = pgm_read_word(&timing_rpm_curve[TIMING_RPM_POINTS - 1][1]);
    if (rpm >= rpm_max) {
      advance_tenths = (uint16_t)(adv_max);
      return;
    }

    // Must avoid overflow
    uint16_t i = (uint32_t(rpm) * (TIMING_RPM_POINTS - 1) + MAX_TIMING_RPM - 1) / MAX_TIMING_RPM;

    uint16_t rpm_low  = pgm_read_word(&timing_rpm_curve[i - 1][0]);
    uint16_t rpm_high = pgm_read_word(&timing_rpm_curve[i][0]);

    uint16_t adv_low_u  = pgm_read_word(&timing_rpm_curve[i - 1][1]);
    uint16_t adv_high_u = pgm_read_word(&timing_rpm_curve[i][1]);

    // Do math in signed to handle negative slope
    int16_t  adv_low  = (int16_t)adv_low_u;
    int16_t  adv_high = (int16_t)adv_high_u;
    int16_t  d_adv    = (int16_t)(adv_high - adv_low);

    int32_t  d_rpm    = (int32_t)(rpm - rpm_low);
    int32_t  den      = (int32_t)(rpm_high - rpm_low);

    // numerator = d_adv * d_rpm * 10   (tenths of a degree)
    int32_t  num = (int32_t)d_adv * d_rpm;

    // Round to nearest: add or subtract half-denominator based on sign
    int32_t  frac_tenths = (num >= 0) ? (num + den / 2) / den
                                      : (num - den / 2) / den;

    int32_t  result_tenths = (int32_t)adv_low + frac_tenths;
    if (result_tenths < 0) result_tenths = 0;                 // optional clamp
    if (result_tenths > 1000) result_tenths = 1000;           // optional clamp
    advance_tenths = (uint16_t)result_tenths;
  }

  // Calculate spark delay from trigger to spark fire
  static inline void calculate_spark_delay() {
    // Calculate RPM and advance angle
    calculate_rpm();
    calculate_advance_angle_tenths();

    // Calculate delay angle: 47° - advance angle (in tenths)
    delay_angle_tenths = TRIGGER_BTDC_TENTHS - advance_tenths;

    // Calculate delay in ticks
    // Delay_ticks >= (delay_angle / 180°) * period_ticks
    // Using tenths: delay_ticks = (delay_angle_tenths * period_ticks) / 1800
    // Add 900 (which is half of 1800) to round the result to the nearest integer
    spark_delay_ticks = ((uint32_t)delay_angle_tenths * (uint32_t)period_ticks + 900UL) / 1800UL;
  }

  // Calculate dwell delay from trigger to dwell start
  static inline void calculate_dwell_delay() {
    // Always force a period (previous lobe)
    if (dwell_ticks > spark_delay_ticks) {
      dwell_delay_ticks = (period_ticks + spark_delay_ticks) - dwell_ticks;
    } else {
      dwell_delay_ticks = spark_delay_ticks - dwell_ticks;
    }
  }

  // Calculate dwell duration based on RPM and duty cycle limits
  static inline void calculate_dwell() {
    uint16_t max_dwell_ticks = ((period_ticks + 50) / 100) * MAX_DUTY_CYCLE;
    dwell_ticks = min(NOMINAL_DWELL_TICKS, max_dwell_ticks);
  }







// Minimum safety margin for timer scheduling (100μs = 200 ticks at 0.5μs/tick)
constexpr uint16_t MIN_TIMER_LEAD_TICKS = 200;

// Schedule a COMPA one-shot at absolute tick 'when' (for dwell start)
static inline void schedule_dwell() {
  // Calculate all timing values
  calculate_dwell();
  calculate_spark_delay();
  calculate_dwell_delay();
  
  // Calculate when to start dwell
  uint16_t when = last_trigger_tcnt + dwell_delay_ticks;
  uint16_t current_ticks = System::get_timer_count();
  
  // Calculate lead time, handling timer wraparound properly
  // For 16-bit timer wraparound: treat differences > 32767 as negative (already passed)
  uint16_t lead_time = when - current_ticks;  // Unsigned subtraction handles wraparound
  
  // Check if we have enough lead time or if the time has already passed
  // If lead_time > 32767, the target time is in the past (wrapped around)
  if (lead_time < MIN_TIMER_LEAD_TICKS || lead_time > 32767) {
    // Too close or already past - start dwell immediately
    System::spark_pin_low();  // Start dwell immediately (coil charging)
    Timing::state = Timing::DWELLING;
    
    // Calculate adjusted dwell length to maintain spark timing accuracy
    // Original spark time = when + dwell_ticks
    // Current time = current_ticks  
    // Adjusted length = (when + dwell_ticks) - current_ticks
    uint16_t original_spark_time = when + Timing::dwell_ticks;
    uint16_t adjusted_length = original_spark_time - current_ticks;
    
    // Update dwell_ticks for the ISR to use
    Timing::dwell_ticks = adjusted_length;

    // Schedule spark timing using adjusted length
    System::clear_dwell_flag();
    System::set_dwell_compare(original_spark_time);  // Maintain original spark timing
    System::enable_dwell_interrupt();
    
    return;
  }
  
  // Safe to schedule normally
  System::clear_dwell_flag();   // clear stale flag
  System::set_dwell_compare(when);         // set absolute time
  System::enable_dwell_interrupt(); // enable dwell interrupt
  Timing::state = Timing::DWELL_SCHEDULED;
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
  Timing::state = Timing::DWELLING;
  System::spark_pin_low();

  // Schedule spark to fire after dwell period
  schedule_spark();
}

// Fire spark (called from COMPB ISR)
static inline void fire_spark() {
  // Fast pin set - HIGH to fire spark
  System::spark_pin_high();

  Timing::state = Timing::SPARK_FIRED;

}

}
// External interrupt: crank trigger
ISR(INT0_vect) {
  uint16_t tcnt_candidate = System::get_timer_count();  // Snapshot Timer1 as early as possible

 

  if (Timing::n_starting_triggers++ <= Timing::NUMBER_STARTUP_TRIGGERS) {
    // During startup phase, just count triggers to stabilize
    Timing::period_ticks = tcnt_candidate - Timing::last_trigger_tcnt;
    Timing::last_trigger_tcnt = tcnt_candidate;
    return;
  } 

    // Calculate period in ticks for noise filtering (avoid expensive microsecond conversion)
  uint16_t period_ticks_candidate = (uint16_t)(tcnt_candidate - Timing::last_trigger_tcnt);

  


  const uint16_t min_valid_ticks = max((Timing::period_ticks/3) , Timing::MIN_PERIOD_TICKS); // 33% or absolute min

  // 30% window filter: ignore triggers that are less than 30% of the previous period
  // Only apply if we have a valid previous period (after first pulse)
  if (period_ticks_candidate < min_valid_ticks && Timing::ignition_on) {

    return;
  }

    if (period_ticks_candidate < Timing::CUTOFF_PERIOD_TICKS) {
      Timing::ignition_on = false;
    }
    // Update engine timing state
    if (Timing::ignition_on) {
      Timing::period_ticks = period_ticks_candidate;
      uint16_t previous_time = Timing::last_trigger_tcnt;
      Timing::last_trigger_tcnt=tcnt_candidate;

      // Schedule dwell start (calculations done inside)
      Timing::schedule_dwell();


 
    }



  



}



// Timer1 COMPA interrupt (dwell start)
ISR(TIMER1_COMPA_vect) {
    System::disable_dwell_interrupt(); // Disable dwell interrupt
    Timing::start_dwell();
}

// Timer1 COMPB interrupt (spark fire)
ISR(TIMER1_COMPB_vect) {
    System::disable_spark_interrupt(); // Disable spark interrupt
    Timing::fire_spark();


}

namespace SerialInterface {
  // Print diagnostic status to Serial
  void print_status() {
    // Print current static timing values (no recalculation needed)
    Serial.print(F("Ignition ")); Serial.print(Timing::ignition_on ? F("on") : F("off"));
    Serial.print(F(" RPM: ")); Serial.print(Timing::rpm);
    Serial.print(F(" Period ticks: ")); Serial.print(Timing::period_ticks);
    Serial.print(F(" Advance angle: ")); Serial.print(Timing::advance_tenths);
    Serial.print(F(" Spark delay (us): ")); Serial.print(Timing::ticks_to_us(Timing::spark_delay_ticks));
    Serial.print(F(" Dwell (us): ")); Serial.print(Timing::ticks_to_us(Timing::dwell_ticks));
    Serial.print(F(" Dwell delay (us): ")); Serial.print(Timing::ticks_to_us(Timing::dwell_delay_ticks));

    Serial.println();

 

  }
}



void setup() {
  Serial.begin(115200);
  Serial.println(F("Rotax 787 Ignition Controller with timing curve"));
  Serial.print(F("Minimum period ticks: ")); Serial.println(Timing::MIN_PERIOD_TICKS);
  Serial.print(F("Initialized at ")); Serial.print(Timing::TIMER_TICKS_PER_SEC); Serial.println(F(" ticks per second"));

  // Initialize spark pin
  System::set_spark_pin_output();
  System::set_spark_pin_safe();  // Initialize HIGH (safe state - no dwell)

  // Initialize relay control
  System::setup_relay();

  // Setup external interrupt for crank trigger
  System::setup_trigger_interrupt();

  // Setup Timer1 with prescaler /8 (0.5 us/tick)
  System::setup_timer();

  sei();
}





void loop() {
  // Arm relay when trigger pin is HIGH (with startup delay)
  if (!System::relay_armed && digitalRead(System::TRIGGER_PIN)) {
    if (System::startup_millis == 0) {
      System::startup_millis = millis();  // Record the time when trigger went HIGH
    } else if (millis() - System::startup_millis > 1000) {  // 1-second delay
      System::arm_relay();  // Open relay to arm coil
      System::relay_armed = true;
      Timing::ignition_on = true;
      Serial.println(F("Relay armed and ready")); 
    }
  } 
}



