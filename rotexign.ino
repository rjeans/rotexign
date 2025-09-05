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

namespace Engine {


  enum EngineState {
    WAITING,
    DWELL_SCHEDULED,
    DWELLING
  };

  constexpr uint8_t NUMBER_STARTUP_TRIGGERS = 3; // Number of initial ticks to ignore for stable state

  volatile EngineState state = WAITING;
  volatile bool ignition_on = false;
  volatile uint8_t n_starting_triggers = 0; // Number of initial ticks collected before stable
  volatile uint16_t period_ticks = 0; // Period in ticks between last two triggers
  volatile uint16_t tcnt=0;
  volatile uint16_t dwell_ticks = 0;

}

namespace System {


  volatile uint32_t last_diagnostic_millis = 0;
  volatile uint32_t startup_millis = 0;
  volatile bool relay_armed = false;



}



namespace Timing {

struct TimingTriggerEvent {
    volatile uint16_t time;
    volatile uint16_t previous_time;
    volatile uint16_t period_ticks;
  };

  
    static constexpr uint8_t BUFFER_SIZE = 128; // Size of the ring buffer
    TimingTriggerEvent buffer[BUFFER_SIZE];
    uint8_t head = 0;
    uint8_t tail = 0;

    // Add an event to the buffer
    static inline void add_event(uint16_t time, uint16_t previous_time, uint16_t period_ticks) {
      buffer[head] = {time, previous_time, period_ticks};
      head = (head + 1) % BUFFER_SIZE;
      if (head == tail) {
        tail = (tail + 1) % BUFFER_SIZE; // Overwrite oldest event
      }
    }

    // Get an event from the buffer
    static inline TimingTriggerEvent * get_event() {
      if (head == tail) {
        return nullptr; // Buffer is empty
      }
      TimingTriggerEvent *event = &buffer[tail];
      tail = (tail + 1) % BUFFER_SIZE;
      return event;
    }

    // Check if the buffer is empty
    static inline bool events_ready() {
      return head != tail;
    }
  






  
  // Set the prescaler bits for a prescaler of 8.
  // The CS10 bit is NOT set.
  constexpr uint16_t PRESCALE_BITS = _BV(CS11); 

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



  static inline uint16_t rpm_from_period_ticks(uint16_t period_ticks) {
    if (!period_ticks) return 0;
    return (uint16_t)(RPM_NUMERATOR_TICKS / (uint32_t(period_ticks)));
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

  // Get timing advance angle from RPM using linear interpolation (returns tenths of degrees)
  static inline uint16_t get_advance_angle_tenths(uint16_t rpm) {
    // Edge cases
    uint16_t rpm_min = pgm_read_word(&timing_rpm_curve[0][0]);
    uint16_t adv_min = pgm_read_word(&timing_rpm_curve[0][1]);
    if (rpm <= rpm_min) return (uint16_t)(adv_min);

    uint16_t rpm_max = pgm_read_word(&timing_rpm_curve[TIMING_RPM_POINTS - 1][0]);
    uint16_t adv_max = pgm_read_word(&timing_rpm_curve[TIMING_RPM_POINTS - 1][1]);
    if (rpm >= rpm_max) return (uint16_t)(adv_max);

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
    return (uint16_t)result_tenths;
  }

  // Calculate spark delay in microseconds from RPM using advance angle and 47° BTDC trigger
  static inline uint16_t get_spark_delay(uint16_t period_ticks) {
    // Get advance angle in tenths of degrees
    uint16_t rpm = Timing::rpm_from_period_ticks(period_ticks);
    uint16_t advance_tenths = get_advance_angle_tenths(rpm);

    // Calculate delay angle: 47° - advance angle (in tenths)
    uint16_t delay_angle_tenths = TRIGGER_BTDC_TENTHS - advance_tenths;

    // Calculate delay in ticks
    // Delay_ticks >= (delay_angle / 180°) * period_ticks
    // Using tenths: delay_ticks = (delay_angle_tenths * period_ticks) / 1800
    // Add 900 (which is half of 1800) to round the result to the nearest integer
    uint16_t delay_ticks = ((uint32_t)delay_angle_tenths * (uint32_t)period_ticks + 900UL) / 1800UL;


   // Serial.print(F("[get_spark_delay] Period (ticks): ")); Serial.println(period_ticks);
   // Serial.print(F("[get_spark_delay] Spark Delay (ticks): ")); Serial.println(delay_ticks);
   // Serial.print(F("[get_spark_delay] Advance Angle (tenths): ")); Serial.println(advance_tenths);
   // Serial.print(F("[get_spark_delay] Delay Angle (tenths): ")); Serial.println(delay_angle_tenths);
   // Serial.print(F("[get_spark_delay] RPM: ")); Serial.println(rpm);

    // Return delay in timer ticks
    return delay_ticks;
  }

  // Calculate dwell delay in microseconds from RPM
  static inline uint16_t get_dwell_delay(uint16_t period_ticks,uint16_t dwell_ticks) {
    uint16_t spark_delay_ticks = get_spark_delay(period_ticks);

   // Serial.print(F("[get_dwell_delay] Period (ticks): ")); Serial.println(period_ticks);
   // Serial.print(F("[get_dwell_delay] Spark Delay (ticks): ")); Serial.println(spark_delay_ticks);
   // Serial.print(F("[get_dwell_delay] Dwell (ticks): ")); Serial.println(dwell_ticks);

    uint16_t dwell_delay_ticks;

  // Always force a period (previous lobe)

  if (dwell_ticks > spark_delay_ticks) {
      dwell_delay_ticks = (period_ticks + spark_delay_ticks) - dwell_ticks;

  } else {
      dwell_delay_ticks = spark_delay_ticks - dwell_ticks;

  }

   // Serial.print(F("[get_dwell_delay] Dwell delay (ticks): ")); Serial.println(dwell_delay_ticks);

    return dwell_delay_ticks;
  }

  // Get dwell time in microseconds from RPM
  static inline uint16_t get_dwell(uint16_t period_ticks) {

    uint16_t max_dwell_ticks = ((period_ticks + 50) / 100) * MAX_DUTY_CYCLE;
    uint16_t dwell_ticks = min(NOMINAL_DWELL_TICKS, max_dwell_ticks);

   // Serial.print(F("[get_dwell] Period (ticks): ")); Serial.println(period_ticks);
  //  Serial.print(F("[get_dwell] Nominal dwell (ticks): ")); Serial.println(NOMINAL_DWELL_TICKS);
   // Serial.print(F("[get_dwell] Dwell (ticks): ")); Serial.println(dwell_ticks);
   // Serial.print(F("[get_dwell] Max Dwell (ticks): ")); Serial.println(max_dwell_ticks);
    return dwell_ticks;
  }
}


namespace SerialInterface {
  // Print diagnostic status to Serial
  void print_status() {
    // Use new rpm_from_period_ticks to keep consistency
    uint16_t period_ticks = Engine::period_ticks;
    uint16_t dwell_ticks = Timing::get_dwell(period_ticks);
    uint16_t spark_delay_ticks = Timing::get_spark_delay(period_ticks);
    uint16_t dwell_delay_ticks = Timing::get_dwell_delay(period_ticks,dwell_ticks);
    uint16_t rpm = Timing::rpm_from_period_ticks(period_ticks);
    uint8_t advance_angle_tenths = Timing::get_advance_angle_tenths(rpm);
  

    Serial.print(F("Ignition ")); Serial.print(Engine::ignition_on ? F("on") : F("off"));
    Serial.print(F(" RPM: ")); Serial.print(rpm);
    Serial.print(F(" Period ticks: ")); Serial.print(period_ticks);
    Serial.print(F(" Advance angle: ")); Serial.print(advance_angle_tenths);
    Serial.print(F(" Spark delay (us): ")); Serial.print(Timing::ticks_to_us(spark_delay_ticks));
    Serial.print(F(" Dwell (us): ")); Serial.print(Timing::ticks_to_us(dwell_ticks));
    Serial.print(F(" Dwell delay (us): ")); Serial.print(Timing::ticks_to_us(dwell_delay_ticks));

    Serial.println();

 

  }
}



// Direct port control for FIRE_PIN (digital pin 3 = PD3 on ATmega328P)
#define FIRE_PORT  PORTD
#define FIRE_DDR   DDRD
#define FIRE_BIT   PD3
constexpr uint8_t FIRE_PIN = 3;          // Output pin for ignition pulse
constexpr uint8_t RELAY_PIN = 4;         // D4 - Relay control (HIGH = open/armed, LOW = closed/safe)

// Minimum safety margin for timer scheduling (100μs = 200 ticks at 0.5μs/tick)
constexpr uint16_t MIN_TIMER_LEAD_TICKS = 200;

// Schedule a COMPA one-shot at absolute tick 'when' (for dwell start)
// Returns true if scheduled successfully, false if immediate fallback used
static inline bool schedule_dwell(uint16_t when, uint16_t length) {
  uint16_t current_ticks = TCNT1;
  
  // Calculate lead time, handling timer wraparound properly
  // For 16-bit timer wraparound: treat differences > 32767 as negative (already passed)
  uint16_t lead_time = when - current_ticks;  // Unsigned subtraction handles wraparound
  
  // Check if we have enough lead time or if the time has already passed
  // If lead_time > 32767, the target time is in the past (wrapped around)
  if (lead_time < MIN_TIMER_LEAD_TICKS || lead_time > 32767) {
    // Too close or already past - start dwell immediately
    FIRE_PORT &= ~_BV(FIRE_BIT);  // Start dwell immediately (coil charging)
    Engine::state = Engine::DWELLING;
    
    // Calculate adjusted dwell length to maintain spark timing accuracy
    // Original spark time = when + length
    // Current time = current_ticks  
    // Adjusted length = (when + length) - current_ticks
    uint16_t original_spark_time = when + length;
    uint16_t adjusted_length;
    
    
    adjusted_length = original_spark_time - current_ticks;
    
    
    Engine::dwell_ticks = adjusted_length;


    // Schedule spark timing using adjusted length
    TIFR1  = _BV(OCF1A);
    OCR1A  = original_spark_time;  // Maintain original spark timing
    TIMSK1 |= _BV(OCIE1A);
    
    return false; // Indicate immediate fallback was used
  }
  
  // Safe to schedule normally
  TIFR1  = _BV(OCF1A);   // clear stale flag
  OCR1A  = when;         // set absolute time
  TIMSK1 |= _BV(OCIE1A); // enable COMPA interrupt
  Engine::dwell_ticks = length;
  Engine::state = Engine::DWELL_SCHEDULED;
  
  return true; // Indicate normal scheduling was used
}

// Schedule a COMPA one-shot at absolute tick 'when' (for spark fire)
static inline void schedule_spark(uint16_t when) {
  TIFR1  = _BV(OCF1A);
  OCR1A  = when;
  TIMSK1 |= _BV(OCIE1A);
}

static inline void start_dwell(uint16_t from, uint16_t length) {
  // Fast pin clear - LOW to start dwell
  
  Engine::state = Engine::DWELLING;
  FIRE_PORT &= (uint8_t)~_BV(FIRE_BIT);
  TIMSK1 &= ~_BV(OCIE1A);

  uint16_t t_off = (uint16_t)(from + length);

  schedule_spark(t_off);

}

// Fire spark (called from COMPA ISR)
static inline void fire_spark() {
  // Fast pin set - HIGH to fire spark
  FIRE_PORT |= _BV(FIRE_BIT);
  TIMSK1 &= ~_BV(OCIE1A);
  Engine::state = Engine::WAITING;

}


// External interrupt: crank trigger
ISR(INT0_vect) {
  uint16_t tcnt_candidate = TCNT1;  // Snapshot Timer1 as early as possible

 

  if (Engine::n_starting_triggers++ <= Engine::NUMBER_STARTUP_TRIGGERS) {
    // During startup phase, just count triggers to stabilize
    Engine::period_ticks = tcnt_candidate - Engine::tcnt;
    Engine::tcnt = tcnt_candidate;
    return;
  } 

    // Calculate period in ticks for noise filtering (avoid expensive microsecond conversion)
  uint16_t period_ticks_candidate = (uint16_t)(tcnt_candidate - Engine::tcnt);

  


  const uint16_t min_valid_ticks = max((Engine::period_ticks/3) , Timing::MIN_PERIOD_TICKS); // 33% or absolute min

  // 30% window filter: ignore triggers that are less than 30% of the previous period
  // Only apply if we have a valid previous period (after first pulse)
  if (period_ticks_candidate < min_valid_ticks && Engine::ignition_on) {

    return;
  }

    if (period_ticks_candidate < Timing::CUTOFF_PERIOD_TICKS) {
      Engine::ignition_on = false;
    }
    // Update engine timing state
    if (Engine::ignition_on) {
      Engine::period_ticks = period_ticks_candidate;
      uint16_t previous_time = Engine::tcnt;
      Engine::tcnt=tcnt_candidate;
      Timing::add_event(tcnt_candidate, previous_time, period_ticks_candidate);
    }



  



}



// Timer1 COMPA interrupt
ISR(TIMER1_COMPA_vect) {
  if (Engine::state == Engine::DWELL_SCHEDULED) {
    // Start dwell
    start_dwell(OCR1A,Engine::dwell_ticks);
  } else if (Engine::state == Engine::DWELLING) {
    // Fire spark
    fire_spark();
  } else {
    // Unexpected state - should not happen
    TIMSK1 &= ~_BV(OCIE1A); // Disable COMPA interrupt
    FIRE_PORT |= _BV(FIRE_BIT); // Ensure D3 HIGH (safe state - no dwell)
    Engine::state = Engine::WAITING;

  }

}


void setup() {
  Serial.begin(115200);
  Serial.println(F("Rotax 787 Ignition Controller with timing curve"));
  Serial.print(F("Minimum period ticks: ")); Serial.println(Timing::MIN_PERIOD_TICKS);
  Serial.print(F("Initialized at ")); Serial.print(Timing::TIMER_TICKS_PER_SEC); Serial.println(F(" ticks per second"));

  // Replace pinMode/digitalWrite for FIRE_PIN with direct register setup
  FIRE_DDR  |= _BV(FIRE_BIT);
  FIRE_PORT |= _BV(FIRE_BIT);  // Initialize HIGH (safe state - no dwell)

  // Initialize relay control pin (D4 = PD4)
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Start with relay closed (safe state - coil grounded)

  // INT0 on falling edge (D2)
  pinMode(2, INPUT_PULLUP);
  EICRA = (1 << ISC01);    // falling edge
  EIFR  = (1 << INTF0);    // clear stale
  EIMSK = (1 << INT0);     // enable

  // Timer1: normal mode, prescaler /64 (4 us/tick)
  TCCR1A = 0;
  TCCR1B = Timing::PRESCALE_BITS;  
  TCNT1  = 0;
  TIFR1  = _BV(TOV1) | _BV(OCF1A) | _BV(OCF1B); // clear stale flags

  sei();
}

namespace Logger {
  struct LogEntry {
    uint16_t time;
    uint16_t previous_time;
    uint16_t period_ticks;
    uint16_t dwell_ticks;
    uint16_t dwell_delay_ticks;
  };

  constexpr uint8_t LOG_BUFFER_SIZE = 128;
  LogEntry log_buffer[LOG_BUFFER_SIZE];
  uint8_t log_head = 0;
  uint8_t log_tail = 0;
  bool log_full = false;

  // Add an entry to the log buffer
  void add_log(uint16_t time, uint16_t previous_time, uint16_t period_ticks, uint16_t dwell_ticks, uint16_t dwell_delay_ticks) {
    log_buffer[log_head] = {time, previous_time, period_ticks, dwell_ticks, dwell_delay_ticks};
    log_head = (log_head + 1) % LOG_BUFFER_SIZE;
    if (log_head == log_tail) {
      log_full = true;
    }
  }

  // Print the log buffer
  void print_log() {
    Serial.println(F("Log buffer contents:"));
    uint8_t index = log_tail;
    for (uint8_t i= 0; i < LOG_BUFFER_SIZE; i++) {
      LogEntry &entry = log_buffer[index];
      Serial.print(F("Time: ")); Serial.print(entry.time/2);
      Serial.print(F(", Previous Time: ")); Serial.print(entry.previous_time/2);
      Serial.print(F(", Period: ")); Serial.print(entry.period_ticks/2);
      Serial.print(F(", Dwell: ")); Serial.print(entry.dwell_ticks/2);
      Serial.print(F(", Dwell_delay: ")); Serial.print(entry.dwell_delay_ticks/2);
      Serial.print(F(", Delay: ")); Serial.print((entry.dwell_delay_ticks+entry.dwell_ticks)/2);
      Serial.print(F(", Spark at: ")); Serial.print((entry.dwell_delay_ticks+entry.dwell_ticks+entry.time)/2);
      Serial.println("");
      index = (index + 1) % LOG_BUFFER_SIZE;
    }
  }

  // Clear the log buffer
  void clear_log() {
    log_head = 0;
    log_tail = 0;
    log_full = false;
  }
}



void loop() {
  // Arm relay when D2 is HIGH (with startup delay)
  if (!System::relay_armed && digitalRead(2)) {
    if (System::startup_millis == 0) {
      System::startup_millis = millis();  // Record the time when trigger went HIGH
    } else if (millis() - System::startup_millis > 1000) {  // 1-second delay
      digitalWrite(RELAY_PIN, HIGH);  // Open relay to arm coil
      System::relay_armed = true;
      Engine::ignition_on = true;
      Serial.println(F("Relay armed and ready")); 
    }
  } else {



  if (Engine::state == Engine::WAITING && Timing::events_ready() && Engine::ignition_on) {
    // Start dwell process
   

    Timing::TimingTriggerEvent * event = Timing::get_event();

    uint16_t dwell_ticks = Timing::get_dwell(event->period_ticks);
    uint16_t dwell_delay_ticks = Timing::get_dwell_delay(event->period_ticks,dwell_ticks);
/*
    if (Logger::log_full) {
  Serial.println(F("Log buffer full. Stopping engine."));
  Engine::ignition_on  = false;
  Logger::print_log();
  Logger::clear_log();
} else {
  Logger::add_log(event->time, event->previous_time, event->period_ticks, dwell_ticks, dwell_delay_ticks);


}
*/




  //  Serial.print(F("(MAIN) Time=")); Serial.print(event->time);
  //  Serial.print(F(" Period_ticks=")); Serial.print(event->period_ticks);
  //  Serial.print(F(" Dwell: ")); Serial.print(dwell_ticks); Serial.print(F(" us, Delay: ")); Serial.print(dwell_delay_ticks); Serial.println(F(" us"));
   
  

    // Schedule dwell with safety protection
    bool scheduled_normally = schedule_dwell(event->time + dwell_delay_ticks, dwell_ticks);


  }

}
}



