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

struct EngineState {
  volatile uint16_t last_interrupt_ticks = 0;
  volatile uint16_t this_interrupt_ticks = 0;
  volatile uint16_t period_ticks = 0;
  volatile uint16_t n_starting_ticks = 0;
  volatile uint16_t next_dwell_ticks = 0;
  volatile bool running = false;
};

static EngineState engine;

struct SystemState {
  volatile uint32_t last_diagnostic_millis = 0;
  volatile uint32_t startup_millis = 0;
  volatile bool relay_armed = false;
};

static volatile SystemState sys;

namespace Utils {
  // Convert microseconds to Timer1 ticks at prescaler /64 (F_CPU = 16 MHz)
  static inline uint16_t us_to_ticks64(uint32_t us) {
    // round to nearest tick
    return (uint16_t)((us + 2) / 4);
  }

  // Convert Timer1 ticks at prescaler /64 back to microseconds (F_CPU = 16 MHz)
  static inline uint32_t ticks64_to_us(uint16_t ticks) {
    return (uint32_t)ticks * 4UL;
  }

  // Direct RPM from period ticks (avoids microsecond conversion).
  // Formula: RPM = (timer_ticks_per_sec * 60) / (period_ticks * pulses_per_rev)
  // timer_ticks_per_sec = F_CPU / prescaler = 16,000,000 / 64 = 250,000
  // Numerator = 250,000 * 60 = 15,000,000
  constexpr uint32_t TIMER_TICKS_PER_SEC = (F_CPU / 64UL);
  constexpr uint32_t RPM_NUMERATOR = TIMER_TICKS_PER_SEC * 60UL;

  static inline uint16_t rpm_from_period_ticks(uint16_t period_ticks, uint8_t pulses_per_rev) {
    if (!period_ticks) return 0;
    return (uint16_t)(RPM_NUMERATOR / (uint32_t(period_ticks) * pulses_per_rev));
  }
}

namespace Timing {
  const uint16_t MAX_DUTY_CYCLE = 40; 
  const uint16_t NOMINAL_DWELL_US = 3000;
  const uint16_t MAX_RPM = 7000;
  const uint16_t DWELL_TO_TRIGGER_MARGIN_US = 40;
  const uint16_t TRIGGER_BTDC_TENTHS = 470;  // 47.0 degrees in tenths
  const uint8_t  PULSES_PER_REVOLUTION = 2;

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

  #define TIMING_RPM_POINTS 201
  #define MAX_TIMING_RPM 8000

  // Get timing advance angle from RPM using linear interpolation (returns tenths of degrees)
  uint16_t get_advance_angle_tenths(uint16_t rpm) {
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
  uint16_t get_spark_delay_us_from_rpm(uint16_t rpm) {
    // Get advance angle in tenths of degrees
    uint16_t advance_tenths = get_advance_angle_tenths(rpm);

    // Calculate delay angle: 47° - advance angle (in tenths)
    uint16_t delay_angle_tenths = TRIGGER_BTDC_TENTHS - advance_tenths;

    // Calculate period in microseconds
    // Period_us = 60,000,000 / (RPM * 2 pulses_per_rev)
    uint32_t period_us = 60000000UL / (rpm * PULSES_PER_REVOLUTION);

    // Calculate delay in microseconds
    // Delay_us = (delay_angle / 180°) * period_us
    // Using tenths: delay_us = (delay_angle_tenths * period_us) / 1800
    // Adding 900 provides for the correct rounding
    uint32_t delay_us = ((uint32_t)delay_angle_tenths * period_us + 900) / 1800;

    // Return delay in microseconds
    return delay_us;
  }

  // Calculate dwell delay in microseconds from RPM
  uint16_t get_dwell_delay_us_from_rpm(uint16_t rpm) {
    uint16_t spark_delay_us = get_spark_delay_us_from_rpm(rpm);
    uint32_t period_us = 60000000UL / (rpm * PULSES_PER_REVOLUTION);
    uint32_t max_dwell_us = ((period_us + 50) / 100) * MAX_DUTY_CYCLE;
    uint16_t dwell_us = min(NOMINAL_DWELL_US, max_dwell_us);

    boolean skip_period = (dwell_us + DWELL_TO_TRIGGER_MARGIN_US) > spark_delay_us;
    uint16_t dwell_delay_us;

    if (skip_period) {
      dwell_delay_us = (period_us + spark_delay_us) - dwell_us;
    } else {
      dwell_delay_us = spark_delay_us - dwell_us;
    }
    return dwell_delay_us;
  }

  // Get dwell time in microseconds from RPM
  uint16_t get_dwell_us_from_rpm(uint16_t rpm) {
    // Removed unused spark_delay_us to save ISR path work when inlined.
    uint32_t period_us = 60000000UL / (rpm * PULSES_PER_REVOLUTION);
    uint32_t max_dwell_us = ((period_us + 50) / 100) * MAX_DUTY_CYCLE;
    uint16_t dwell_us = min(NOMINAL_DWELL_US, max_dwell_us);
    return dwell_us;
  }
}


namespace SerialInterface {
  // Print diagnostic status to Serial
  void print_status() {
    // Use new rpm_from_period_ticks to keep consistency
    uint16_t period_ticks = engine.period_ticks;
    uint16_t rpm = Utils::rpm_from_period_ticks(period_ticks, Timing::PULSES_PER_REVOLUTION);

    Serial.print(F("Engine ")); Serial.print(engine.running ? F("running") : F("stopped"));
    Serial.print(F(" RPM: ")); Serial.print(rpm);
    Serial.print(F(" Advance angle: ")); Serial.print(Timing::get_advance_angle_tenths(rpm));
    Serial.print(F(" Spark delay (us): ")); Serial.print(Timing::get_spark_delay_us_from_rpm(rpm));
    Serial.print(F(" Dwell delay (us): ")); Serial.print(Timing::get_dwell_delay_us_from_rpm(rpm));
    Serial.print(F(" Dwell (us): ")); Serial.print(Timing::get_dwell_us_from_rpm(rpm));
    Serial.println();

 

  }
}



// Direct port control for FIRE_PIN (digital pin 3 = PD3 on ATmega328P)
#define FIRE_PORT  PORTD
#define FIRE_DDR   DDRD
#define FIRE_BIT   PD3
constexpr uint8_t FIRE_PIN = 3;          // Output pin for ignition pulse
constexpr uint8_t RELAY_PIN = 4;         // D4 - Relay control (HIGH = open/armed, LOW = closed/safe)
constexpr uint16_t PRESCALE_BITS = _BV(CS11) | _BV(CS10); // /64 -> 4us/tick

// Schedule a COMPB one-shot at absolute tick 'when'
static inline void schedule_B(uint16_t when) {
  TIFR1  = _BV(OCF1B);   // clear stale flag
  OCR1B  = when;         // set absolute time
  TIMSK1 |= _BV(OCIE1B); // enable COMPB interrupt
}

// Schedule a COMPA one-shot at absolute tick 'when' (for pulse end)
static inline void schedule_A(uint16_t when) {
  TIFR1  = _BV(OCF1A);
  OCR1A  = when;
  TIMSK1 |= _BV(OCIE1A);
}


// External interrupt: crank trigger
ISR(INT0_vect) {
  uint16_t tcnt = TCNT1;  // Snapshot Timer1 as early as possible

  // Calculate period in ticks for noise filtering (avoid expensive microsecond conversion)
  uint16_t period_ticks_candidate = (uint16_t)(tcnt - engine.this_interrupt_ticks);
  
  const uint16_t min_ticks_by_rpm = 1000;

  //min_valid_ticks = max(engine.period_ticks / 10, min_ticks_by_rpm); // 30% or absolute min 

  
  
  // 30% window filter: ignore triggers that are less than 30% of the previous period
  // Only apply if we have a valid previous period
  if (engine.n_starting_ticks > 2 && period_ticks_candidate < min_ticks_by_rpm) {
    
 
      return;  // Ignore this trigger - likely bounce or noise
    
  }
  
  
 
  // Update engine timing state
 
  engine.last_interrupt_ticks = engine.this_interrupt_ticks;
  engine.this_interrupt_ticks = tcnt;
  engine.period_ticks = period_ticks_candidate;

  if (engine.n_starting_ticks<=2  ) { 
    engine.n_starting_ticks++; 
    return; 
  }
  

  // Calculate RPM and update engine state (MAX_RPM check covers high RPM filtering)
  uint16_t rpm = Utils::rpm_from_period_ticks(engine.period_ticks, Timing::PULSES_PER_REVOLUTION);
  engine.running = ( rpm <= Timing::MAX_RPM);



      

  if (engine.running) {
 
    // Precompute dwell metrics
    uint16_t dwell_ticks = Utils::us_to_ticks64(Timing::get_dwell_us_from_rpm(rpm));
    uint16_t dwell_delay_ticks = Utils::us_to_ticks64(Timing::get_dwell_delay_us_from_rpm(rpm));
    engine.next_dwell_ticks = dwell_ticks;

    // Schedule dwell start
    schedule_B(engine.this_interrupt_ticks + dwell_delay_ticks);
  } else {
    // Engine stopped - cancel any pending timer interrupts and ensure safe state
    TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));  // Disable both compare interrupts
    FIRE_PORT |= _BV(FIRE_BIT);              // Ensure D3 HIGH (safe state - no dwell)
  } 
}

// Timer1 COMPB interrupt: start dwell (output goes LOW)
ISR(TIMER1_COMPB_vect) {
  // Fast pin clear - LOW to start dwell
  FIRE_PORT &= (uint8_t)~_BV(FIRE_BIT);
  TIMSK1 &= ~_BV(OCIE1B);

  const uint16_t width_ticks = engine.next_dwell_ticks;
  uint16_t t_off = (uint16_t)(OCR1B + width_ticks);
  schedule_A(t_off);
}

// Timer1 COMPA interrupt: fire spark (output goes HIGH)
ISR(TIMER1_COMPA_vect) {
  // Fast pin set - HIGH to fire spark
  FIRE_PORT |= _BV(FIRE_BIT);
  TIMSK1 &= ~_BV(OCIE1A);
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Rotax 787 Ignition Controller"));

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
  TCCR1B = PRESCALE_BITS;  // CS12:0 = 011 -> /64
  TCNT1  = 0;
  TIFR1  = _BV(TOV1) | _BV(OCF1A) | _BV(OCF1B); // clear stale flags

  sei();
}



void loop() {
  // Arm relay when D2 is HIGH (with startup delay)
  if (!sys.relay_armed && digitalRead(2)) {
    if (sys.startup_millis == 0) {
      sys.startup_millis = millis();  // Record the time when trigger went HIGH
    } else if (millis() - sys.startup_millis > 1000) {  // 1-second delay
      digitalWrite(RELAY_PIN, HIGH);  // Open relay to arm coil
      sys.relay_armed = true;
      Serial.println(F("Relay armed - coil ready"));
    }
  }

  // Print diagnostics every 5 seconds
  if (millis() - sys.last_diagnostic_millis > 5000) {
    SerialInterface::print_status();
    sys.last_diagnostic_millis = millis();
  }

 
}



