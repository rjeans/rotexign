/*
  Rotax 787 Ignition Controller
  -----------------------------
  This Arduino sketch implements a programmable ignition controller for the Rotax 787 engine.
  It uses Timer1 and external interrupts to precisely control ignition timing and dwell based on engine RPM.
  The timing advance curve is stored in PROGMEM and interpolated for smooth operation.
  Output is provided on FIRE_PIN, and diagnostic information is printed periodically via Serial.
  Author: [Your Name]
  Date: [Date]
*/

#include <avr/interrupt.h>

struct EngineState {
  volatile uint16_t last_interrupt_ticks = 0;
  volatile uint16_t this_interrupt_ticks = 0;
  volatile uint16_t period_ticks = 0;
  volatile uint16_t n_ticks = 0;
  volatile uint16_t next_dwell_ticks = 0;
};

static EngineState engine;

struct SystemState {
  volatile uint32_t last_diagnostic_millis = 0;
  volatile bool engine_running = false;
};

static SystemState sys;

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
}

namespace Timing {
  const uint16_t MAX_DUTY_CYCLE = 40; 
  const uint16_t NOMINAL_DWELL_US = 3000;
  const uint16_t DWELL_TO_TRIGGER_MARGIN_US = 20;
  const uint16_t TRIGGER_BTDC_TENTHS = 470;  // 47.0 degrees in tenths
  const uint8_t  PULSES_PER_REVOLUTION = 2;

  // Timing advance curve: {RPM, advance in tenths of degrees}
  static const uint16_t timing_rpm_curve[][2] PROGMEM = {
    {0, 0}, {100, 5}, {200, 10}, {300, 16}, {400, 22}, {500, 28}, {600, 34}, {700, 40},
    {800, 47}, {900, 53}, {1000, 60}, {1100, 67}, {1200, 73}, {1300, 80}, {1400, 86},
    {1500, 92}, {1600, 98}, {1700, 104}, {1800, 110}, {1900, 115}, {2000, 120},
    {2100, 125}, {2200, 129}, {2300, 133}, {2400, 136}, {2500, 139}, {2600, 142},
    {2700, 145}, {2800, 147}, {2900, 149}, {3000, 150}, {3100, 151}, {3200, 152},
    {3300, 152}, {3400, 153}, {3500, 153}, {3600, 152}, {3700, 152}, {3800, 151},
    {3900, 151}, {4000, 150}, {4100, 149}, {4200, 148}, {4300, 147}, {4400, 146},
    {4500, 145}, {4600, 144}, {4700, 143}, {4800, 142}, {4900, 141}, {5000, 140},
    {5100, 139}, {5200, 138}, {5300, 137}, {5400, 136}, {5500, 135}, {5600, 134},
    {5700, 133}, {5800, 132}, {5900, 131}, {6000, 130}, {6100, 129}, {6200, 128},
    {6300, 127}, {6400, 126}, {6500, 125}, {6600, 124}, {6700, 123}, {6800, 122},
    {6900, 121}, {7000, 120}, {7100, 119}, {7200, 118}, {7300, 117}, {7400, 116},
    {7500, 115}, {7600, 114}, {7700, 113}, {7800, 112}, {7900, 111}, {8000, 110}
  };

  #define TIMING_RPM_POINTS 81
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

    if (rpm < rpm_low || rpm > rpm_high) {
      Serial.print("RPM= "); Serial.print(rpm);
      Serial.print(" i="); Serial.print(i);
      Serial.println("");
    }

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
    uint16_t spark_delay_us = get_spark_delay_us_from_rpm(rpm);
    uint32_t period_us = 60000000UL / (rpm * PULSES_PER_REVOLUTION);
    uint32_t max_dwell_us = ((period_us + 50) / 100) * MAX_DUTY_CYCLE;
    uint16_t dwell_us = min(NOMINAL_DWELL_US, max_dwell_us);
    return dwell_us;
  }
}

namespace SerialInterface {
  // Print diagnostic status to Serial
  void print_status() {
    uint16_t period_ticks = engine.period_ticks;
    uint32_t period_us = Utils::ticks64_to_us(period_ticks); 
    uint16_t rpm = 60000000 / (period_us * Timing::PULSES_PER_REVOLUTION);

    Serial.print(F("RPM: ")); Serial.print(rpm);
    Serial.print(F(" Advance angle: ")); Serial.print(Timing::get_advance_angle_tenths(rpm));
    Serial.print(F(" Spark delay (us): ")); Serial.print(Timing::get_spark_delay_us_from_rpm(rpm));
    Serial.print(F(" Dwell delay (us): ")); Serial.print(Timing::get_dwell_delay_us_from_rpm(rpm));
    Serial.print(F(" Dwell (us): ")); Serial.print(Timing::get_dwell_us_from_rpm(rpm));
    Serial.println();

  }
}

constexpr uint8_t FIRE_PIN = 3;          // Output pin for ignition pulse
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
  if (!engine.n_ticks++) {
    engine.last_interrupt_ticks = TCNT1;
  } else {
    sys.engine_running = true;
    engine.last_interrupt_ticks = engine.this_interrupt_ticks;
    engine.this_interrupt_ticks = TCNT1;
    engine.period_ticks = (uint16_t)(engine.this_interrupt_ticks - engine.last_interrupt_ticks);

    // Fast lookup using pre-calculated table - no expensive calculations in interrupt!
    // Timing::TimingEntry timing = Timing::fast_timing_lookup(engine.period_ticks);
    // engine.next_dwell_ticks = timing.dwell_ticks;
    // schedule_B(engine.this_interrupt_ticks + timing.dwell_delay_ticks);

    uint32_t period_us = Utils::ticks64_to_us(engine.period_ticks); 
    uint16_t rpm = 60000000 / (period_us * Timing::PULSES_PER_REVOLUTION);

    uint16_t dwell_ticks = Utils::us_to_ticks64(Timing::get_dwell_us_from_rpm(rpm));
    uint16_t dwell_delay_ticks = Utils::us_to_ticks64(Timing::get_dwell_delay_us_from_rpm(rpm));

    engine.next_dwell_ticks = dwell_ticks;
    schedule_B(engine.this_interrupt_ticks + dwell_delay_ticks);
  }
}

// Timer1 COMPB interrupt: start ignition pulse
ISR(TIMER1_COMPB_vect) {
  digitalWrite(FIRE_PIN, HIGH);          // Start the pulse
  TIMSK1 &= ~_BV(OCIE1B);                // one-shot: disable B until next schedule

  // Schedule pulse OFF after fixed width
  const uint16_t width_ticks = engine.next_dwell_ticks;
  uint16_t t_off = (uint16_t)(OCR1B + width_ticks);
  schedule_A(t_off);
}

// Timer1 COMPA interrupt: end ignition pulse
ISR(TIMER1_COMPA_vect) {
  digitalWrite(FIRE_PIN, LOW);           // End the pulse
  TIMSK1 &= ~_BV(OCIE1A);                // one-shot: disable A
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Rotax 787 Ignition Controller"));

  pinMode(FIRE_PIN, OUTPUT);
  digitalWrite(FIRE_PIN, LOW);

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
  // Print diagnostics every 5 seconds
  if (millis() - sys.last_diagnostic_millis > 5000) {
    if (sys.engine_running) {
      SerialInterface::print_status();
    }
    sys.last_diagnostic_millis = millis();
  }
}
