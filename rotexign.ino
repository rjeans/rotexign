/*
  
*/

#include <avr/interrupt.h>

struct EngineState {

  volatile  uint16_t last_interrupt_ticks=0;
  volatile  uint16_t this_interrupt_ticks=0;
  volatile  uint16_t period_ticks=0;
  volatile  uint16_t n_ticks=0;
  volatile  uint16_t next_dwell_ticks=0;


};

static EngineState engine;

struct SystemState {
  volatile uint32_t last_diagnostic_millis=0;
  volatile bool engine_running=false;
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
  const uint8_t  PULSES_PER_REVOLUTION=2;
  
  
  
  

 
static const uint16_t timing_rpm_curve[][2] PROGMEM = {
    {0, 0},  // 0 RPM -> 0.0°
    {100, 5},  // 100 RPM -> 0.5°
    {200, 10},  // 200 RPM -> 1.0°
    {300, 16},  // 300 RPM -> 1.6°
    {400, 22},  // 400 RPM -> 2.2°
    {500, 28},  // 500 RPM -> 2.8°
    {600, 34},  // 600 RPM -> 3.4°
    {700, 40},  // 700 RPM -> 4.0°
    {800, 47},  // 800 RPM -> 4.7°
    {900, 53},  // 900 RPM -> 5.3°
    {1000, 60},  // 1000 RPM -> 6.0°
    {1100, 67},  // 1100 RPM -> 6.7°
    {1200, 73},  // 1200 RPM -> 7.3°
    {1300, 80},  // 1300 RPM -> 8.0°
    {1400, 86},  // 1400 RPM -> 8.6°
    {1500, 92},  // 1500 RPM -> 9.2°
    {1600, 98},  // 1600 RPM -> 9.8°
    {1700, 104},  // 1700 RPM -> 10.4°
    {1800, 110},  // 1800 RPM -> 11.0°
    {1900, 115},  // 1900 RPM -> 11.5°
    {2000, 120},  // 2000 RPM -> 12.0°
    {2100, 125},  // 2100 RPM -> 12.5°
    {2200, 129},  // 2200 RPM -> 12.9°
    {2300, 133},  // 2300 RPM -> 13.3°
    {2400, 136},  // 2400 RPM -> 13.6°
    {2500, 139},  // 2500 RPM -> 13.9°
    {2600, 142},  // 2600 RPM -> 14.2°
    {2700, 145},  // 2700 RPM -> 14.5°
    {2800, 147},  // 2800 RPM -> 14.7°
    {2900, 149},  // 2900 RPM -> 14.9°
    {3000, 150},  // 3000 RPM -> 15.0°
    {3100, 151},  // 3100 RPM -> 15.1°
    {3200, 152},  // 3200 RPM -> 15.2°
    {3300, 152},  // 3300 RPM -> 15.2°
    {3400, 153},  // 3400 RPM -> 15.3°
    {3500, 153},  // 3500 RPM -> 15.3°
    {3600, 152},  // 3600 RPM -> 15.2°
    {3700, 152},  // 3700 RPM -> 15.2°
    {3800, 151},  // 3800 RPM -> 15.1°
    {3900, 151},  // 3900 RPM -> 15.1°
    {4000, 150},  // 4000 RPM -> 15.0°
    {4100, 149},  // 4100 RPM -> 14.9°
    {4200, 148},  // 4200 RPM -> 14.8°
    {4300, 147},  // 4300 RPM -> 14.7°
    {4400, 146},  // 4400 RPM -> 14.6°
    {4500, 145},  // 4500 RPM -> 14.5°
    {4600, 144},  // 4600 RPM -> 14.4°
    {4700, 143},  // 4700 RPM -> 14.3°
    {4800, 142},  // 4800 RPM -> 14.2°
    {4900, 141},  // 4900 RPM -> 14.1°
    {5000, 140},  // 5000 RPM -> 14.0°
    {5100, 139},  // 5100 RPM -> 13.9°
    {5200, 138},  // 5200 RPM -> 13.8°
    {5300, 137},  // 5300 RPM -> 13.7°
    {5400, 136},  // 5400 RPM -> 13.6°
    {5500, 135},  // 5500 RPM -> 13.5°
    {5600, 134},  // 5600 RPM -> 13.4°
    {5700, 133},  // 5700 RPM -> 13.3°
    {5800, 132},  // 5800 RPM -> 13.2°
    {5900, 131},  // 5900 RPM -> 13.1°
    {6000, 130},  // 6000 RPM -> 13.0°
    {6100, 129},  // 6100 RPM -> 12.9°
    {6200, 128},  // 6200 RPM -> 12.8°
    {6300, 127},  // 6300 RPM -> 12.7°
    {6400, 126},  // 6400 RPM -> 12.6°
    {6500, 125},  // 6500 RPM -> 12.5°
    {6600, 124},  // 6600 RPM -> 12.4°
    {6700, 123},  // 6700 RPM -> 12.3°
    {6800, 122},  // 6800 RPM -> 12.2°
    {6900, 121},  // 6900 RPM -> 12.1°
    {7000, 120},  // 7000 RPM -> 12.0°
    {7100, 119},  // 7100 RPM -> 11.9°
    {7200, 118},  // 7200 RPM -> 11.8°
    {7300, 117},  // 7300 RPM -> 11.7°
    {7400, 116},  // 7400 RPM -> 11.6°
    {7500, 115},  // 7500 RPM -> 11.5°
    {7600, 114},  // 7600 RPM -> 11.4°
    {7700, 113},  // 7700 RPM -> 11.3°
    {7800, 112},  // 7800 RPM -> 11.2°
    {7900, 111},  // 7900 RPM -> 11.1°
    {8000, 110}   // 8000 RPM -> 11.0°
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
//
// Must avoid overflow
//
  uint16_t i = (uint32_t(rpm) * (TIMING_RPM_POINTS-1) + MAX_TIMING_RPM -1)/MAX_TIMING_RPM;

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




    // Calculate spark delay in ticks from RPM using advance angle and 47° BTDC trigger
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
    
    // Convert to ticks (4μs per tick with /64 prescaler)
    return delay_us;
  }

  uint16_t get_dwell_delay_us_from_rpm(uint16_t rpm) {
     
    uint16_t spark_delay_us = get_spark_delay_us_from_rpm(rpm);

    uint32_t period_us = 60000000UL / (rpm * PULSES_PER_REVOLUTION);

    uint32_t max_dwell_us = ((period_us + 50 )/ 100) * MAX_DUTY_CYCLE;

    uint16_t dwell_us = min(NOMINAL_DWELL_US , max_dwell_us);

    boolean skip_period =  (dwell_us + DWELL_TO_TRIGGER_MARGIN_US) > spark_delay_us;

    uint16_t dwell_delay_us;
    
    if (skip_period) {
      
      dwell_delay_us = (period_us + spark_delay_us) - dwell_us;

 
    } else {
      dwell_delay_us = spark_delay_us - dwell_us;
    }


 
    return dwell_delay_us;


  }

    uint16_t get_dwell_us_from_rpm(uint16_t rpm) {
     
    uint16_t spark_delay_us = get_spark_delay_us_from_rpm(rpm);

    uint32_t period_us = 60000000UL / (rpm * PULSES_PER_REVOLUTION);

    uint32_t max_dwell_us = ((period_us + 50 )/ 100) * MAX_DUTY_CYCLE;

    uint16_t dwell_us = min(NOMINAL_DWELL_US , max_dwell_us);

    

 
    return dwell_us;


  }


}


namespace SerialInterface {
  void print_status() {

    uint16_t period_ticks = engine.period_ticks;
    uint32_t period_us = Utils::ticks64_to_us(period_ticks); 

    uint16_t rpm= 60000000 / (period_us * Timing::PULSES_PER_REVOLUTION);




    Serial.print(F("RPM: ")); Serial.print(rpm);
    Serial.print(F(" Advance angle: ")); Serial.print(Timing::get_advance_angle_tenths(rpm));
    Serial.print(F(" Spark delay (us): ")); Serial.print(Timing::get_spark_delay_us_from_rpm(rpm));
    Serial.print(F(" Dwell delay (us): ")); Serial.print(Timing::get_dwell_delay_us_from_rpm(rpm));
    Serial.print(F(" Dwell (us): ")); Serial.print(Timing::get_dwell_us_from_rpm(rpm));
    Serial.println("");

    Serial.print(F("RPM: ")); Serial.print(rpm);
    Serial.print(F(" Dwell delay (ticks): ")); Serial.print(Utils::us_to_ticks64(Timing::get_dwell_delay_us_from_rpm(rpm)));
    Serial.print(F(" Dwell (ticks): ")); Serial.print(Utils::us_to_ticks64(Timing::get_dwell_us_from_rpm(rpm)));
    Serial.println("");

    

  }

}



constexpr uint8_t FIRE_PIN = 3;          // your output pin (any GPIO)
constexpr uint16_t PRESCALE_BITS = _BV(CS11) | _BV(CS10); // /64 -> 4us/tick


// Schedule a COMPB one-shot at absolute tick 'when'
static inline void schedule_B(uint16_t when) {
//  Serial.print(TCNT1); Serial.print("--");
//  Serial.println(when);
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

ISR(INT0_vect) {


  if (!engine.n_ticks++) {
    engine.last_interrupt_ticks = TCNT1;

  } else {
    sys.engine_running = true;
    engine.last_interrupt_ticks = engine.this_interrupt_ticks;
    engine.this_interrupt_ticks = TCNT1;
    engine.period_ticks = (uint16_t)(engine.this_interrupt_ticks - engine.last_interrupt_ticks);

    // Fast lookup using pre-calculated table - no expensive calculations in interrupt!
//    Timing::TimingEntry timing = Timing::fast_timing_lookup(engine.period_ticks);
    
//    engine.next_dwell_ticks = timing.dwell_ticks;
//    schedule_B(engine.this_interrupt_ticks + timing.dwell_delay_ticks);
  
    uint32_t period_us = Utils::ticks64_to_us(engine.period_ticks); 
    uint16_t rpm= 60000000 / (period_us * Timing::PULSES_PER_REVOLUTION);

  
    uint16_t dwell_ticks= Utils::us_to_ticks64(Timing::get_dwell_us_from_rpm(rpm));
   

    uint16_t dwell_delay_ticks = Utils::us_to_ticks64(Timing::get_dwell_delay_us_from_rpm(rpm));

     

   engine.next_dwell_ticks = dwell_ticks;
     schedule_B(engine.this_interrupt_ticks+dwell_delay_ticks);
   }
}
 


ISR(TIMER1_COMPB_vect) {

  // Start the pulse
  digitalWrite(FIRE_PIN, HIGH);          // (or direct port write for speed)
  TIMSK1 &= ~_BV(OCIE1B);                // one-shot: disable B until next schedule

  // Schedule pulse OFF after fixed width
  const uint16_t width_ticks = engine.next_dwell_ticks;
  uint16_t t_off = (uint16_t)(OCR1B + width_ticks);
  schedule_A(t_off);
}

ISR(TIMER1_COMPA_vect) {
  // End the pulse
  digitalWrite(FIRE_PIN, LOW);
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
