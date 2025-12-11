// =============================================================================
// FINAL PRODUCTION CODE: Single-Phase Unipolar SPWM Inverter
// Hardware: Arduino Nano/Uno + IR2112 + IRF540N
// Switching Freq: ~31.25 kHz (Timer1) and ~62.5 kHz (Timer0)
// Output: 50Hz Pure Sine Wave (after LC Filter)
// =============================================================================

#include <avr/io.h>
#include <avr/interrupt.h>

// --- PIN DEFINITIONS ---
#define PIN_HIA   9   // Leg A High (PWM)
#define PIN_LOA   8   // Leg A Low  (Digital)
#define PIN_HIB   6   // Leg B High (PWM)
#define PIN_LOB   5   // Leg B Low  (Digital)

// Direct Port Access (for speed)
#define HIA_BIT   1   // PB1
#define LOA_BIT   0   // PB0
#define HIB_BIT   6   // PD6
#define LOB_BIT   5   // PD5

// --- CONFIGURATION ---
#define DEADTIME_US   3      // 3us Deadtime at crossover
#define OUTPUT_FREQ   50     // Target Hz
#define SINE_SAMPLES  200    // Table size
#define UPDATE_DIV    3      // ISR Divider (31kHz / 3 / 200 ≈ 52Hz)

// --- VARIABLES ---
volatile uint8_t sineIndex = 0;
volatile bool positiveHalf = true;
volatile bool inverterEnabled = true;
volatile uint8_t updateCounter = 0;

// Replacement for millis() logic
volatile uint16_t loopTick = 0; 

// Sine Table (0-511 scale for 9-bit PWM)
const uint16_t sineTable[SINE_SAMPLES] = {
  0, 15, 30, 46, 61, 76, 91, 106, 121, 136, 151, 165, 180, 194, 208, 222, 236, 249, 263, 276,
  289, 301, 314, 326, 337, 349, 360, 371, 381, 391, 401, 410, 419, 428, 436, 444, 451, 458, 464, 470,
  476, 481, 485, 489, 493, 496, 499, 501, 503, 504, 505, 505, 504, 503, 501, 499, 496, 493, 489, 485,
  481, 476, 470, 464, 458, 451, 444, 436, 428, 419, 410, 401, 391, 381, 371, 360, 349, 337, 326, 314,
  301, 289, 276, 263, 249, 236, 222, 208, 194, 180, 165, 151, 136, 121, 106, 91, 76, 61, 46, 30,
  // Duplicate for 2nd half (logic handles polarity)
  0, 15, 30, 46, 61, 76, 91, 106, 121, 136, 151, 165, 180, 194, 208, 222, 236, 249, 263, 276,
  289, 301, 314, 326, 337, 349, 360, 371, 381, 391, 401, 410, 419, 428, 436, 444, 451, 458, 464, 470,
  476, 481, 485, 489, 493, 496, 499, 501, 503, 504, 505, 505, 504, 503, 501, 499, 496, 493, 489, 485,
  481, 476, 470, 464, 458, 451, 444, 436, 428, 419, 410, 401, 391, 381, 371, 360, 349, 337, 326, 314,
  301, 289, 276, 263, 249, 236, 222, 208, 194, 180, 165, 151, 136, 121, 106, 91, 76, 61, 46, 30
};

// --- HELPER FUNCTIONS ---
static inline void applyDeadtime() {
  delayMicroseconds(DEADTIME_US);
}

void safeShutdown() {
  cli();
  // Disable PWM Output
  TCCR1A &= ~_BV(COM1A1);
  TCCR0A &= ~_BV(COM0A1);
  OCR1A = 0; OCR0A = 0;
  // Force Digital Low
  PORTB &= ~(_BV(HIA_BIT) | _BV(LOA_BIT));
  PORTD &= ~(_BV(HIB_BIT) | _BV(LOB_BIT));
  inverterEnabled = false;
  sei();
  Serial.println(F("\n!!! EMERGENCY SHUTDOWN !!!"));
}

void setupPWM() {
  pinMode(PIN_HIA, OUTPUT); pinMode(PIN_LOA, OUTPUT);
  pinMode(PIN_HIB, OUTPUT); pinMode(PIN_LOB, OUTPUT);
  
  // Start OFF
  digitalWrite(PIN_HIA, LOW); digitalWrite(PIN_LOA, LOW);
  digitalWrite(PIN_HIB, LOW); digitalWrite(PIN_LOB, LOW);

  // Timer1 (Leg A): Fast PWM 9-bit, No Prescaler (~31.25 kHz)
  TCCR1A = _BV(COM1A1) | _BV(WGM11); 
  TCCR1B = _BV(WGM12) | _BV(CS10); 
  OCR1A = 0;

  // Timer0 (Leg B): Fast PWM 8-bit, No Prescaler (~62.5 kHz)
  // WARNING: This breaks millis() and delay()!
  TCCR0A = _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);
  TCCR0B = _BV(CS00); 
  OCR0A = 0;

  TIMSK1 = _BV(TOIE1); // Enable Interrupt
}

// --- INTERRUPT SERVICE ROUTINE (THE ENGINE) ---
ISR(TIMER1_OVF_vect) {
  // 1. Timekeeping replacement (Counts 31,250 times/sec)
  loopTick++; 

  if (!inverterEnabled) return;

  // 2. Frequency Divider
  updateCounter++;
  if (updateCounter < UPDATE_DIV) return;
  updateCounter = 0;

  // 3. Fetch Duty Cycle
  uint16_t duty = sineTable[sineIndex];
  uint8_t duty8bit = duty >> 1; // Scale 9-bit to 8-bit for Timer0

  // 4. Switching Logic
  if (sineIndex < 100) { // === POSITIVE HALF ===
    if (!positiveHalf) {
      // Transition Neg -> Pos
      positiveHalf = true;
      OCR0A = 0;                // Kill Leg B High
      PORTB &= ~_BV(LOA_BIT);   // Kill Leg A Low
      applyDeadtime();          // Wait
      PORTD |= _BV(LOB_BIT);    // Clamp Leg B Low
    }
    OCR1A = duty; // PWM on Leg A High

  } else { // === NEGATIVE HALF ===
    if (positiveHalf) {
      // Transition Pos -> Neg
      positiveHalf = false;
      OCR1A = 0;                // Kill Leg A High
      PORTD &= ~_BV(LOB_BIT);   // Kill Leg B Low
      applyDeadtime();          // Wait
      PORTB |= _BV(LOA_BIT);    // Clamp Leg A Low
    }
    OCR0A = duty8bit; // PWM on Leg B High
  }

  // 5. Advance Index
  sineIndex++;
  if (sineIndex >= SINE_SAMPLES) sineIndex = 0;
}

void setup() {
  cli();
  setupPWM();
  sei();
  Serial.begin(115200);
  Serial.println(F("SPWM Inverter Ready."));
  Serial.println(F("Leg A: 31kHz | Leg B: 62kHz"));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') safeShutdown();
    if (c == 'r') {
      // Reset logic
      cli();
      inverterEnabled = true;
      sineIndex = 0;
      TCCR1A |= _BV(COM1A1);
      TCCR0A |= _BV(COM0A1);
      sei();
      Serial.println(F("Restarted."));
    }
  }

  // Non-blocking status print using loopTick
  // 31250 ticks ~= 1 second
  noInterrupts();
  uint16_t currentTick = loopTick;
  interrupts();

  if (currentTick >= 31250) {
    noInterrupts(); loopTick = 0; interrupts(); // Reset
    Serial.print(F("Status: "));
    Serial.println(inverterEnabled ? F("RUNNING") : F("HALTED"));
  }
}
