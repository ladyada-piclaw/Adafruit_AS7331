/**
 * PWM Demo for Metro Mini Oscilloscope Testing
 * 
 * Generates three different PWM frequencies using direct timer register manipulation:
 *   - Timer2: ~245 Hz on pins 3 and 11
 *   - Timer1: 1000 Hz on pins 9 and 10
 *   - Timer0: 5000 Hz on pins 5 and 6
 * 
 * Each frequency pair has:
 *   - One pin with fixed 50% duty cycle (square wave reference)
 *   - One pin with sweeping duty cycle (0% → 100% → 0% over ~5 seconds)
 * 
 * WARNING: Timer0 modification breaks millis()/delay() - uses _delay_ms() instead
 * 
 * Hardware: Metro Mini (ATmega328P @ 16 MHz)
 */

#include <util/delay.h>

// Pin assignments (fixed by ATmega328P hardware)
#define PIN_245HZ_FIXED    11  // OC2A - 50% duty
#define PIN_245HZ_SWEEP    3   // OC2B - sweeping duty
#define PIN_1KHZ_FIXED     9   // OC1A - 50% duty  
#define PIN_1KHZ_SWEEP     10  // OC1B - sweeping duty
#define PIN_5KHZ_FIXED     6   // OC0A - 50% duty (toggle mode)
#define PIN_5KHZ_SWEEP     5   // OC0B - sweeping duty

// Timer1 TOP value for 1000 Hz (16-bit timer allows exact frequency)
// f = f_clk / (2 * N * TOP) for Phase Correct PWM
// 1000 = 16000000 / (2 * 1 * 8000)
#define TIMER1_TOP 8000

// Timer0 TOP value for 5000 Hz
// f = f_clk / (2 * N * TOP) for Phase Correct PWM
// 5000 = 16000000 / (2 * 8 * 200)
#define TIMER0_TOP 200

// Sweep timing
#define SWEEP_STEPS 100        // Steps in one direction
#define SWEEP_DELAY_MS 25      // Delay per step (~5 sec full cycle)

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  
  Serial.println(F("\n=== PWM Demo for Oscilloscope Testing ==="));
  Serial.println(F("Metro Mini (ATmega328P @ 16 MHz)\n"));
  
  // Configure all PWM pins as outputs
  pinMode(PIN_245HZ_FIXED, OUTPUT);
  pinMode(PIN_245HZ_SWEEP, OUTPUT);
  pinMode(PIN_1KHZ_FIXED, OUTPUT);
  pinMode(PIN_1KHZ_SWEEP, OUTPUT);
  pinMode(PIN_5KHZ_FIXED, OUTPUT);
  pinMode(PIN_5KHZ_SWEEP, OUTPUT);
  
  // ========================================
  // Timer2: ~245 Hz (pins 3, 11)
  // Phase Correct PWM, prescaler 128
  // f = 16MHz / (128 * 510) = 245.1 Hz
  // ========================================
  TCCR2A = (1 << COM2A1) |   // Clear OC2A on up-count match, set on down-count
           (1 << COM2B1) |   // Clear OC2B on up-count match, set on down-count
           (1 << WGM20);     // Phase Correct PWM, TOP = 0xFF
  TCCR2B = (1 << CS22) |     // Prescaler = 128 (CS22:CS20 = 101)
           (0 << CS21) |
           (1 << CS20);
  OCR2A = 127;               // Pin 11: 50% duty (127/255)
  OCR2B = 0;                 // Pin 3: start at 0%
  
  float freq2 = 16000000.0 / (128.0 * 510.0);
  Serial.println(F("Timer2 (~245 Hz):"));
  Serial.print(F("  Pin 11 (OC2A): 50% fixed   - "));
  Serial.print(freq2, 1);
  Serial.println(F(" Hz"));
  Serial.print(F("  Pin 3  (OC2B): sweeping    - "));
  Serial.print(freq2, 1);
  Serial.println(F(" Hz\n"));
  
  // ========================================
  // Timer1: 1000 Hz exact (pins 9, 10)
  // Phase & Freq Correct PWM, ICR1 = TOP
  // f = 16MHz / (2 * 1 * 8000) = 1000 Hz
  // ========================================
  TCCR1A = (1 << COM1A1) |   // Clear OC1A on up-count match, set on down-count
           (1 << COM1B1);    // Clear OC1B on up-count match, set on down-count
  TCCR1B = (1 << WGM13) |    // Phase & Freq Correct PWM, TOP = ICR1
           (1 << CS10);      // Prescaler = 1
  ICR1 = TIMER1_TOP;         // TOP = 8000 for 1000 Hz
  OCR1A = TIMER1_TOP / 2;    // Pin 9: 50% duty (4000/8000)
  OCR1B = 0;                 // Pin 10: start at 0%
  
  uint32_t freq1 = 16000000UL / (2UL * 1UL * TIMER1_TOP);
  Serial.println(F("Timer1 (1000 Hz):"));
  Serial.print(F("  Pin 9  (OC1A): 50% fixed   - "));
  Serial.print(freq1);
  Serial.println(F(" Hz"));
  Serial.print(F("  Pin 10 (OC1B): sweeping    - "));
  Serial.print(freq1);
  Serial.println(F(" Hz\n"));
  
  // ========================================
  // Timer0: 5000 Hz (pins 5, 6)
  // Phase Correct PWM with OCRA as TOP
  // f = 16MHz / (2 * 8 * 200) = 5000 Hz
  // WARNING: This breaks millis()/delay()!
  // ========================================
  TCCR0A = (1 << COM0A0) |   // Toggle OC0A on compare match (50% square wave)
           (1 << COM0B1) |   // Clear OC0B on up-count match, set on down-count
           (1 << WGM00);     // Phase Correct PWM (WGM = 101 with WGM02)
  TCCR0B = (1 << WGM02) |    // Phase Correct PWM, TOP = OCRA
           (1 << CS01);      // Prescaler = 8
  OCR0A = TIMER0_TOP;        // TOP = 200, Pin 6 toggles = 50% square wave
  OCR0B = 0;                 // Pin 5: start at 0%
  
  uint32_t freq0 = 16000000UL / (2UL * 8UL * TIMER0_TOP);
  Serial.println(F("Timer0 (5000 Hz):"));
  Serial.print(F("  Pin 6  (OC0A): 50% fixed   - "));
  Serial.print(freq0);
  Serial.println(F(" Hz"));
  Serial.print(F("  Pin 5  (OC0B): sweeping    - "));
  Serial.print(freq0);
  Serial.println(F(" Hz\n"));
  
  Serial.println(F("WARNING: millis()/delay() broken (Timer0 modified)"));
  Serial.println(F("Using _delay_ms() for timing.\n"));
  Serial.println(F("Sweep cycle: ~5 seconds (0% -> 100% -> 0%)"));
  Serial.println(F("Probe pins with oscilloscope to see waveforms!"));
  Serial.println(F("==========================================\n"));
}

void loop() {
  // Sweep duty cycle up: 0% -> 100%
  for (uint8_t i = 0; i <= SWEEP_STEPS; i++) {
    // Timer2 sweep (8-bit, max 255)
    OCR2B = (uint8_t)(((uint32_t)i * 255) / SWEEP_STEPS);
    
    // Timer1 sweep (16-bit, max = TIMER1_TOP)
    OCR1B = (uint16_t)(((uint32_t)i * TIMER1_TOP) / SWEEP_STEPS);
    
    // Timer0 sweep (8-bit, max = TIMER0_TOP)
    OCR0B = (uint8_t)(((uint32_t)i * TIMER0_TOP) / SWEEP_STEPS);
    
    _delay_ms(SWEEP_DELAY_MS);
  }
  
  // Sweep duty cycle down: 100% -> 0%
  for (uint8_t i = SWEEP_STEPS; i > 0; i--) {
    // Timer2 sweep
    OCR2B = (uint8_t)(((uint32_t)i * 255) / SWEEP_STEPS);
    
    // Timer1 sweep
    OCR1B = (uint16_t)(((uint32_t)i * TIMER1_TOP) / SWEEP_STEPS);
    
    // Timer0 sweep
    OCR0B = (uint8_t)(((uint32_t)i * TIMER0_TOP) / SWEEP_STEPS);
    
    _delay_ms(SWEEP_DELAY_MS);
  }
}
