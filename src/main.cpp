#include <Arduino.h>

// ======================================================
// AeroHalo LED Fade Test
// Fades D5, D7, D10 sequentially
// D5 -> hardware PWM (Timer0B)
// D10 -> hardware PWM (Timer1B)
// D7 -> software PWM (Timer2 ISR)
// ======================================================

#define LED1_PIN 5    // LED Bank 1 (hardware PWM)
#define LED2_PIN 7    // LED Bank 2 (soft PWM)
#define LED3_PIN 10   // LED Bank 3 (hardware PWM)

// Fade speed (lower = slower)
const int fadeDelay = 10;   // ms between brightness steps
const int fadeSteps = 255;  // 8-bit range

// ---------- Soft PWM globals for D7 ----------
#define SOFTPWM_RES 64           // brightness steps (0..63)
volatile uint8_t d7Duty = 0;     // duty (0..63)
volatile uint8_t d7Phase = 0;

// ---------- Timer2 ISR for D7 ----------
ISR(TIMER2_COMPA_vect) {
  d7Phase++;
  if (d7Phase >= SOFTPWM_RES) d7Phase = 0;
  if (d7Phase < d7Duty)
    PORTD |=  _BV(PD7);   // D7 HIGH
  else
    PORTD &= ~_BV(PD7);   // D7 LOW
}

// ---------- Soft PWM init ----------
void d7PwmInit() {
  pinMode(LED2_PIN, OUTPUT);
  // Timer2 in CTC mode @ ~16 kHz ISR rate
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS21);      // prescaler = 8
  OCR2A  = 124;            // 16e6 / (8 * (124+1)) = 16 kHz
  TIMSK2 = _BV(OCIE2A);    // enable compare match A interrupt
}

// ---------- Set brightness for D7 (0–255) ----------
void setLed2Brightness(uint8_t level) {
  // Map 0–255 -> 0–SOFTPWM_RES
  d7Duty = (uint8_t)((uint16_t)level * SOFTPWM_RES / 255);
}

// ---------- Fade helper ----------
void fadeLED(uint8_t pin) {
  for (int i = 0; i <= fadeSteps; i++) {
    if (pin == LED2_PIN)
      setLed2Brightness(i);
    else
      analogWrite(pin, i);
    delay(fadeDelay);
  }
  for (int i = fadeSteps; i >= 0; i--) {
    if (pin == LED2_PIN)
      setLed2Brightness(i);
    else
      analogWrite(pin, i);
    delay(fadeDelay);
  }
  delay(500);
}

void setup() {
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  analogWrite(LED1_PIN, 0);
  analogWrite(LED3_PIN, 0);

  d7PwmInit();              // start soft-PWM on D7
}

void loop() {
  fadeLED(LED1_PIN);   // LED Bank 1 (D5)
  fadeLED(LED2_PIN);   // LED Bank 2 (D7 soft-PWM)
  fadeLED(LED3_PIN);   // LED Bank 3 (D10)
  delay(1000);
}
