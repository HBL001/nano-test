#include <Arduino.h>

const int LED_OUT = 9;   // PWM pin driving your MMBT3904 -> P-MOSFET
const int MAX_DUTY = 255;
const int MIN_DUTY = 1;

// Half-cycle = 2.5 s (up or down). Full cycle = 5.0 s
const unsigned long HALF_MS = 2500;
const unsigned long STEP_DELAY_MS = HALF_MS / MAX_DUTY; // ≈ 9 ms per step

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_OUT, OUTPUT);

  // Optional: boost PWM freq on D9/D10 to ~31 kHz (quieter on cameras)
  // TCCR1B = (TCCR1B & 0b11111000) | 0x01;
}

void loop() {
  // Ramp UP: 0 -> 255 over ~2.5 s
  digitalWrite(LED_BUILTIN, HIGH);
  for (int d = MIN_DUTY; d <= MAX_DUTY; d++) {
    analogWrite(LED_OUT, d);
    delay(STEP_DELAY_MS);
  }

  // Ramp DOWN: 255 -> 0 over ~2.5 s
  digitalWrite(LED_BUILTIN, LOW);
  for (int d = MAX_DUTY; d >= MIN_DUTY; d--) {
    analogWrite(LED_OUT, d);
    delay(STEP_DELAY_MS);
  }
}
