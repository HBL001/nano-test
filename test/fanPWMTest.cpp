/* ************************************************************************
 * AeroHalo Test Software
 * (c) 2023 AeroHalo - Dr. Richard Day
 *
 * Pin usage:
 *   FAN_PWM_PIN = D11 (PB3 / OC2A): PWM output (Timer2 Fast PWM 8-bit, ~62.5 kHz)
 *   LED_BLINK_PIN = D5 (PD5): activity blink
 *
 * Tests out the PWM output for the FAN.
 * Cycles duty steps on FAN_PWM_PIN: 0%, 25%, 50%, 75%, 100%
 * Each step held ~5 seconds.
 * Prints duty cycle and expected average voltage to Serial.
 */

#include <Arduino.h>

#define FAN_PWM_PIN    11    // PB3 / OC2A
#define LED_BLINK_PIN   5    // PD5

#define DUTY_STEPS_N 5
static const uint8_t duty_steps[DUTY_STEPS_N] = { 0, 64, 128, 192, 255 };

static void timer2_init_fast_pwm_8bit_prescale_1(void) {
  // FAN pin = D11 = PB3 = OC2A; set as output
  DDRB |= _BV(DDB3);

  // Fast PWM 8-bit on Timer2: WGM22:0 = 0b011 (WGM21=1, WGM20=1)
  // Inverting on OC2A: COM2A1=1, COM2A0=1  (matches previous inverted wiring)
  // If you want non-inverting: (1<<COM2A1) only, remove COM2A0.
  TCCR2A = (1<<COM2A1) | (1<<COM2A0) | (1<<WGM21) | (1<<WGM20);

  // Clock prescaler = 1: CS20=1  -> 16 MHz / 256 ≈ 62.5 kHz PWM
  TCCR2B = (1<<CS20);

  // Start with 0% duty
  OCR2A = 0;
}

void setup(void) {
  // LED pin as output
  pinMode(LED_BLINK_PIN, OUTPUT);
  digitalWrite(LED_BLINK_PIN, LOW);

  // Serial monitor
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n--- FAN PWM static-step test ---"));
  Serial.println(F("Steps: 0, 64, 128, 192, 255 (= 0%, 25%, 50%, 75%, 100%)"));
  Serial.println(F("Multimeter should read average voltage ≈ 5.0 * duty/255 V"));

  // Configure Timer2 for ~62.5 kHz PWM on FAN_PWM_PIN (OC2A)
  timer2_init_fast_pwm_8bit_prescale_1();
}

void loop(void) {
  for (uint8_t i = 0; i < DUTY_STEPS_N; ++i) {
    // Set PWM duty on FAN_PWM_PIN (D11)
    OCR2A = duty_steps[i];

    // Blink LED briefly
    digitalWrite(LED_BLINK_PIN, HIGH);
    delay(10);
    digitalWrite(LED_BLINK_PIN, LOW);

    // Print status
    float duty_percent = (100.0 * duty_steps[i]) / 255.0;
    float v_avg = 5.0 * duty_steps[i] / 255.0;
    Serial.print(F("Step "));
    Serial.print(i + 1);
    Serial.print(F(": duty= "));
    Serial.print(duty_steps[i]);
    Serial.print(F(" ("));
    Serial.print(duty_percent, 1);
    Serial.print(F("%), Vavg= "));
    Serial.print(v_avg, 2);
    Serial.println(F(" V"));

    // Hold this step ~5s
    delay(4900);
  }
}
