#include <Arduino.h>

/* ************************************************************************
 * AeroHalo Fan PWM step test (Timer1 @ 25 kHz)
 * D9 = OC1A (PB1): fan PWM
 * D5 = LED blink
 * Duty steps: 0, 64, 128, 192, 255
 * ************************************************************************/

#define FAN_PWM_PIN     9    // D9 = PB1 / OC1A
#define LED_BLINK_PIN   5    // D5

// Set to true if your fan expects active-low PWM (Intel 4-wire spec)
static const bool PWM_INVERTED = true;

// 25 kHz using Fast PWM, TOP = ICR1 = 639 (F_CPU = 16 MHz, prescaler = 1)
static const uint16_t PWM_TOP = 639;

#define DUTY_STEPS_N 5
static const uint8_t duty_steps[DUTY_STEPS_N] = { 0, 64, 128, 192, 255 };

// Initialize Timer1 for 25 kHz on OC1A (D9)
static void timer1_init_25k_oc1a(bool inverted) {
  // Set D9 (PB1/OC1A) as output
  DDRB |= _BV(DDB1);

  // Fast PWM, mode 14: WGM13:0 = 1110 (TOP = ICR1)
  TCCR1A = 0;
  TCCR1B = 0;
  // Compare output: OC1A enabled, inverting or non-inverting
  if (inverted) {
    // Inverting: COM1A1=1, COM1A0=1  (low-time proportional to OCR1A)
    TCCR1A |= _BV(COM1A1) | _BV(COM1A0);
  } else {
    // Non-inverting: COM1A1=1, COM1A0=0 (high-time proportional to OCR1A)
    TCCR1A |= _BV(COM1A1);
  }
  // WGM bits
  TCCR1A |= _BV(WGM11);
  TCCR1B |= _BV(WGM13) | _BV(WGM12);

  // No prescale (clk/1)
  TCCR1B |= _BV(CS10);

  // Set TOP for 25 kHz
  ICR1 = PWM_TOP;

  // Start with 0% duty
  OCR1A = 0;
}

// Set duty using 0..255 scale (like analogWrite), mapped to 0..PWM_TOP
static void set_pwm_0_255(uint8_t duty) {
  // Map 0..255 -> 0..PWM_TOP
  uint16_t ticks = (uint32_t)duty * (PWM_TOP + 1) / 255u;
  if (ticks > PWM_TOP) ticks = PWM_TOP;
  OCR1A = ticks;
}

void setup() {
  pinMode(LED_BLINK_PIN, OUTPUT);
  digitalWrite(LED_BLINK_PIN, LOW);

  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n--- FAN PWM static-step test on D9 (Timer1 @ 25 kHz) ---"));
  Serial.println(F("Steps: 0, 64, 128, 192, 255"));
  Serial.println(F("Multimeter should read average voltage ≈ 5.0 * duty/255 V"));

  timer1_init_25k_oc1a(PWM_INVERTED);
}

void loop() {
  for (uint8_t i = 0; i < DUTY_STEPS_N; ++i) {
    set_pwm_0_255(duty_steps[i]);

    // Blink LED briefly
    digitalWrite(LED_BLINK_PIN, HIGH);
    delay(10);
    digitalWrite(LED_BLINK_PIN, LOW);

    // Print status
    float duty_percent = (100.0 * duty_steps[i]) / 255.0;
    float v_avg = 5.0 * duty_steps[i] / 255.0;
    Serial.print(F("Step "));
    Serial.print(i + 1);
    Serial.print(F(": duty_raw="));
    Serial.print(duty_steps[i]);
    Serial.print(F(" ("));
    Serial.print(duty_percent, 1);
    Serial.print(F("% of scale), Timer1 OCR1A="));
    Serial.print(OCR1A);
    Serial.print(F("/"));
    Serial.print(PWM_TOP);
    Serial.print(F("), "));
    Serial.print(F("Vavg= "));
    Serial.print(v_avg, 2);
    Serial.println(F(" V"));

    delay(4900); // ~5 s
  }
}
