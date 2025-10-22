#include <Arduino.h>

// AeroHalo LED bank tester
// D5, D7, D10 each driven as independent PWM outputs.
// Turns each LED bank on for 2 s, then off.

#define LED1_PIN 5   // LED bank 1 (PWM on Timer0B)
#define LED2_PIN 7   // LED bank 2 (soft-PWM or digital)
#define LED3_PIN 10  // LED bank 3 (PWM on Timer1B)

// Brightness level (0–255)
const uint8_t LED_ON = 200;   // about 80 % duty

void setup() {
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);

  // start all off
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
}

void loop() {
  // LED bank 1 test
  analogWrite(LED1_PIN, LED_ON);
  delay(2000);
  analogWrite(LED1_PIN, 0);
  delay(500);

  // LED bank 2 test
  analogWrite(LED2_PIN, LED_ON);
  delay(2000);
  analogWrite(LED2_PIN, 0);
  delay(500);

  // LED bank 3 test
  analogWrite(LED3_PIN, LED_ON);
  delay(2000);
  analogWrite(LED3_PIN, 0);
  delay(500);

  // pause before repeating
  delay(1000);
}

