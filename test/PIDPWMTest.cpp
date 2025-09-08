#include <Arduino.h>

/* ************************************************************************
 * AeroHalo PIDPWMTest Firmware  (fix: avoid OCR1A = 0 or TOP)
 * D9 = OC1A (Timer1 @ 25 kHz), D3 = tach
 ************************************************************************ */

#define FAN_PWM_PIN      9   // D9, OC1A
#define TACH_PIN         3   // D3, INT1
#define ACTIVE_LOW        1   // 1: inverting for open-collector (fan PWM active-low)
#define PPR               2
#define TARGET_RPM     150.0f

#define PID_DT_S        0.20f
#define RPM_WIN_MS        200

#define DUTY_MIN_RUN     0.40f
#define SLEW_UP_PER_S    2.0f
#define SLEW_DN_PER_S    4.0f

#define FF_A            0.001228f
#define FF_B            0.3758f

#define KP              0.10f
#define KI              0.01f
#define I_MIN          -0.15f
#define I_MAX           0.15f

volatile unsigned long g_tachEdges = 0;
volatile unsigned long g_lastEdgeUs = 0;

static void tachISR() {
  unsigned long now = micros();
  if (now - g_lastEdgeUs >= 50) {
    g_tachEdges++;
    g_lastEdgeUs = now;
  }
}

static float rpmComputeWindowed() {
  static unsigned long lastMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastMs < RPM_WIN_MS) return -1.0f;

  noInterrupts();
  unsigned long edges = g_tachEdges;
  g_tachEdges = 0;
  interrupts();

  lastMs = nowMs;

  float window_s = RPM_WIN_MS / 1000.0f;
  float Hz = edges / window_s;
  if (PPR <= 0) return 0.0f;
  return (Hz * 60.0f) / float(PPR);
}

/* ===== Timer1 @ 25 kHz (OC1A on D9) ===== */
static const uint16_t PWM_TOP = 639; // 16MHz/(1*(1+639)) = 25kHz

static inline uint16_t clamp16(uint16_t x, uint16_t lo, uint16_t hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

static void timer1_init_25kHz_pwm(void) {
  pinMode(FAN_PWM_PIN, OUTPUT);   // PB1 as output

  // Fast PWM, mode 14: TOP=ICR1
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= _BV(WGM11);
  TCCR1B |= _BV(WGM13) | _BV(WGM12);

  if (ACTIVE_LOW) {
    TCCR1A |= _BV(COM1A1) | _BV(COM1A0);  // inverting
  } else {
    TCCR1A |= _BV(COM1A1);                // non-inverting
  }

  ICR1 = PWM_TOP;

  // Start at true 0% duty **without** writing exact 0 or TOP
  // In inverting mode, 0% (fan fully off) means OCR1A≈TOP; use TOP-1.
  // In non-inverting, 0% means OCR1A≈0; use 1.
  OCR1A = ACTIVE_LOW ? (PWM_TOP - 1) : 1;

  // clk/1
  TCCR1B |= _BV(CS10);
}

/* Map 0..1 duty fraction to OCR1A, avoiding 0 and TOP */
static uint16_t fracToOCR1A(float dutyFrac) {
  if (dutyFrac <= 0.0f) {
    return ACTIVE_LOW ? (PWM_TOP - 1) : 1;
  }
  if (dutyFrac >= 1.0f) {
    return ACTIVE_LOW ? 1 : (PWM_TOP - 1);
  }

  // Scale to ticks
  float ticksF = dutyFrac * PWM_TOP;
  uint16_t ticks = (uint16_t)(ticksF + 0.5f);

  // Convert for inverting vs non-inverting
  uint16_t ocr = ACTIVE_LOW ? (PWM_TOP - ticks) : ticks;

  // HARD CLAMP to [1 .. TOP-1]
  return clamp16(ocr, 1, PWM_TOP - 1);
}

/* ===== Duty write with slew ===== */
static float g_prevDuty = 0.0f;
static unsigned long g_lastDutyMs = 0;

static void writeDutyFrac(float target) {
  if (target > 0.0f && target < DUTY_MIN_RUN) target = DUTY_MIN_RUN;
  if (target < 0.0f) target = 0.0f;
  if (target > 1.0f) target = 1.0f;

  unsigned long now = millis();
  float dt = (now - g_lastDutyMs) / 1000.0f;
  if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;
  g_lastDutyMs = now;

  float maxUp = (SLEW_UP_PER_S > 0.0f) ? (SLEW_UP_PER_S * dt) : 1e9f;
  float maxDn = (SLEW_DN_PER_S > 0.0f) ? (SLEW_DN_PER_S * dt) : 1e9f;

  float u = target;
  if (u > g_prevDuty + maxUp) u = g_prevDuty + maxUp;
  if (u < g_prevDuty - maxDn) u = g_prevDuty - maxDn;

  uint16_t ocr = fracToOCR1A(u);
  OCR1A = ocr;
  g_prevDuty = u;
}

/* ===== PI + feed-forward ===== */
static float g_integrator = 0.0f;

static float controlStep(float setpointRPM, float measuredRPM) {
  float duty_ff = FF_A * setpointRPM + FF_B;
  float e = setpointRPM - measuredRPM;

  g_integrator += (KI * e * PID_DT_S);
  if (g_integrator > I_MAX) g_integrator = I_MAX;
  if (g_integrator < I_MIN) g_integrator = I_MIN;

  float u = duty_ff + (KP * e) + g_integrator;
  if (u < 0.0f) u = 0.0f;
  if (u > 1.0f) u = 1.0f;
  return u;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n--- PIDPWMTest (Nano): D9 PWM 25kHz, D3 tach (edge-safe) ---"));

  timer1_init_25kHz_pwm();

  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachISR, RISING);

  g_lastDutyMs = millis();
}

void loop() {
  static unsigned long lastCtlMs = 0;
  unsigned long now = millis();

  float rpm = rpmComputeWindowed();
  if (rpm >= 0.0f) {
    Serial.print(F("RPM="));
    Serial.print(rpm, 0);
    Serial.print(F(", duty="));
    Serial.print(g_prevDuty, 3);
    Serial.print(F(", OCR1A="));
    Serial.print(OCR1A);
    Serial.print(F("/"));
    Serial.println(PWM_TOP);
  }

  if (now - lastCtlMs >= (unsigned long)(PID_DT_S * 1000.0f)) {
    lastCtlMs = now;
    float u = controlStep(TARGET_RPM, (rpm >= 0.0f ? rpm : 0.0f));
    writeDutyFrac(u);
  }
}
