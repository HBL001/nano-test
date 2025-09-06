/* ************************************************************************
 * AeroHalo PIDPWMTest Firmware
 * (c) 2025 AeroHalo - Dr. Richard Day
 *
 * Purpose:
 *   Closed-loop fan speed control using PI + feed-forward.
 *   Target RPM is hard-coded (default: 150 RPM).
 *
 * Hardware:
 *   FAN_PWM_PIN  = D9  (OC1A / Timer1, ~25 kHz PWM, active-low/open-collector)
 *   TACH_PIN     = D3  (INT1, tachometer input, PPR=2 typical)
 *
 * Features:
 *   - 25 kHz PWM generation (Intel 4-wire fan spec)
 *   - RPM measurement via tach ISR
 *   - PI + feed-forward control loop at ~5 Hz
 *   - Duty clamping, minimum-run threshold, slew rate limiting
 *   - Serial debug output: "RPM=<value>, duty=<fraction>"
 *
 * Notes:
 *   - Adjust TARGET_RPM to change the control setpoint.
 *   - Requires 12 V supply for the Arctic P9 PWM fan.
 *   - Open-collector PWM drive with pull-up to 5 V.
 ************************************************************************ */

#include <Arduino.h>

/* ===================== Pin plan ===================== */
#define FAN_PWM_PIN      9   // D9, OC1A, Timer1 ~25 kHz
#define TACH_PIN         3   // D3, INT1 (tach input)

/* ===================== User settings ===================== */
#define ACTIVE_LOW        1      // 1: inverting (low=ON, open-collector style)
#define PPR               2      // tach pulses per revolution
#define TARGET_RPM     150.0f

// Control cadence ~5 Hz
#define PID_DT_S        0.20f    // 200 ms
#define RPM_WIN_MS        200    // update window in ms

// Duty guards & shaping
#define DUTY_MIN_RUN     0.40f   // fraction (0.40 = 40%)
#define SLEW_UP_PER_S    2.0f    // duty fraction per second (200%/s)
#define SLEW_DN_PER_S    4.0f    // duty fraction per second

// Feed-forward from your measured map: duty = a*RPM + b
#define FF_A            0.001228f
#define FF_B            0.3758f

// PI gains (D=0 for low RPM)
#define KP              0.10f
#define KI              0.01f
#define I_MIN          -0.15f
#define I_MAX           0.15f

/* ===================== Tach capture state ===================== */
volatile unsigned long g_tachEdges = 0;
volatile unsigned long g_lastEdgeUs = 0;

/* ===================== ISR ===================== */
static void tachISR() {
  unsigned long now = micros();
  if (now - g_lastEdgeUs >= 50) { // deglitch: ignore pulses <50us
    g_tachEdges++;
    g_lastEdgeUs = now;
  }
}

/* ===================== RPM computation ===================== */
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
  float Hz = edges / window_s;            // edges per second
  if (PPR <= 0) return 0.0f;
  float rpm = (Hz * 60.0f) / float(PPR);  // RPM
  return rpm;
}

/* ===================== Timer1 @ 25 kHz (OC1A on D9) ===================== */
static const uint16_t PWM_TOP = 639; // 16e6 / (1*(1+639)) = 25 kHz

static void timer1_init_25kHz_pwm(void) {
  pinMode(FAN_PWM_PIN, OUTPUT);

  // Fast PWM, TOP = ICR1
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= _BV(WGM11);
  TCCR1B |= _BV(WGM13) | _BV(WGM12);

  if (ACTIVE_LOW) {
    TCCR1A |= _BV(COM1A1) | _BV(COM1A0); // inverting mode
  } else {
    TCCR1A |= _BV(COM1A1); // non-inverting
  }

  ICR1 = PWM_TOP;
  OCR1A = ACTIVE_LOW ? PWM_TOP : 0;

  TCCR1B |= _BV(CS10); // prescaler = 1
}

/* Map duty fraction (0..1) to OCR1A */
static uint16_t fracToOCR1A(float dutyFrac) {
  if (dutyFrac <= 0.0f) return ACTIVE_LOW ? PWM_TOP : 0;
  if (dutyFrac >= 1.0f) return ACTIVE_LOW ? 0 : PWM_TOP;
  uint16_t c = (uint16_t)(dutyFrac * PWM_TOP + 0.5f);
  return ACTIVE_LOW ? (PWM_TOP - c) : c;
}

/* ===================== Duty write with slew ===================== */
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

  OCR1A = fracToOCR1A(u);
  g_prevDuty = u;
}

/* ===================== PI + feed-forward ===================== */
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

/* ===================== Setup / Loop ===================== */
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n--- PIDPWMTest (Nano): D9 PWM 25kHz, D3 tach ---"));

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
    Serial.println(g_prevDuty, 3);
  }

  if (now - lastCtlMs >= (unsigned long)(PID_DT_S * 1000.0f)) {
    lastCtlMs = now;
    float u = controlStep(TARGET_RPM, (rpm >= 0.0f ? rpm : 0.0f));
    writeDutyFrac(u);
  }
}
