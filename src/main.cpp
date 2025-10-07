#include <Arduino.h>
#include <math.h>

/* ========================================================================
 * AeroHalo Fan Control — SIMPLE PID + PERIOD-BASED RPM (Timer1 25 kHz on D9)
 *
 * Pins:
 *   D9  -> OC1A 25 kHz PWM (to your fan driver)     [PWM_INVERTED=true by default]
 *   D3  -> INT1 tach input (open-collector; 10k pull-up to 5V)  [uses ISR period]
 *   D5  -> blink when |RPM - TARGET_RPM| <= 2% (checked every UI_MS)
 *
 * Tuning (simple, duty-per-RPM units):
 *   KP_DUTY_PER_RPM      duty change per RPM of error        (e.g. 0.0020)
 *   KI_DUTY_PER_RPM_S    duty per RPM per second (integral)  (e.g. 0.0006)
 *   KD_DUTY_PER_RPM_S    duty*second per RPM (derivative)    (start at 0.0)
 *
 * Notes:
 *   - If “higher duty” makes the fan slower, flip PWM_INVERTED.
 *   - Conditional integration prevents windup at 0% / 100% duty.
 *   - Period-based RPM avoids window/phase aliasing at low speeds.
 * ======================================================================== */

#define FAN_PWM_PIN      9
#define TACH_PIN         3
#define LED_BLINK_PIN    5

static const bool     PWM_INVERTED = true;   // matches your working step-test
static const uint16_t PWM_TOP      = 639;    // 25 kHz @ 16 MHz, prescale=1

/* -------- Tach / target / cadence -------- */
#define PPR                2            // pulses per revolution
#define TARGET_RPM     64.0f
#define PID_DT_S        0.20f          // controller step = 5 Hz
#define UI_MS            300           // print/blink cadence

/* -------- Blink band -------- */
#define RPM_TOL_PCT       0.1f        // ±10%
#define BLINK_MS            50

/* -------- Simple PID gains (duty-per-RPM units) -------- */
#define KP_DUTY_PER_RPM      0.005f   // 100 RPM error -> 0.010 duty change
#define KI_DUTY_PER_RPM_S    0.002f   // integral strength (start low)
#define KD_DUTY_PER_RPM_S    0.0005f   // keep 0.0 initially

/* -------- Duty clamps (keep simple) -------- */
#define DUTY_MIN_FRAC        0.36f     // set >0 if you need a spin guarantee
#define DUTY_MAX_FRAC        1.000f

/* -------- Tach period capture (interrupt-driven) -------- */
#define TACH_DEGLITCH_US     50        // ignore edges closer than this
#define RPM_TIMEOUT_MS       1200      // no edge for this long => RPM = 0

volatile uint32_t g_lastEdgeUs = 0;    // last edge timestamp (micros)
volatile uint32_t g_periodUs   = 0;    // most recent edge->edge period (micros)
volatile uint32_t g_lastEdgeMs = 0;    // millis of last valid edge

static void tachISR() {
  uint32_t nowUs = micros();
  uint32_t dt    = nowUs - g_lastEdgeUs;
  if (dt >= TACH_DEGLITCH_US) {
    g_periodUs   = dt;
    g_lastEdgeUs = nowUs;
    g_lastEdgeMs = millis();
  }
}

// Convert latest captured period to RPM. Returns 0 on timeout.
static float rpmReadPeriodic() {
  noInterrupts();
  uint32_t periodUs = g_periodUs;
  uint32_t ageMs    = millis() - g_lastEdgeMs;
  interrupts();

  if (periodUs == 0 || ageMs > RPM_TIMEOUT_MS) return 0.0f;
  // RPM = (1e6/periodUs)/PPR * 60
  return (60.0f * 1000000.0f) / (float(periodUs) * float(PPR));
}

/* ------------------ Timer1 @ 25 kHz on D9 ------------------ */
static void timer1_init_25k_oc1a(bool inverted) {
  DDRB |= _BV(DDB1);                // D9 as output

  TCCR1A = 0;
  TCCR1B = 0;

  if (inverted) {
    TCCR1A |= _BV(COM1A1) | _BV(COM1A0);   // inverting
  } else {
    TCCR1A |= _BV(COM1A1);                 // non-inverting
  }

  TCCR1A |= _BV(WGM11);
  TCCR1B |= _BV(WGM13) | _BV(WGM12);       // Fast PWM, TOP = ICR1 (mode 14)
  TCCR1B |= _BV(CS10);                     // clk/1

  ICR1  = PWM_TOP;
  OCR1A = 1;                               // avoid exact 0/TOP
}

static inline uint16_t map255_to_top_safe(uint8_t duty0_255) {
  uint32_t ticks = (uint32_t)duty0_255 * (PWM_TOP + 1) / 255u;
  if (ticks == 0)        ticks = 1;
  if (ticks >= PWM_TOP)  ticks = PWM_TOP - 1;
  return (uint16_t)ticks;
}

static void set_pwm_0_255(uint8_t duty) {
  OCR1A = map255_to_top_safe(duty);
}

/* ------------------ SIMPLE PI(D) CONTROLLER ------------------ */
static float g_integral  = 0.0f;     // duty fraction
static float g_prevErr   = 0.0f;     // RPM
static float g_prevDuty  = 0.0f;     // duty fraction 0..1

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Return new duty fraction 0..1
static float simplePID_step(float setRPM, float measRPM, float dt)
{
  const float e = setRPM - measRPM;                     // RPM error
  const float dedt = (dt > 1e-6f) ? (e - g_prevErr) / dt : 0.0f;

  // Tentative integral update
  float integ_new = g_integral + (KI_DUTY_PER_RPM_S * e * dt);

  // Unsaturated control (duty fraction)
  float u_unsat =
      (KP_DUTY_PER_RPM * e)
    + integ_new
    + (KD_DUTY_PER_RPM_S * dedt);

  // Saturate
  float u_sat = clampf(u_unsat, DUTY_MIN_FRAC, DUTY_MAX_FRAC);

  // Conditional anti-windup
  const bool sat_hi = (u_unsat > DUTY_MAX_FRAC);
  const bool sat_lo = (u_unsat < DUTY_MIN_FRAC);
  if ((sat_hi && e > 0.0f) || (sat_lo && e < 0.0f)) {
    integ_new = g_integral;  // undo if pushing further into clamp
  }

  g_integral = integ_new;
  g_prevErr  = e;
  return u_sat;
}

/* ------------------ Arduino setup/loop ------------------ */
void setup() {
  pinMode(LED_BLINK_PIN, OUTPUT);
  digitalWrite(LED_BLINK_PIN, LOW);

  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n--- AeroHalo SIMPLE PID + PERIOD RPM @ 25 kHz (D9) ---"));

  timer1_init_25k_oc1a(PWM_INVERTED);

  pinMode(TACH_PIN, INPUT_PULLUP);               // + external 10k to 5V
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachISR, FALLING);

  g_lastEdgeUs = micros();
  g_lastEdgeMs = millis();

  g_integral = 0.0f;
  g_prevErr  = 0.0f;
  g_prevDuty = 0.0f;
}

void loop() {
  static unsigned long lastCtlMs = 0;
  static unsigned long lastUiMs  = 0;
  unsigned long now = millis();

  // Always read the latest RPM from ISR timing
  static float lastRPM = 0.0f;
  float rpm = rpmReadPeriodic();
  lastRPM = rpm;   // (0.0 is valid if timed out / stopped)

  // UI/print + blink every UI_MS
  if (now - lastUiMs >= UI_MS) {
    lastUiMs = now;
    const float tol = TARGET_RPM * RPM_TOL_PCT;
    const bool inBand = (fabsf(lastRPM - TARGET_RPM) <= tol);
    if (inBand) {
      digitalWrite(LED_BLINK_PIN, HIGH);
    } else
    {
      digitalWrite(LED_BLINK_PIN, LOW);
    }

    Serial.print(F("RPM="));     Serial.print(lastRPM, 0);
    Serial.print(F(", duty="));  Serial.print(g_prevDuty, 3);
    Serial.print(F(", OCR1A=")); Serial.print(OCR1A);
    Serial.print(F("/"));        Serial.print(PWM_TOP);
    Serial.print(F(", inBand="));Serial.println(inBand ? F("Y") : F("N"));
  }

  // Controller at fixed cadence
  if (now - lastCtlMs >= (unsigned long)(PID_DT_S * 1000.0f)) {
    lastCtlMs = now;

    float u = simplePID_step(TARGET_RPM, lastRPM, PID_DT_S);

    uint8_t duty255 = (uint8_t)lroundf(u * 255.0f);
    set_pwm_0_255(duty255);
    g_prevDuty = u;
  }
}
