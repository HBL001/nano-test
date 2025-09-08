#include <Arduino.h>

/* =========================================================================
 * AeroHalo Fan Control (Timer1 25 kHz on D9 + LabVIEW-style PID)
 *
 * QUICK REFERENCE (edit these to tune/port):
 * -------------------------------------------------------------------------
 * PINS
 *   FAN_PWM_PIN      D9  -> OC1A: 25 kHz PWM output to your transistor stage
 *   TACH_PIN         D3  -> INT1: fan tach input (open-collector; 10k pull-up to 5V)
 *   LED_BLINK_PIN    D5  -> blinks whenever |RPM - TARGET_RPM| <= 2% (each RPM window)
 *
 * PWM / TIMER1
 *   PWM_INVERTED     true means Timer1 inverts OC1A (matches your working demo).
 *                    Flip to false ONLY if your hardware polarity needs it.
 *   PWM_TOP          TOP=639 at prescale=1 -> 25 kHz. Do not hit 0 or TOP in OCR1A.
 *
 * TACH & SAMPLING
 *   PPR              pulses per rev from tach (most PC fans are 2)
 *   PID_DT_S         controller step (s)  -> 0.20 s = 5 Hz
 *   RPM_WIN_MS       tach window (ms)     -> 500 ms ~ 2 Hz RPM updates/LED blinks
 *
 * TARGET & BLINK
 *   TARGET_RPM       hard-wired setpoint
 *   RPM_TOL_PCT      blink band (fraction of setpoint), e.g. 0.02 = ±2%
 *   BLINK_MS         LED pulse width (ms) at each in-band RPM update
 *
 * OPTIONAL OUTPUT SLEW (for smooth duty changes)
 *   SLEW_UP_PER_S    max increase per second in duty fraction (0..1)
 *   SLEW_DN_PER_S    max decrease per second in duty fraction (0..1)
 *
 * PID (LabVIEW-style)
 *   KP, KI, KD       PID gains. Units are in % domain (see ERR_RPM_TO_PCT below).
 *   LV_BETA          setpoint weight for P (1.0 = classic P on error; <1 reduces SP kick)
 *   LV_TAU_D         derivative filter time constant (s); 0 disables filtering
 *   LV_OUT_MIN_PCT   output clamp min (%)     -> matches your LV node (1%)
 *   LV_OUT_MAX_PCT   output clamp max (%)     -> matches your LV node (99%)
 *   LV_DUTY_MIN_RUN_PCT  minimum run (%) when output > 0 (e.g., 5% to ensure spin)
 *
 * FEED-FORWARD & ERROR SCALING
 *   FF_A, FF_B       duty ≈ FF_A*RPM + FF_B (measured on your rig)
 *   LV_USE_FEEDFORWARD 1 to add FF to output (recommended); 0 to disable for testing
 *   ERR_RPM_TO_PCT   converts RPM error to % so P/I/D operate in percent space.
 *                    Chosen equal to (FF_A * 100) which is your measured slope (%/RPM)
 *
 * ANTI-WINDUP
 *   KAW              back-calculation gain (1/s). Higher unwinds I faster at clamps.
 *
 * DEBUG
 *   Serial prints: RPM, duty (0..1), OCR1A, inBand flag
 * -------------------------------------------------------------------------
 * Notes:
 * - Keep a real 10k pull-up to 5V on TACH. Use FALLING edge ISR (most fans pull low).
 * - If “higher duty → slower fan” or vice-versa, flip PWM_INVERTED.
 * - If duty sticks at 99% while RPM is high, raise KAW (e.g., 5–10).
 * - If response is too aggressive, lower KP/KI or reduce ERR_RPM_TO_PCT slightly.
 * ========================================================================= */

#define FAN_PWM_PIN      9
#define TACH_PIN         3
#define LED_BLINK_PIN    5

// Keep this as per your proven step-test path; flip if overall response is inverted.
static const bool PWM_INVERTED = true;

// Timer1 TOP for 25 kHz (16 MHz / (1*(1+639)) = 25 kHz)
static const uint16_t PWM_TOP = 639;

/* ------------------ Tach / sampling ------------------ */
#define PPR               2            // tach pulses per revolution
#define TARGET_RPM     150.0f
#define PID_DT_S        0.20f          // controller cadence ~5 Hz
#define RPM_WIN_MS        500          // RPM update window (~2 Hz print/blink)

/* ------------------ Blink: ±2% band ------------------ */
#define RPM_TOL_PCT       0.02f        // 2% of setpoint
#define BLINK_MS            60

/* ------------------ Optional slew on duty write ------------------ */
#define SLEW_UP_PER_S     2.0f         // duty fraction per second
#define SLEW_DN_PER_S     4.0f

/* ------------------ PID & limits (LabVIEW-style) ------------------ */
#define KP                0.85f
#define KI                0.01f
#define KD                0.005f

#define LV_BETA           1.0f         // setpoint weight for P
#define LV_TAU_D          0.02f        // derivative filter time constant (s)
#define LV_OUT_MIN_PCT    1.0f         // output clamp min (%)
#define LV_OUT_MAX_PCT   99.0f         // output clamp max (%)
#define LV_DUTY_MIN_RUN_PCT 5.0f       // minimum run (%) when >0

/* ------------------ Feed-forward & error scaling ------------------ */
// Your measured linear map (kept here explicitly)
#define FF_A              0.001228f    // duty fraction per RPM
#define FF_B              0.3758f      // duty fraction offset
#define LV_USE_FEEDFORWARD 1           // 1=add FF; 0=disable

// Convert RPM error to P/I/D percent domain using your measured slope
#define ERR_RPM_TO_PCT   (FF_A * 100.0f)  // 0.1228 % per RPM

/* ------------------ Anti-windup (back-calculation) ------------------ */
#define KAW               3.0f         // 1/s; how strongly I is pulled by (u_sat - u)

/* ------------------ Tach capture ------------------ */
volatile unsigned long g_tachEdges = 0;
volatile unsigned long g_lastEdgeUs = 0;

static void tachISR() {
  unsigned long now = micros();
  if (now - g_lastEdgeUs >= 50) {      // ~50 us deglitch
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

/* ------------------ Timer1 @ 25 kHz on D9 (as per your working demo) ------------------ */
static void timer1_init_25k_oc1a(bool inverted) {
  DDRB |= _BV(DDB1);              // D9 as output

  TCCR1A = 0;
  TCCR1B = 0;

  if (inverted) {
    TCCR1A |= _BV(COM1A1) | _BV(COM1A0);   // inverting
  } else {
    TCCR1A |= _BV(COM1A1);                 // non-inverting
  }

  TCCR1A |= _BV(WGM11);
  TCCR1B |= _BV(WGM13) | _BV(WGM12);       // Fast PWM, TOP = ICR1
  TCCR1B |= _BV(CS10);                     // clk/1

  ICR1  = PWM_TOP;
  OCR1A = 1;                               // safe non-zero start (avoid 0/TOP)
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

/* ------------------ LabVIEW-equivalent PID core (percent domain) ------------------ */
static float lv_integral = 0.0f;      // % units
static float lv_prev_meas = 0.0f;     // last RPM
static float lv_d_filt    = 0.0f;     // filtered D term in % units

// Returns duty as 0..1 fraction
static float controlStep_LV(float setpointRPM, float measuredRPM, float dt_s)
{
  const float dt = (dt_s > 1e-6f) ? dt_s : 1e-6f;
  const float k  = ERR_RPM_TO_PCT;             // % per RPM

  // 1) Error (RPM → %) and P-term (setpoint-weighted), in percent
  const float e_rpm = setpointRPM - measuredRPM;
  const float e_pct = k * e_rpm;
  const float up    = KP * (k * (LV_BETA*setpointRPM - measuredRPM));

  // 2) D on measurement with 1st-order LPF, output in percent
  const float d_meas_rpm = (measuredRPM - lv_prev_meas) / dt;
  lv_prev_meas = measuredRPM;
  const float alpha = (LV_TAU_D <= 0.0f) ? 1.0f : (dt / (LV_TAU_D + dt));
  const float d_raw_pct = -KD * (d_meas_rpm * k);     // minus sign: D on measurement
  lv_d_filt += alpha * (d_raw_pct - lv_d_filt);
  const float ud = lv_d_filt;

  // 3) Integrate error in percent
  lv_integral += (KI * e_pct) * dt;

  // 4) Combine + optional feed-forward (percent)
  float u = up + ud + lv_integral;
#if LV_USE_FEEDFORWARD
  u += (FF_A * setpointRPM + FF_B) * 100.0f;          // FF in %
#endif

  // 5) Saturate to [min..max] and enforce minimum-run (all in %)
  float u_sat = u;
  if (u_sat > LV_OUT_MAX_PCT) u_sat = LV_OUT_MAX_PCT;
  if (u_sat < LV_OUT_MIN_PCT) u_sat = LV_OUT_MIN_PCT;
  if (LV_DUTY_MIN_RUN_PCT > 0.0f && u_sat > 0.0f && u_sat < LV_DUTY_MIN_RUN_PCT)
    u_sat = LV_DUTY_MIN_RUN_PCT;

  // 6) Anti-windup by back-calculation (pull I toward the saturated output)
  lv_integral += KAW * (u_sat - u) * dt;

  // 7) Return 0..1 duty fraction
  float duty_frac = u_sat / 100.0f;
  if (duty_frac < 0.0f) duty_frac = 0.0f;
  if (duty_frac > 1.0f) duty_frac = 1.0f;
  return duty_frac;
}

/* ------------------ Duty writer with optional slew ------------------ */
static float         g_prevDuty   = 0.0f;
static unsigned long g_lastDutyMs = 0;

static void writeDutyFrac(float target) {
  if (target < 0.0f) target = 0.0f;
  if (target > 1.0f) target = 1.0f;

  unsigned long now = millis();
  float dt = (now - g_lastDutyMs) / 1000.0f;
  if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;           // robust if first run/lag
  g_lastDutyMs = now;

  const float maxUp = (SLEW_UP_PER_S > 0.0f) ? (SLEW_UP_PER_S * dt) : 1e9f;
  const float maxDn = (SLEW_DN_PER_S > 0.0f) ? (SLEW_DN_PER_S * dt) : 1e9f;

  float u = target;
  if (u > g_prevDuty + maxUp) u = g_prevDuty + maxUp;
  if (u < g_prevDuty - maxDn) u = g_prevDuty - maxDn;

  const uint8_t duty255 = (uint8_t)roundf(u * 255.0f);
  set_pwm_0_255(duty255);

  g_prevDuty = u;
}

/* ------------------ Arduino setup/loop ------------------ */
void setup() {
  pinMode(LED_BLINK_PIN, OUTPUT);
  digitalWrite(LED_BLINK_PIN, LOW);

  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n--- AeroHalo LabVIEW-PID @ 25 kHz (D9) ---"));
  Serial.println(F("Blink D5 within ±2% of setpoint; RPM window 500 ms."));

  timer1_init_25k_oc1a(PWM_INVERTED);

  pinMode(TACH_PIN, INPUT_PULLUP);               // still add external 10k -> 5V
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachISR, FALLING);

  // Reset controller state
  lv_integral = 0.0f;
  lv_prev_meas = 0.0f;
  lv_d_filt = 0.0f;
  g_prevDuty = 0.0f;
  g_lastDutyMs = millis();
}

void loop() {
  static unsigned long lastCtlMs = 0;
  unsigned long now = millis();

  // 1) RPM update + blink-on-band
  float rpm = rpmComputeWindowed();
  if (rpm >= 0.0f) {
    const float tol = TARGET_RPM * RPM_TOL_PCT;
    const bool inBand = (fabsf(rpm - TARGET_RPM) <= tol);

    if (inBand) {
      digitalWrite(LED_BLINK_PIN, HIGH);
      delay(BLINK_MS);
      digitalWrite(LED_BLINK_PIN, LOW);
    }

    Serial.print(F("RPM="));
    Serial.print(rpm, 0);
    Serial.print(F(", duty="));
    Serial.print(g_prevDuty, 3);
    Serial.print(F(", OCR1A="));
    Serial.print(OCR1A);
    Serial.print(F("/"));
    Serial.print(PWM_TOP);
    Serial.print(F(", inBand="));
    Serial.println(inBand ? F("Y") : F("N"));
  }

  // 2) Controller at fixed cadence
  if (now - lastCtlMs >= (unsigned long)(PID_DT_S * 1000.0f)) {
    lastCtlMs = now;
    // Use the latest available RPM; if none yet, reuse last measurement
    float measured = (rpm >= 0.0f ? rpm : lv_prev_meas);
    float u = controlStep_LV(TARGET_RPM, measured, PID_DT_S);
    writeDutyFrac(u);
  }
}
