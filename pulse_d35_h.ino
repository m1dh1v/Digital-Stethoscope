#include <Arduino.h>
#include <math.h>

const int    MIC_PIN   = 35;
const int    FS        = 2000;
const int    AVG       = 4;
const double F_HP      = 15.0;
const double F_LP      = 80.0;
const double F_NOTCH   = 50.0;
const double F_ENV     = 6.0;
const double F_BASE    = 0.3;
const int    WIN       = 100;
const int    WARMUP_S  = 4;
const bool   SHOW_ENV  = false;

struct Biquad {
  double b0, b1, b2, a1, a2;
  double z1, z2;
};

void makeLowpass(Biquad &f, double fc, double q) {
  double w0 = 2.0 * PI * fc / FS;
  double c = cos(w0), alpha = sin(w0) / (2.0 * q);
  double a0 = 1.0 + alpha;
  f.b0 = (1.0 - c) / 2.0 / a0;  f.b1 = (1.0 - c) / a0;  f.b2 = f.b0;
  f.a1 = -2.0 * c / a0;         f.a2 = (1.0 - alpha) / a0;
  f.z1 = 0; f.z2 = 0;
}

void makeHighpass(Biquad &f, double fc, double q) {
  double w0 = 2.0 * PI * fc / FS;
  double c = cos(w0), alpha = sin(w0) / (2.0 * q);
  double a0 = 1.0 + alpha;
  f.b0 = (1.0 + c) / 2.0 / a0;  f.b1 = -(1.0 + c) / a0;  f.b2 = f.b0;
  f.a1 = -2.0 * c / a0;         f.a2 = (1.0 - alpha) / a0;
  f.z1 = 0; f.z2 = 0;
}

void makeNotch(Biquad &f, double fc, double q) {
  double w0 = 2.0 * PI * fc / FS;
  double c = cos(w0), alpha = sin(w0) / (2.0 * q);
  double a0 = 1.0 + alpha;
  f.b0 = 1.0 / a0;  f.b1 = -2.0 * c / a0;  f.b2 = 1.0 / a0;
  f.a1 = -2.0 * c / a0;  f.a2 = (1.0 - alpha) / a0;
  f.z1 = 0; f.z2 = 0;
}

double runBiquad(Biquad &f, double x) {
  double y = f.b0 * x + f.z1;
  f.z1 = f.b1 * x - f.a1 * y + f.z2;
  f.z2 = f.b2 * x - f.a2 * y;
  return y;
}

Biquad hp1, hp2, lp1, lp2, nt;
double aEnv, aBase;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);
  makeHighpass(hp1, F_HP, 0.5412);
  makeHighpass(hp2, F_HP, 1.3066);
  makeLowpass(lp1, F_LP, 0.5412);
  makeLowpass(lp2, F_LP, 1.3066);
  makeNotch(nt, F_NOTCH, 8.0);
  aEnv  = 1.0 - exp(-2.0 * PI * F_ENV  / FS);
  aBase = 1.0 - exp(-2.0 * PI * F_BASE / FS);
}

void loop() {
  static uint32_t next = micros();
  static double x0 = 0, env = 0, base = 0;
  static int n = 0, warm = WARMUP_S * FS;
  static bool started = false;

  while ((int32_t)(micros() - next) < 0) {}
  next += 1000000UL / FS;

  uint32_t sum = 0;
  for (int i = 0; i < AVG; i++) sum += analogRead(MIC_PIN);
  double x = (double)sum / AVG;
  if (!started) { x0 = x; started = true; }

  double y = runBiquad(hp1, x - x0);
  y = runBiquad(hp2, y);
  y = runBiquad(lp1, y);
  y = runBiquad(lp2, y);
  y = runBiquad(nt, y);

  env  += aEnv  * (fabs(y) - env);
  base += aBase * (env - base);

  if (warm > 0) {
    warm--;
    if (warm == 0) base = env;
    return;
  }

  if (++n >= WIN) {
    n = 0;
    Serial.print("Beat:");
    Serial.print(env - base, 2);
    if (SHOW_ENV) { Serial.print(",Env:"); Serial.print(env, 2); }
    Serial.println();
  }
}
