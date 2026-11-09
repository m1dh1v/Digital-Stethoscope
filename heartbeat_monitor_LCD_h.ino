#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Pin and timing settings

const int    MIC_PIN    = 35;
const int    LED_RED    = 25;
const int    LED_GREEN  = 26;
const int    BTN_PIN    = 27;
const int    LCD_SDA    = 21;
const int    LCD_SCL    = 22;
const uint8_t LCD_ADDR  = 0x27;
const int    LCD_COLS   = 16;
const int    LCD_ROWS   = 2;

const int    AIR_S      = 8;
const int    GOOD_NEED_S = 2;
const int    BAD_LIMIT_S = 3;
const int    NOW_WINDOW_S = 4;
const int    AVG_WINDOW_S = 10;
const int    DEBUG      = 0;

const int    FS         = 2000;
const int    AVG        = 4;
const double F_HP       = 15.0;
const double F_LP       = 80.0;
const double F_MAINS    = 50.0;
const double Q_NOTCH    = 6.0;
const double F_ENV      = 6.0;
const double MIN_RANGE  = 1.0;
const double MIN_RATIO  = 1.3;
const double TRIG_HI    = 0.50;
const double TRIG_LO    = 0.30;
const double TAU_ATTACK_S = 0.05;
const double TAU_DECAY_S  = 2.0;
const double HI_CAP_RATIO = 10.0;
const double REFRACT_MIN_S = 0.25;
const double REFRACT_MAX_S = 0.50;
const double BEAT_MIN_S = 0.30;
const double BEAT_MAX_S = 1.50;
const int    WARMUP_S   = 3;

const int    ENV_DEC    = 40;
const int    ENV_FS     = FS / ENV_DEC;
const int    AC_WIN     = 10 * ENV_FS;
const int    AC_MIN     = 4 * ENV_FS;
const int    LAG_MIN    = (int)(0.40 * ENV_FS + 0.5);
const int    LAG_MAX    = (int)(1.50 * ENV_FS + 0.5);
const double Q_CONN     = 0.25;
const double Q_STAY     = 0.17;
const double Q_SOME     = 0.15;
const double PROMINENCE = 0.15;
const double SAME_TOL   = 0.12;
const int    PRINT_PLOT = 0;

// Connection states
enum State { ST_AIR, ST_PLACE, ST_READING };

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

Biquad hp1, hp2, lp1, lp2, nt1, nt2, nt3, nt4, nt5;
double aEnv, aUp, aDown;

long    n = 0;
double  x0 = 0, env = 0, hi = 0, lo = 0;
bool    started = false, armed = true;
long    warmEnd = 0;
double  tSession0 = 0;
long    railCount = 0;
bool    fault = false;

const int MAX_BEATS = 64;
double beatT[MAX_BEATS];  int beatHead = 0;
double lastBeatT = -100.0;
long   totalBeats = 0;
long   envM = 0;
volatile double phiRef = 0, phiT = 0;
volatile bool   phaseValid = false;
double foldF[96];  int foldNb = 0;
double riseT = -100.0;
bool   belowLo = true;
volatile bool secPending = false;
volatile double secT = 0;
volatile bool clearReq = false;
long   blankUntil = 0;
long   lateTicks = 0;  uint32_t lateMax = 0;
double readFrom = 0;
double refractory = 0.40;

const int ENV_N = 600;
float envBuf[ENV_N];  int envHead = 0, envCount = 0;
double acQ = 0, acLag = 0;

State  state = ST_AIR;
int    goodStreak = 0, badStreak = 0;
bool   wasConnected = false;
int    readSec = 0;
double smoothLag = 0;
bool   pulsesFound = false;
int    nowBpm = -1, avgBpm = -1;
double gCv = -1.0;

char   dispLine[4][21];
volatile bool lcdPresent = false;

void clearBeats() {
  beatHead = 0; totalBeats = 0; lastBeatT = -100.0;
}

int beatsInWindow(double tSec, double window, double *out) {
  int stored = (totalBeats < MAX_BEATS) ? (int)totalBeats : MAX_BEATS;
  int k = 0;
  for (int i = 0; i < stored; i++) {
    int idx = (beatHead - stored + i + 2 * MAX_BEATS) % MAX_BEATS;
    if (beatT[idx] > tSec - window && beatT[idx] >= readFrom) out[k++] = beatT[idx];
  }
  return k;
}

void computeAutocorr(int winMax, double *qOut, double *lagOut) {
  *qOut = 0; *lagOut = 0;
  const int M = (envCount < winMax) ? envCount : winMax;
  if (M < AC_MIN) return;
  static float  x[AC_WIN];
  static double cs[AC_WIN + 1];
  static double r[LAG_MAX + 2];
  double mean = 0;
  for (int i = 0; i < M; i++) { x[i] = envBuf[(envHead - M + i + 2 * ENV_N) % ENV_N]; mean += x[i]; }
  mean /= M;
  double ic = 0.5 * (M - 1), sxx = 0, sxy = 0;
  for (int i = 0; i < M; i++) { sxx += (i - ic) * (i - ic); sxy += (i - ic) * (x[i] - mean); }
  double slope = sxy / sxx;
  cs[0] = 0;
  for (int i = 0; i < M; i++) { x[i] = (float)(x[i] - mean - slope * (i - ic)); cs[i + 1] = cs[i] + (double)x[i] * x[i]; }
  if (cs[M] <= 1e-9) return;
  for (int k = 1; k <= LAG_MAX + 1; k++) {
    if (M - k < 100) { r[k] = 0.0; continue; }
    double num = 0;
    for (int i = 0; i + k < M; i++) num += (double)x[i] * x[i + k];
    double den = sqrt((cs[M - k] - cs[0]) * (cs[M] - cs[k]));
    r[k] = (den > 0) ? num / den : 0.0;
  }

  static double valley[LAG_MAX + 2];
  valley[0] = 1.0;
  for (int k = 1; k <= LAG_MAX + 1; k++) valley[k] = (r[k] < valley[k - 1]) ? r[k] : valley[k - 1];
  int kb = -1;  double rb = -1.0;
  for (int k = LAG_MIN; k <= LAG_MAX; k++)
    if (r[k] >= r[k - 1] && r[k] >= r[k + 1] && r[k] - valley[k] >= PROMINENCE && r[k] > rb) { rb = r[k]; kb = k; }
  if (kb < 0) return;
  for (int div = 2; div <= 3; div++) {
    int kc = (int)((double)kb / div + 0.5);
    for (int k = kc - 2; k <= kc + 2; k++)
      if (k >= LAG_MIN && k <= LAG_MAX && r[k] >= r[k - 1] && r[k] >= r[k + 1] && r[k] - valley[k] >= PROMINENCE && r[k] >= 0.9 * rb) { rb = r[k]; kb = k; break; }
  }
  double d = r[kb - 1] - 2.0 * r[kb] + r[kb + 1];
  double off = (d < 0) ? 0.5 * (r[kb - 1] - r[kb + 1]) / d : 0.0;
  *lagOut = (kb + off) / ENV_FS;
  *qOut = rb;
}

// Reset the filters and start a fresh reading
void resetSession() {
  makeHighpass(hp1, F_HP, 0.5412);
  makeHighpass(hp2, F_HP, 1.3066);
  makeLowpass(lp1, F_LP, 0.5412);
  makeLowpass(lp2, F_LP, 1.3066);
  makeNotch(nt1, F_MAINS, Q_NOTCH);
  makeNotch(nt2, F_MAINS, Q_NOTCH);
  makeNotch(nt3, 2 * F_MAINS, 2 * Q_NOTCH);
  makeNotch(nt4, 2 * F_MAINS, 2 * Q_NOTCH);
  makeNotch(nt5, 3 * F_MAINS, 3 * Q_NOTCH);
  env = 0; hi = 0; lo = 0; armed = true; started = false;
  clearBeats();
  phaseValid = false; riseT = -100.0; belowLo = true;
  envHead = 0; envCount = 0; acQ = 0; acLag = 0; smoothLag = 0; refractory = 0.40;
  state = ST_AIR; goodStreak = 0; badStreak = 0; wasConnected = false; pulsesFound = false; readFrom = 0;
  nowBpm = -1; avgBpm = -1; gCv = -1.0;
  warmEnd = n + (long)WARMUP_S * FS;
  tSession0 = (double)n / FS;
  railCount = 0; fault = false;
}

void setLine(int r, const char *s) {
  int i = 0;
  for (; i < 20 && s[i]; i++) dispLine[r][i] = s[i];
  for (; i < 20; i++) dispLine[r][i] = ' ';
  dispLine[r][20] = 0;
}

void fmt3(char *out, int v) {
  if (v < 0) strcpy(out, " --"); else snprintf(out, 8, "%3d", (v > 999) ? 999 : v);
}

const char *varLabel(double cv) {
  if (cv < 0) return "...";
  if (cv < 0.08) return "Steady";
  if (cv < 0.15) return "Slight";
  return "Uneven";
}

void buildDisplay(double tSec) {
  char a[16], b[16], l[48];
  int  varPct = (gCv >= 0) ? (int)(gCv * 100.0 + 0.5) : -1;
  for (int r = 0; r < 4; r++) setLine(r, "");
  if (fault) {
    setLine(0, "NO MIC SIGNAL"); setLine(1, "check wires");
    return;
  }
  if (state == ST_AIR) {
    int left = AIR_S - (int)(tSec - tSession0);
    if (left < 0) left = 0;
    setLine(0, "KEEP MIC IN AIR");
    snprintf(l, sizeof l, "calibrating %2d s", left);
    setLine(1, l);
    return;
  }
  if (state == ST_PLACE) {
    setLine(0, wasConnected ? "CONNECTION LOST" : "CONNECTING...");
    setLine(1, pulsesFound ? "hold still" : "press on chest");
    return;
  }
  fmt3(a, nowBpm); fmt3(b, avgBpm);
  if (LCD_ROWS >= 4) {
    snprintf(l, sizeof l, "NOW %s bpm  (%d s)", a, NOW_WINDOW_S);   setLine(0, l);
    snprintf(l, sizeof l, "AVG %s bpm  (%d s)", b, AVG_WINDOW_S);   setLine(1, l);
    if (varPct >= 0) snprintf(l, sizeof l, "Beat: %s %d%%", varLabel(gCv), varPct);
    else             snprintf(l, sizeof l, "Beat: ...");
    setLine(2, l);
    setLine(3, "Signal: GOOD");
  } else {
    snprintf(l, sizeof l, "NOW %s  AVG %s", a, b);                  setLine(0, l);
    if (varPct >= 0) snprintf(l, sizeof l, "%-6s %2d%%", varLabel(gCv), varPct);
    else             strcpy(l, "...");
    setLine(1, l);
  }
}

#define LCD_BL 0x08
#define LCD_EN 0x04
#define LCD_RS 0x01

void lcdNibble(uint8_t nib, uint8_t rs) {
  uint8_t b = (nib & 0xF0) | LCD_BL | rs;
  Wire.beginTransmission(LCD_ADDR);
  Wire.write((uint8_t)(b | LCD_EN));
  Wire.write(b);
  Wire.endTransmission();
  delayMicroseconds(60);
}

void lcdByte(uint8_t v, uint8_t rs) {
  lcdNibble(v & 0xF0, rs);
  lcdNibble((v << 4) & 0xF0, rs);
}

void lcdCommand(uint8_t c) { lcdByte(c, 0); if (c <= 0x03) delay(3); }

void lcdInit() {
  delay(60);
  lcdNibble(0x30, 0); delay(5);
  lcdNibble(0x30, 0); delay(5);
  lcdNibble(0x30, 0); delay(1);
  lcdNibble(0x20, 0); delay(1);
  lcdCommand(0x28);
  lcdCommand(0x08);
  lcdCommand(0x01);
  lcdCommand(0x06);
  lcdCommand(0x0C);
}

void lcdSetCursor(int col, int row) {
  static const uint8_t base[4] = {0x00, 0x40, 0x14, 0x54};
  lcdCommand(0x80 | (base[row] + col));
}

void lcdTask(void *) {
  Wire.begin(LCD_SDA, LCD_SCL);
  Wire.beginTransmission(LCD_ADDR);
  bool found = (Wire.endTransmission() == 0);
  if (!found) {
    lcdPresent = false;
    vTaskDelete(nullptr);
    return;
  }
  lcdPresent = true;
  lcdInit();
  for (;;) {
    char copy[4][21];
    memcpy(copy, dispLine, sizeof copy);
    for (int r = 0; r < LCD_ROWS; r++) {
      lcdSetCursor(0, r);
      for (int c = 0; c < LCD_COLS; c++) lcdByte((uint8_t)copy[r][c], LCD_RS);
    }
    delay(400);
  }
}

void analysisTask(void *);

// Start the serial monitor and background tasks
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);
  pinMode(LED_RED, OUTPUT);   digitalWrite(LED_RED, HIGH);
  pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, LOW);
  pinMode(BTN_PIN, INPUT_PULLUP);
  aEnv  = 1.0 - exp(-2.0 * PI * F_ENV / FS);
  aUp   = 1.0 - exp(-1.0 / (FS * TAU_ATTACK_S));
  aDown = 1.0 - exp(-1.0 / (FS * TAU_DECAY_S));
  resetSession();
  setLine(0, "STARTING"); setLine(1, "");
  for (int r = 2; r < 4; r++) setLine(r, "");
  xTaskCreatePinnedToCore(lcdTask, "lcd", 4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(analysisTask, "ana", 8192, nullptr, 1, nullptr, 0);
  Serial.println("Heartbeat monitor started. Keep the mic in the air for a few seconds.");
}

void computePhase(double T) {
  phaseValid = false;
  const int M = (envCount < AC_WIN) ? envCount : AC_WIN;
  if (M < AC_MIN || T <= 0) return;
  double Tm = T * ENV_FS;
  int nb = (int)(Tm + 0.5);
  if (nb < 12 || nb > 90) return;
  static double sum[96], f[96];
  static int cnt[96];
  for (int b = 0; b < nb; b++) { sum[b] = 0; cnt[b] = 0; }
  long mNew = envM;
  for (int i = 0; i < M; i++) {
    long m = mNew - (M - 1 - i);
    double ph = fmod((double)m, Tm) / Tm;
    if (ph < 0) ph += 1.0;
    int b = (int)(ph * nb);  if (b >= nb) b = nb - 1;
    sum[b] += envBuf[(envHead - M + i + 2 * ENV_N) % ENV_N];
    cnt[b]++;
  }
  for (int b = 0; b < nb; b++) if (cnt[b] == 0) return;
  for (int b = 0; b < nb; b++) f[b] = 0.25 * (sum[(b + nb - 1) % nb] / cnt[(b + nb - 1) % nb] + 2.0 * sum[b] / cnt[b] + sum[(b + 1) % nb] / cnt[(b + 1) % nb]);
  double fl = f[0], fh = f[0];
  for (int b = 1; b < nb; b++) { if (f[b] < fl) fl = f[b]; if (f[b] > fh) fh = f[b]; }
  if (fh - fl < 1e-6) return;
  for (int b = 0; b < nb; b++) foldF[b] = f[b];
  foldNb = nb;
  double level = fl + 0.4 * (fh - fl);
  int bestLen = 0, bestEnd = -1, run = 0;
  for (int i = 0; i < 2 * nb; i++) {
    if (f[i % nb] < level) { run++; if (run > bestLen && run <= nb) { bestLen = run; bestEnd = i % nb; } }
    else run = 0;
  }
  if (bestLen < 3 || bestLen >= nb) return;
  int onsetBin = (bestEnd + 1) % nb;
  double pOn = (double)onsetBin / nb;
  double tEnd = (double)mNew / ENV_FS;
  phiRef = floor(tEnd / T - pOn) * T + pOn * T;
  phiT = T;
  phaseValid = true;
}

int matchedBeats(double T, double *bt, int maxN) {
  const int M = (envCount < AC_WIN) ? envCount : AC_WIN;
  const int nb = foldNb;
  if (!phaseValid || M < AC_MIN || nb < 12) return 0;
  static float  x[AC_WIN];
  static float  c[AC_WIN];
  static float  tp[96];
  static float  cs1[AC_WIN + 1], cs2[AC_WIN + 1];
  double mean = 0;
  for (int i = 0; i < M; i++) { x[i] = envBuf[(envHead - M + i + 2 * ENV_N) % ENV_N]; mean += x[i]; }
  mean /= M;
  cs1[0] = 0; cs2[0] = 0;
  for (int i = 0; i < M; i++) { x[i] -= (float)mean; cs1[i + 1] = cs1[i] + x[i]; cs2[i + 1] = cs2[i] + x[i] * x[i]; }
  double tm = 0, tn = 0;
  for (int b = 0; b < nb; b++) tm += foldF[b];
  tm /= nb;
  for (int b = 0; b < nb; b++) { tp[b] = (float)(foldF[b] - tm); tn += (double)tp[b] * tp[b]; }
  if (tn < 1e-9) return 0;
  tn = sqrt(tn);
  for (int b = 0; b < nb; b++) tp[b] = (float)(tp[b] / tn);
  int nc = M - nb + 1;
  if (nc < 3) return 0;
  float cmax = -1.0f;
  for (int j = 0; j < nc; j++) {
    float num = 0;
    for (int i = 0; i < nb; i++) num += tp[i] * x[j + i];
    float sx = cs1[j + nb] - cs1[j], sxx = cs2[j + nb] - cs2[j];
    float var = sxx - sx * sx / nb;
    c[j] = (var > 1e-9f) ? num / sqrtf(var) : 0.0f;
    if (c[j] > cmax) cmax = c[j];
  }
  if (cmax < 0.3f) return 0;
  int sep = (int)(0.6 * T * ENV_FS);
  static bool used[AC_WIN];
  for (int j = 0; j < nc; j++) used[j] = false;
  int idx[24], n = 0;
  while (n < 24) {
    int jb = -1;  float vb = 0.5f * cmax;
    for (int j = 1; j < nc - 1; j++)
      if (!used[j] && c[j] >= vb && c[j] >= c[j - 1] && c[j] >= c[j + 1]) { vb = c[j]; jb = j; }
    if (jb < 0) break;
    idx[n++] = jb;
    for (int j = jb - sep; j <= jb + sep; j++) if (j >= 0 && j < nc) used[j] = true;
  }
  for (int i = 1; i < n; i++) { int v = idx[i], k = i - 1; while (k >= 0 && idx[k] > v) { idx[k + 1] = idx[k]; k--; } idx[k + 1] = v; }
  int cnt = 0;
  for (int i = 0; i < n && cnt < maxN; i++) {
    int j = idx[i];
    double d = c[j - 1] - 2.0 * c[j] + c[j + 1];
    double off = (d < 0) ? 0.5 * (c[j - 1] - c[j + 1]) / d : 0.0;
    bt[cnt++] = ((double)(envM - (M - 1) + j) + off) / ENV_FS;
  }
  return cnt;
}

void secondTick(double tSec) {
  static double prevLag = 0;
  static bool   prevOk = false;
  computeAutocorr(AC_WIN, &acQ, &acLag);
  double qShort = 0, lagShort = 0;
  computeAutocorr(5 * ENV_FS, &qShort, &lagShort);
  bool   hasP   = !fault && acQ >= Q_SOME && acLag > 0;
  double T      = hasP ? acLag : 0.0;
  pulsesFound   = hasP;

  refractory = (T > 0) ? constrain(0.60 * T, REFRACT_MIN_S, REFRACT_MAX_S) : 0.40;

  double qNeed  = (state == ST_READING) ? Q_STAY : Q_CONN;
  bool   ok     = !fault && acQ >= qNeed && acLag > 0;
  bool   same   = ok && prevOk && fabs(acLag - prevLag) <= SAME_TOL * prevLag;
  prevOk = ok;  prevLag = acLag;
  double tmpB[MAX_BEATS];

  if (state == ST_AIR) {
    if (tSec - tSession0 >= AIR_S) {
      state = ST_PLACE; goodStreak = 0; badStreak = 0;
      clearReq = true;
    }
  } else if (state == ST_PLACE) {
    goodStreak = same ? goodStreak + 1 : 0;
    if (goodStreak >= GOOD_NEED_S) { state = ST_READING; badStreak = 0; wasConnected = true; smoothLag = acLag; readSec = 0; }
  } else {
    int kAll = beatsInWindow(tSec, AVG_WINDOW_S, tmpB), kNow = beatsInWindow(tSec, 3.0, tmpB);
    bool vanished = (kAll >= 4 && kNow == 0);
    bool alive = ok && qShort >= fmax(0.8 * Q_STAY, 0.5 * acQ) && !vanished;
    badStreak = alive ? 0 : badStreak + 1;
    if (ok && same) smoothLag = 0.5 * smoothLag + 0.5 * acLag;
    else if (acQ >= Q_CONN && acLag > 0) smoothLag = acLag;
    if (badStreak >= BAD_LIMIT_S) { state = ST_PLACE; goodStreak = 0; }
  }

  if (state == ST_READING && smoothLag > 0) computePhase(smoothLag); else phaseValid = false;

  if (state == ST_READING) {
    avgBpm = (smoothLag > 0) ? (int)(60.0 / smoothLag + 0.5) : -1;

    double mb[24];
    int nbeat = matchedBeats(smoothLag, mb, 24);
    int nowRaw = -1;
    double nowWin = fmax((double)NOW_WINDOW_S, 3.3 * smoothLag);
    {
      int k = 0; double first = 0, last = 0;
      for (int i = 0; i < nbeat; i++) if (mb[i] > tSec - nowWin) { if (k == 0) first = mb[i]; last = mb[i]; k++; }
      if (k >= 3 && last > first) nowRaw = (int)(60.0 * (k - 1) / (last - first) + 0.5);
    }
    nowBpm = (nowRaw > 0 && avgBpm > 0 && fabs((double)nowRaw - avgBpm) <= 0.25 * avgBpm) ? nowRaw : avgBpm;
    readSec++;
    gCv = -1.0;
    if (readSec >= 6 && nbeat >= 5) {
      double g[24]; int m = 0;
      for (int i = 1; i < nbeat; i++) { double d = mb[i] - mb[i - 1]; if (d <= 1.7 * smoothLag) g[m++] = d; }
      if (m >= 4) {
        double mean = 0, sd = 0;
        for (int i = 0; i < m; i++) mean += g[i];
        mean /= m;
        for (int i = 0; i < m; i++) sd += (g[i] - mean) * (g[i] - mean);
        gCv = sqrt(sd / m) / mean;
      }
    }
  } else {
    nowBpm = -1; avgBpm = -1; gCv = -1.0;
  }
  buildDisplay(tSec);

  if (!PRINT_PLOT) {
    char nb[8], ab[8], extra[96] = "";
    if (DEBUG) snprintf(extra, sizeof extra, " | q=%.2f ac=%d state=%d floor=%.1f peak=%.1f late=%ld/%luus",
                        acQ, (acLag > 0) ? (int)(60.0 / acLag + 0.5) : -1, (int)state, lo, hi, lateTicks, (unsigned long)lateMax);
    if (fault) {
      Serial.printf("No mic signal - check the wiring%s\n", extra);
    } else if (state == ST_AIR) {
      Serial.printf("Calibrating: keep the mic in the air (%d s)%s\n", AIR_S - (int)(tSec - tSession0), extra);
    } else if (state == ST_PLACE) {
      Serial.printf("%s%s\n", wasConnected ? "Connection lost - press the mic against your chest"
                                           : "Connecting... press the mic against your chest", extra);
    } else {
      fmt3(nb, nowBpm); fmt3(ab, avgBpm);
      if (gCv >= 0) Serial.printf("NOW %s bpm | AVG %s bpm | %s%s\n", nb, ab, varLabel(gCv), extra);
      else          Serial.printf("NOW %s bpm | AVG %s bpm | rhythm ...%s\n", nb, ab, extra);
    }
  }
}

void analysisTask(void *) {
  for (;;) {
    if (secPending) { double t = secT; secPending = false; secondTick(t); }
    delay(5);
  }
}

// Read the mic and look for heartbeat patterns
void loop() {
  static uint32_t next = micros();
  static uint32_t btnLowSince = 0;

  while ((int32_t)(micros() - next) < 0) {}
  uint32_t late = micros() - next;
  if (late > 1500 && late < 1000000UL) {
    lateTicks++;  if (late > lateMax) lateMax = late;
    blankUntil = n + FS / 5;
  }
  next += 1000000UL / FS;
  n++;
  double tSec = (double)n / FS;

  bool red = false, green = false;
  if (state == ST_AIR) {
    red = true;
  } else if (state == ST_READING) {
    green = true;
    red   = !armed;
  } else {
    green = pulsesFound ? (((n / (FS / 2)) % 2) == 0)
                        : (((n / (FS / 8)) % 2) == 0);
  }
  digitalWrite(LED_RED, red ? HIGH : LOW);
  digitalWrite(LED_GREEN, green ? HIGH : LOW);

  if (digitalRead(BTN_PIN) == LOW) {
    if (btnLowSince == 0) btnLowSince = (uint32_t)n;
    if ((uint32_t)n - btnLowSince > (uint32_t)(0.3 * FS)) {
      resetSession();
      setLine(0, "STARTING"); setLine(1, "");
      Serial.println("Restart. Keep the mic in the air for a few seconds.");
      btnLowSince = 0;
      return;
    }
  } else {
    btnLowSince = 0;
  }

  uint32_t sum = 0;
  for (int i = 0; i < AVG; i++) sum += analogRead(MIC_PIN);
  double x = (double)sum / AVG;
  if (x < 20.0 || x > 4075.0) railCount++; else railCount = 0;
  fault = (railCount > FS);
  if (!started) { x0 = x; started = true; }

  double y = runBiquad(hp1, x - x0);
  y = runBiquad(hp2, y);
  y = runBiquad(lp1, y);
  y = runBiquad(lp2, y);
  y = runBiquad(nt1, y);
  y = runBiquad(nt2, y);
  y = runBiquad(nt3, y);
  y = runBiquad(nt4, y);
  y = runBiquad(nt5, y);
  env += aEnv * (fabs(y) - env);

  if (n < warmEnd) return;
  if (n == warmEnd) { hi = env; lo = env; return; }

  hi += ((env > hi) ? aUp : aDown) * (env - hi);
  lo += ((env < lo) ? aUp : aDown) * (env - lo);
  if (hi > HI_CAP_RATIO * lo + MIN_RANGE) hi = HI_CAP_RATIO * lo + MIN_RANGE;
  double range = hi - lo;
  double thHi  = lo + TRIG_HI * range;
  double thLo  = lo + TRIG_LO * range;

  if (n % ENV_DEC == 0) {
    double e = env, cap = HI_CAP_RATIO * lo + MIN_RANGE;
    float v = (float)log(1.0 + ((e > cap) ? cap : e));
    if (n < blankUntil && envCount > 0) v = envBuf[(envHead + ENV_N - 1) % ENV_N];
    envBuf[envHead] = v;
    envM = n / ENV_DEC;
    envHead = (envHead + 1) % ENV_N;
    if (envCount < ENV_N) envCount++;
  }
  bool clear = (range >= MIN_RANGE) && (hi >= MIN_RATIO * lo);

  if (clearReq) { clearBeats(); clearReq = false; }
  if (env < thLo) belowLo = true;
  else if (belowLo) { belowLo = false; riseT = tSec; }
  if (env < thLo) armed = true;
  if (armed && clear && env > thHi && (tSec - lastBeatT) > refractory && n >= blankUntil) {
    double bt = (riseT > lastBeatT && tSec - riseT < 0.4) ? riseT : tSec;
    bool phaseOK = true;
    if (phaseValid) {
      double u = fmod(bt - phiRef, phiT) / phiT;
      if (u < 0) u += 1.0;
      phaseOK = (u < 0.35 || u > 0.80);
    }
    if (phaseOK) {
      lastBeatT = bt;
      beatT[beatHead] = bt; beatHead = (beatHead + 1) % MAX_BEATS;
      totalBeats++;
      armed = false;
    }
  }

  if (PRINT_PLOT && n % ENV_DEC == 0) {
    double u = 0;
    bool gateOpen = false;
    if (state == ST_READING && phaseValid) {
      u = fmod(tSec - phiRef, phiT) / phiT;
      if (u < 0) u += 1.0;
      gateOpen = (u < 0.35 || u > 0.80);
    }
    double top = (hi > 1.0) ? hi : 1.0;
    Serial.print("Env:");       Serial.print(env, 1);
    Serial.print(",ThHigh:");   Serial.print(thHi, 1);
    Serial.print(",ThLow:");    Serial.print(thLo, 1);
    Serial.print(",RedLED:");   Serial.print((state == ST_READING && !armed) ? top : 0.0, 1);
    Serial.print(",BeatAllowed:"); Serial.print(gateOpen ? 0.25 * top : 0.0, 1);
    Serial.print(",Connected:");   Serial.println(state == ST_READING ? 0.10 * top : 0.0, 1);
  }
  if (n % FS == 0) { secT = tSec; secPending = true; }
}
