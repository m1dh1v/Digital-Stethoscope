const int   MIC_PIN = 35;
const int   FS      = 2000;     // samples per second
const int   WIN     = 100;      // samples per printed point (100 / 2000 = 50 ms)
const float A_DC    = 0.01f;    // tracks the DC level (about 3 Hz at 2 kHz)
const float A_LP    = 0.376f;   // one-pole low-pass, about 150 Hz at 2 kHz

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);
}

void loop() {
  static uint32_t next = micros();
  static float dc = 0, lp = 0, accW = 0, accL = 0;
  static int n = 0;
  static bool started = false;

  while ((int32_t)(micros() - next) < 0) {}   // wait for the next sample tick
  next += 1000000UL / FS;

  float x = (float)analogRead(MIC_PIN);
  if (!started) { dc = lp = x; started = true; }

  dc += A_DC * (x - dc);          // DC level
  lp += A_LP * (x - lp);          // low-passed copy
  float wide = x - dc;            // AC part, all frequencies
  float low  = lp - dc;           // AC part, low frequencies only
  accW += wide * wide;
  accL += low * low;

  if (++n >= WIN) {
    Serial.print("Wide:");  Serial.print(sqrt(accW / n), 1);
    Serial.print(",Low:");  Serial.println(sqrt(accL / n), 1);
    n = 0; accW = 0; accL = 0;
  }
}