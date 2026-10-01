/*
 * ============================================================
 *  EchoShield ANC — ESP32 Firmware  (v3 - Race Condition Fixed)
 * ============================================================
 *  Fix v3:  Double buffer to prevent ISR overwriting FFT data
 *           Normalize FFT input to prevent magnitude overflow
 *           Fixed dB calculation
 *  Wiring:
 *    MAX9814 OUT → GPIO34 (ADC)
 *    GPIO25      → 1µF cap → PAM8403 IN → Speaker
 * ============================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <arduinoFFT.h>
#include <driver/dac.h>

// ------------ USER CONFIG ------------
const char* ssid       = "Boss";
const char* pass       = "11111111";
const char* SERVER_URL = "http://10.71.233.56:3000/api/readings";

// ------------ SIGNAL / PINS ------------
#define SAMPLES        256
#define SAMPLING_FREQ  8000
#define ADC_PIN        34
#define DAC_PIN        25

// ------------ DOUBLE BUFFER (fixes race condition) ------------
// Buffer A — ISR writes here
volatile int  rawBufA[SAMPLES];
volatile int  rawBufIdx  = 0;
volatile bool bufAReady  = false;

// Buffer B — main loop processes this (copied from A safely)
double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

// ------------ TIMERS ------------
hw_timer_t* sampleTimer = NULL;
hw_timer_t* sineTimer   = NULL;
portMUX_TYPE timerMux   = portMUX_INITIALIZER_UNLOCKED;

// ------------ STATE ------------
bool   ancEnabled    = true;
double lastDb        = 0.0;
double lastPeakFreq  = 0.0;
double antiAmplitude = 0.0;

// Sine LUT — inverted phase for anti-noise
#define SINE_TABLE_SIZE 256
uint8_t sineTable[SINE_TABLE_SIZE];
volatile bool     outputActive = false;
volatile uint32_t sinePhase    = 0;

unsigned long lastPostMs = 0;

// ============================================================
//  ISR 1 — Sample ADC at 8 kHz into raw buffer A
// ============================================================
void IRAM_ATTR onSampleTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (!bufAReady && rawBufIdx < SAMPLES) {
    rawBufA[rawBufIdx++] = analogRead(ADC_PIN);
    if (rawBufIdx >= SAMPLES) {
      bufAReady  = true;
      rawBufIdx  = 0;
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ============================================================
//  ISR 2 — Output anti-noise sine wave via DAC
// ============================================================
void IRAM_ATTR onSineTimer() {
  if (!outputActive) {
    dac_output_voltage(DAC_CHANNEL_1, 128);
    return;
  }
  uint32_t idx = sinePhase & (SINE_TABLE_SIZE - 1);
  dac_output_voltage(DAC_CHANNEL_1, sineTable[idx]);
  sinePhase++;
}

// ============================================================
//  Rebuild sine LUT with given amplitude (0.0 – 1.0)
// ============================================================
void buildSineTable(double amp) {
  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    double s = sin(2.0 * PI * i / (double)SINE_TABLE_SIZE);
    int v = (int)(128.0 - s * amp * 127.0);  // inverted phase
    sineTable[i] = (uint8_t)constrain(v, 0, 255);
  }
}

// ============================================================
//  Compute RMS dB from normalised buffer (-1..+1)
// ============================================================
double computeDb(double* buf, int n) {
  double sumSq = 0.0;
  for (int i = 0; i < n; i++) sumSq += buf[i] * buf[i];
  double rms = sqrt(sumSq / n);
  if (!isfinite(rms) || rms <= 0.0001) return 0.0;
  double db = 20.0 * log10(rms) + 90.0;
  return constrain(db, 0.0, 120.0);
}

// ============================================================
//  Post to MongoDB backend
// ============================================================
void postToMongoDB(double db, double peakHz, double amplitude) {
  if (WiFi.status() != WL_CONNECTED) return;
  unsigned long now = millis();
  if (now - lastPostMs < 1000) return;
  lastPostMs = now;

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(600);

  StaticJsonDocument<200> doc;
  doc["dbSPL"]      = (int)db;
  doc["peakHz"]     = (int)peakHz;
  doc["amplitude"]  = amplitude;
  doc["ancEnabled"] = ancEnabled;
  doc["status"]     = outputActive ? "anc_active" : "anc_idle";

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  Serial.printf("[HTTP] POST → dB=%.1f Hz=%.1f code=%d\n", db, peakHz, code);
  http.end();
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== EchoShield ANC v3 Starting ===");

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Serial.print("[WiFi] Connecting");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    delay(300); Serial.print("."); retries++;
  }
  Serial.println(WiFi.status() == WL_CONNECTED
    ? "\n[WiFi] Connected → " + WiFi.localIP().toString()
    : "\n[WiFi] Offline — ANC still works");

  // ADC
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
  pinMode(ADC_PIN, INPUT);

  // Sine LUT (start at full amplitude)
  buildSineTable(1.0);

  // DAC
  dac_output_enable(DAC_CHANNEL_1);

  // Sample timer — 8 kHz
  sampleTimer = timerBegin(1000000);
  timerAttachInterrupt(sampleTimer, &onSampleTimer);
  timerAlarm(sampleTimer, 125, true, 0);
  timerStart(sampleTimer);

  // Sine timer — start at 1 kHz placeholder
  sineTimer = timerBegin(1000000);
  timerAttachInterrupt(sineTimer, &onSineTimer);
  timerAlarm(sineTimer, 1000, true, 0);
  timerStart(sineTimer);

  Serial.println("[ANC] Ready. Sampling at 8 kHz on GPIO34.");
  Serial.println("[ANC] Anti-noise output on GPIO25.");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {

  // WiFi reconnect
  static unsigned long lastReconnect = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - lastReconnect > 10000) {
    lastReconnect = millis();
    WiFi.reconnect();
  }

  // Wait for full buffer
  if (!bufAReady) { delay(1); return; }

  // ── CRITICAL: copy raw buffer to working buffer, then free ISR ──
  // This prevents ISR from overwriting data while FFT runs
  int localBuf[SAMPLES];
  portENTER_CRITICAL(&timerMux);
  for (int i = 0; i < SAMPLES; i++) localBuf[i] = rawBufA[i];
  bufAReady = false;   // ISR can now fill buffer again
  portEXIT_CRITICAL(&timerMux);

  // ── DC removal + normalize to -1..+1 ──
  double mean = 0.0;
  for (int i = 0; i < SAMPLES; i++) mean += localBuf[i];
  mean /= SAMPLES;

  double minV = 4095, maxV = 0;
  for (int i = 0; i < SAMPLES; i++) {
    double v = localBuf[i] - mean;
    vReal[i] = v / 2048.0;   // normalize: -1.0 to +1.0
    vImag[i] = 0.0;
    if (localBuf[i] < minV) minV = localBuf[i];
    if (localBuf[i] > maxV) maxV = localBuf[i];
  }

  // ── Noise floor check ──
  if ((maxV - minV) < 20) {
    outputActive = false;
    lastDb = 0.0; lastPeakFreq = 0.0;
    Serial.println("[ANC] Quiet — no output");
    return;
  }

  // ── Compute dB from normalised signal ──
  lastDb = computeDb(vReal, SAMPLES);

  // ── FFT ──
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // ── Find peak frequency (skip DC bins 0-1) ──
  double peakMag   = 0.0;
  int    peakIndex = 0;
  for (int i = 2; i < SAMPLES / 2; i++) {
    if (vReal[i] > peakMag) {
      peakMag   = vReal[i];
      peakIndex = i;
    }
  }
  lastPeakFreq = (double)peakIndex * ((double)SAMPLING_FREQ / (double)SAMPLES);

  // ── Anti-noise amplitude (normalised mag, max ~128 for unit input) ──
  double norm = peakMag / 100.0;
  if (norm > 1.0) norm = 1.0;
  antiAmplitude = norm * 0.9;

  // Rebuild sine table with new amplitude
  buildSineTable(antiAmplitude);

  // ── Update sine timer for peak frequency ──
  if (lastPeakFreq > 20 &&
      lastPeakFreq < (SAMPLING_FREQ / 2.0) &&
      ancEnabled &&
      antiAmplitude > 0.02) {

    double stepUs = 1000000.0 / (lastPeakFreq * (double)SINE_TABLE_SIZE);
    if (stepUs < 20.0) stepUs = 20.0;
    timerAlarm(sineTimer, (uint64_t)stepUs, true, 0);
    outputActive = true;
  } else {
    outputActive = false;
  }

  // ── Serial output ──
  Serial.printf("[ANC] dB=%.1f  Hz=%.1f  Mag=%.2f  Amp=%.2f  Out=%s\n",
    lastDb, lastPeakFreq, peakMag, antiAmplitude,
    outputActive ? "ON" : "OFF");

  // ── Post to MongoDB ──
  postToMongoDB(lastDb, lastPeakFreq, antiAmplitude);

  delay(5);
}
