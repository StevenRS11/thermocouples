/*
  Thermolog v2.2 (ESP32-S3 + 6x MAX6675 + SD + SSD1306)

  - Sensors: Adafruit MAX6675 library (bit-banged 3-wire: SCK, CS, SO)
  - Display: Adafruit_SSD1306 (I2C), static UI + partial redraw for numbers
  - Sensor polling every 0.5 s; display refresh every 0.5 s
  - Logging ONLY every 5 s (configurable), append-only CSV + manifest
  - 60-second batching (based on log interval), flush() after each batch
  - Size-based rotation (~10 MB)
  - Single button to start/stop recording; LED indicates state
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <time.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>   // Adafruit MAX6675 (bit-banged 3-wire SPI)

// -------------------- I2C (OLED) --------------------
#define I2C_SDA         8
#define I2C_SCL         9

// -------------------- SD on hardware SPI (SPI) --------------------
#define SD_SCK          18
#define SD_MISO         19
#define SD_MOSI         23
#define SD_CS           5

// -------------------- Thermocouples (bit-banged) --------------------
#define TC_SCK          26     // shared SCK to all MAX6675
#define TC_SO           27     // shared SO/DO from MAX6675 to ESP32
const int TC_CS[6] = { 12, 13, 14, 15, 16, 17 };   // unique CS per board

// -------------------- UI pins --------------------
#define PIN_BUTTON      36     // active-LOW, uses INPUT_PULLUP
#define PIN_LED         2

// -------------------- Display (Adafruit) --------------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// -------------------- Cadence / Batching / Rotation --------------------
#define SENSOR_INTERVAL_MS   500     // <-- sensor read cadence (0.5 s)
#define DISPLAY_INTERVAL_MS  500     // <-- display refresh cadence (0.5 s)
#define LOG_INTERVAL_MS      5000    // <-- write to CSV every 5 s (make configurable)
#define BATCH_WINDOW_MS      60000   // flush once per minute by default

static const uint8_t  BATCH_MAX_LINES = (BATCH_WINDOW_MS / LOG_INTERVAL_MS); // ~12 at 5s
static const uint32_t ROTATE_BYTES    = 10UL * 1024UL * 1024UL;              // ~10 MB

// -------------------- App State --------------------
enum RecState { IDLE, RECORDING };
static RecState state = IDLE;

static File     logFile;
static char     sessionPath[128] = {0};
static char     sessionDir[64]   = {0};
static char     sessionFile[64]  = {0};

static uint32_t recStartMs    = 0;
static uint32_t lastSensorMs  = 0;
static uint32_t lastDisplayMs = 0;
static uint32_t lastLogMs     = 0;
static uint32_t lastBatchMs   = 0;

// Button debounce
static bool     lastBtnLevel        = HIGH;
static bool     lastStableBtnLevel  = HIGH;
static uint32_t lastBtnChangeMs     = 0;
static const uint32_t DEBOUNCE_MS   = 40;

// Fixed formatting buffers
static char lineFmtBuf[96];
static char smallBuf[48];

// Batch buffer (stores ONLY log lines, not every 0.5s sample)
static char    batchBuf[BATCH_MAX_LINES][96];
static uint8_t batchCount = 0;

// Latest temperatures snapshot for UI/logging
static float latestTemps[6] = {NAN, NAN, NAN, NAN, NAN, NAN};

// -------------------- MAX6675 objects (bit-banged) --------------------
MAX6675 tc0(TC_SCK, TC_CS[0], TC_SO);
MAX6675 tc1(TC_SCK, TC_CS[1], TC_SO);
MAX6675 tc2(TC_SCK, TC_CS[2], TC_SO);
MAX6675 tc3(TC_SCK, TC_CS[3], TC_SO);
MAX6675 tc4(TC_SCK, TC_CS[4], TC_SO);
MAX6675 tc5(TC_SCK, TC_CS[5], TC_SO);
MAX6675* TCs[6] = { &tc0, &tc1, &tc2, &tc3, &tc4, &tc5 };

// -------------------- Layout for partial redraw --------------------
static const int LABEL_COL[2] = { 0, 64 };      // two columns
static const int ROW_Y[3]     = { 12, 28, 44 }; // three rows (~16 px)
static const int NUM_X_OFFSET = 28;
static const int NUM_W        = 34;
static const int NUM_H        = 12;
static const int FOOTER_Y     = 56;

// -------------------- Forward Declarations --------------------
bool   initDisplay();
void   drawStaticUI();
void   updateDynamicUI(const float tempsC[6], bool recording, uint32_t elapsedMs);

bool   initSD();
bool   readMAX6675CelsiusIdx(int idx, float &outC);

bool   openNewSessionFile(bool isRotation);
void   closeSessionFile();
bool   appendLineToBatch(const float tempsC[6], uint32_t elapsedMs, uint32_t seq);
bool   flushBatchToFile();
void   rotateIfNeeded();

bool   buttonPressedEdge();
void   setRecording(bool on);

void   buildSessionPaths();
bool   hasRealTime(time_t *nowOut = nullptr);
void   formatDate(char *dst, size_t n, time_t t);
void   formatTime(char *dst, size_t n, time_t t);

// ======================================================================

void setup() {
  Serial.begin(115200);
  delay(100);

  // I2C (OLED)
  Wire.begin(I2C_SDA, I2C_SCL);
  initDisplay();
  drawStaticUI();

  // UI
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // SD bus (hardware SPI)
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  // Thermocouple pins
  for (int i = 0; i < 6; ++i) { pinMode(TC_CS[i], OUTPUT); digitalWrite(TC_CS[i], HIGH); }
  pinMode(TC_SCK, OUTPUT); digitalWrite(TC_SCK, LOW);
  pinMode(TC_SO, INPUT);

  uint32_t now = millis();
  lastSensorMs  = now;
  lastDisplayMs = now;
  lastLogMs     = now;
  lastBatchMs   = now;

  // Initial UI numbers/footers
  updateDynamicUI(latestTemps, false, 0);

  if (!initSD()) {
    Serial.println("SD init failed; realtime works but recording will fail.");
  }
}

void loop() {
  uint32_t now = millis();

  // --- Sensor poll @ 0.5 s ---
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    for (int i = 0; i < 6; ++i) {
      float t;
      if (!readMAX6675CelsiusIdx(i, t)) t = NAN;
      latestTemps[i] = t;
    }
  }

  // --- Display refresh @ 0.5 s ---
  if (now - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    uint32_t elapsed = (state == RECORDING) ? (now - recStartMs) : 0;
    updateDynamicUI(latestTemps, state == RECORDING, elapsed);
    lastDisplayMs = now;
  }

  // --- Button toggle ---
  if (buttonPressedEdge()) setRecording(state != RECORDING);

  // --- Logging only on LOG_INTERVAL_MS when recording ---
  if (state == RECORDING) {
    if (now - lastLogMs >= LOG_INTERVAL_MS) {
      lastLogMs = now;
      uint32_t seq = (now - recStartMs) / LOG_INTERVAL_MS;
      uint32_t elapsedMs = now - recStartMs;

      if (!appendLineToBatch(latestTemps, elapsedMs, seq)) {
        if (flushBatchToFile()) {
          appendLineToBatch(latestTemps, elapsedMs, seq);
        } else {
          Serial.println("Batch flush failed; stopping recording.");
          setRecording(false);
        }
      }
    }

    // flush batch every minute (or full)
    if ((now - lastBatchMs >= BATCH_WINDOW_MS) || (batchCount >= BATCH_MAX_LINES)) {
      if (!flushBatchToFile()) {
        Serial.println("Batch flush failed; stopping recording.");
        setRecording(false);
      } else {
        lastBatchMs = now;
        rotateIfNeeded();
      }
    }
  }

  delay(10);
}

// ======================================================================
// Display
// ======================================================================
bool initDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    return false;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("Thermolog v2.2"));
  display.println(F("Booting..."));
  display.display();
  delay(200);
  return true;
}

void drawStaticUI() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Header
  display.setCursor(0, 0);
  display.println(F("MAX6675 x6 (C)"));

  // Channel labels "T1:".."T6:"
  for (int i = 0; i < 6; ++i) {
    int col = LABEL_COL[i % 2];
    int row = ROW_Y[i / 2];
    display.setCursor(col, row);
    display.print("T");
    display.print(i + 1);
    display.print(":");
  }

  display.display();
}

void updateDynamicUI(const float tempsC[6], bool recording, uint32_t elapsedMs) {
  for (int i = 0; i < 6; ++i) {
    int col = LABEL_COL[i % 2];
    int row = ROW_Y[i / 2];

    int nx = col + NUM_X_OFFSET;
    int ny = row;
    display.fillRect(nx, ny, NUM_W, NUM_H, SSD1306_BLACK);

    display.setCursor(nx, ny);
    if (isnan(tempsC[i])) {
      display.print(F("---"));
    } else {
      char buf[16];
      dtostrf(tempsC[i], 6, 2, buf);
      display.print(buf);
    }
  }

  // Footer
  display.fillRect(0, FOOTER_Y, SCREEN_WIDTH, SCREEN_HEIGHT - FOOTER_Y, SSD1306_BLACK);
  display.setCursor(0, FOOTER_Y);
  if (recording) {
    uint32_t secs = elapsedMs / 1000;
    uint32_t mm = secs / 60;
    uint32_t ss = secs % 60;
    char tbuf[16];
    snprintf(tbuf, sizeof(tbuf), "%02lu:%02lu", (unsigned long)mm, (unsigned long)ss);
    display.print(F("REC "));
    display.print(tbuf);
  } else {
    display.print(F("IDLE"));
  }

  display.display();
}

// ======================================================================
// SD / Session / CSV (append-only, manifest, batching on log cadence)
// ======================================================================
bool initSD() {
  if (!SD.begin(SD_CS, SPI, 25000000)) {
    if (!SD.begin(SD_CS, SPI, 8000000)) return false;
  }
  return true;
}

bool hasRealTime(time_t *nowOut) {
  time_t now = time(nullptr);
  if (nowOut) *nowOut = now;
  return now > 1609459200; // after Jan 1, 2021
}

void formatDate(char *dst, size_t n, time_t t) {
  struct tm tmv; localtime_r(&t, &tmv);
  strftime(dst, n, "%Y-%m-%d", &tmv);
}

void formatTime(char *dst, size_t n, time_t t) {
  struct tm tmv; localtime_r(&t, &tmv);
  strftime(dst, n, "%H-%M-%S", &tmv);
}

void buildSessionPaths() {
  strcpy(sessionDir, "/logs");
  time_t now;
  if (hasRealTime(&now)) {
    char dateStr[16], timeStr[16];
    formatDate(dateStr, sizeof(dateStr), now);
    formatTime(timeStr, sizeof(timeStr), now);
    snprintf(sessionDir, sizeof(sessionDir), "/logs/%s", dateStr);
    snprintf(sessionFile, sizeof(sessionFile), "SESSION_%s.csv", timeStr);
  } else {
    uint32_t ms = millis();
    strcpy(sessionDir, "/logs");
    snprintf(sessionFile, sizeof(sessionFile), "SESSION_ms%lu.csv", (unsigned long)ms);
  }
  if (!SD.exists(sessionDir)) SD.mkdir(sessionDir);
  snprintf(sessionPath, sizeof(sessionPath), "%s/%s", sessionDir, sessionFile);
}

static const char *CSV_HEADER = "elapsed_s,seq,T1_C,T2_C,T3_C,T4_C,T5_C,T6_C";

bool openNewSessionFile(bool isRotation) {
  buildSessionPaths();

  if (isRotation) {
    for (int i = 1; i <= 99; ++i) {
      snprintf(smallBuf, sizeof(smallBuf), "%s/%.*s_%02d.csv",
               sessionDir, (int)strlen(sessionFile) - 4, sessionFile, i);
      if (!SD.exists(smallBuf)) { strncpy(sessionPath, smallBuf, sizeof(sessionPath)); sessionPath[sizeof(sessionPath)-1]='\0'; break; }
    }
  }

  logFile = SD.open(sessionPath, FILE_WRITE);
  if (!logFile) return false;

  // Manifest header
  time_t now;
  bool timeOk = hasRealTime(&now);
  logFile.println(F("# manifest_begin"));
  logFile.println(F("# firmware: Thermolog v2.2"));
  if (timeOk) {
    char dateStr[16], timeStr[16];
    formatDate(dateStr, sizeof(dateStr), now);
    formatTime(timeStr, sizeof(timeStr), now);
    snprintf(lineFmtBuf, sizeof(lineFmtBuf), "# start_date: %s", dateStr); logFile.println(lineFmtBuf);
    snprintf(lineFmtBuf, sizeof(lineFmtBuf), "# start_time: %s", timeStr); logFile.println(lineFmtBuf);
  } else {
    logFile.println(F("# start_time: unknown (no RTC/NTP)"));
  }
  snprintf(lineFmtBuf, sizeof(lineFmtBuf), "# sample_period_s: %.3f", SENSOR_INTERVAL_MS / 1000.0);
  logFile.println(lineFmtBuf);
  snprintf(lineFmtBuf, sizeof(lineFmtBuf), "# log_period_s: %.3f", LOG_INTERVAL_MS / 1000.0);
  logFile.println(lineFmtBuf);
  logFile.println(F("# channels: T1_C,T2_C,T3_C,T4_C,T5_C,T6_C"));
  logFile.println(F("# units: C"));
  logFile.println(F("# csv_header: elapsed_s,seq,T1_C,T2_C,T3_C,T4_C,T5_C,T6_C"));
  logFile.println(F("# manifest_end"));

  // CSV header
  logFile.println(CSV_HEADER);
  logFile.flush();

  batchCount  = 0;
  lastBatchMs = millis();

  Serial.print("Opened: "); Serial.println(sessionPath);
  return true;
}

void closeSessionFile() {
  if (logFile) { logFile.flush(); logFile.close(); }
}

bool appendLineToBatch(const float tempsC[6], uint32_t elapsedMs, uint32_t seq) {
  if (batchCount >= BATCH_MAX_LINES) return false;
  float elapsedS = elapsedMs / 1000.0f;
  int n = snprintf(lineFmtBuf, sizeof(lineFmtBuf), "%.2f,%lu,", elapsedS, (unsigned long)seq);
  for (int i = 0; i < 6; ++i) {
    if (isnan(tempsC[i])) n += snprintf(lineFmtBuf + n, sizeof(lineFmtBuf) - n, "NaN");
    else                  n += snprintf(lineFmtBuf + n, sizeof(lineFmtBuf) - n, "%.2f", tempsC[i]);
    if (i < 5)            n += snprintf(lineFmtBuf + n, sizeof(lineFmtBuf) - n, ",");
  }
  n += snprintf(lineFmtBuf + n, sizeof(lineFmtBuf) - n, "\n");
  strncpy(batchBuf[batchCount], lineFmtBuf, sizeof(batchBuf[0]) - 1);
  batchBuf[batchCount][sizeof(batchBuf[0]) - 1] = '\0';
  batchCount++;
  return true;
}

bool flushBatchToFile() {
  if (!logFile) return false;
  if (batchCount == 0) return true;
  for (uint8_t i = 0; i < batchCount; ++i) {
    if (logFile.print(batchBuf[i]) != (int)strlen(batchBuf[i])) return false;
  }
  logFile.flush();
  batchCount = 0;
  return true;
}

void rotateIfNeeded() {
  if (!logFile) return;
  uint32_t sz = logFile.size();
  if (sz < ROTATE_BYTES) return;
  logFile.flush();
  logFile.close();
  if (!openNewSessionFile(true)) {
    Serial.println("Rotation failed: could not open new file.");
  } else {
    Serial.println("Rotated log file.");
  }
}

// ======================================================================
// MAX6675 helper (Adafruit lib)
// ======================================================================
bool readMAX6675CelsiusIdx(int idx, float &outC) {
  float t = TCs[idx]->readCelsius();  // NAN on open-circuit/fault
  if (!isnan(t) && (t < -10.0f || t > 1100.0f)) { outC = NAN; return true; }
  outC = t;
  return true;
}

// ======================================================================
// Button / Recording
// ======================================================================
bool buttonPressedEdge() {
  bool lvl = digitalRead(PIN_BUTTON);
  uint32_t now = millis();
  if (lvl != lastBtnLevel) { lastBtnLevel = lvl; lastBtnChangeMs = now; }
  if ((now - lastBtnChangeMs) > DEBOUNCE_MS) {
    if (lvl != lastStableBtnLevel) {
      lastStableBtnLevel = lvl;
      if (lvl == LOW) return true; // active-low press
    }
  }
  return false;
}

void setRecording(bool on) {
  if (on) {
    if (!SD.begin(SD_CS, SPI)) {
      Serial.println("SD not ready; cannot start recording.");
      digitalWrite(PIN_LED, LOW);
      state = IDLE; return;
    }
    if (!openNewSessionFile(false)) {
      Serial.println("Failed to open session file.");
      digitalWrite(PIN_LED, LOW);
      state = IDLE; return;
    }
    uint32_t now = millis();
    recStartMs    = now;
    lastLogMs     = now;
    lastBatchMs   = now;
    batchCount    = 0;

    digitalWrite(PIN_LED, HIGH);
    state = RECORDING;
    Serial.print("Recording → "); Serial.println(sessionPath);
  } else {
    if (!flushBatchToFile()) {
      Serial.println("Warning: flush failed on stop; last batch may be lost.");
    }
    closeSessionFile();
    digitalWrite(PIN_LED, LOW);
    state = IDLE;
    Serial.println("Recording stopped.");
  }
}
