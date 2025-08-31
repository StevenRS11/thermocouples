/*
  Thermolog v2.1 (ESP32-S3 + up to 6x MAX6675 + optional SD + SSD1306)

  NEW:
  - Configurable # of thermocouples: NUM_TC (1..6)
  - Toggle SD logging: ENABLE_SD_LOGGING (1=on, 0=off; no SD init when off)

  Notes:
  - Thermocouples use Adafruit MAX6675 library (bit-banged 3-wire: SCK, CS, SO)
  - SD logging (when enabled) uses hardware SPI (SPI) with:
      * 60 s batching (5 s samples)
      * append-only CSV + manifest header
      * size-based rotation (~10 MB)
  - One button toggles recording timer and logging (if enabled); LED shows state
  - OLED shows live temps and REC timer
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>

// ===================== USER CONFIG =====================
#define NUM_TC              6      // 1..6 thermocouples connected
#define ENABLE_SD_LOGGING   1      // 1 = log to microSD, 0 = no logging (no SD init)
// =======================================================

// -------------------- I2C (OLED) --------------------
#define I2C_SDA         8
#define I2C_SCL         9

// -------------------- SD on hardware SPI (SPI) --------------------
#if ENABLE_SD_LOGGING
  #include <SD.h>
  #define SD_SCK        18
  #define SD_MISO       19
  #define SD_MOSI       23
  #define SD_CS         5
#endif

// -------------------- Thermocouples (Adafruit MAX6675) ------------
#define TC_SCK          26     // shared SCK to all MAX6675
#define TC_SO           27     // shared SO/DO from all MAX6675 to ESP32 (one pin)
// Unique CS per thermocouple breakout (define up to 6; only first NUM_TC are used)
const int TC_CS_pins[6] = { 12, 13, 14, 15, 16, 17 };

// -------------------- UI pins --------------------
#define PIN_BUTTON      36     // active-LOW, uses INPUT_PULLUP
#define PIN_LED         2

// -------------------- Display (Adafruit) --------------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// -------------------- Sampling / Batching / Rotation --------------------
static const uint32_t SAMPLE_INTERVAL_MS = 5000;    // every 5 s
static const uint32_t BATCH_WINDOW_MS    = 60000;   // flush every 60 s
static const uint8_t  BATCH_MAX_LINES    = BATCH_WINDOW_MS / SAMPLE_INTERVAL_MS; // 12
#if ENABLE_SD_LOGGING
  static const uint32_t ROTATE_BYTES     = 10UL * 1024UL * 1024UL;               // ~10 MB
  File     logFile;
  char     sessionPath[128] = {0};
  char     sessionDir[64]   = {0};
  char     sessionFile[64]  = {0};
#endif

// -------------------- App State --------------------
enum RecState { IDLE, RECORDING };
static RecState state = IDLE;

static uint32_t appStartMs   = 0;
static uint32_t recStartMs   = 0;
static uint32_t lastSampleMs = 0;
static uint32_t lastBatchMs  = 0;

// Button debounce
static bool     lastBtnLevel        = HIGH;
static bool     lastStableBtnLevel  = HIGH;
static uint32_t lastBtnChangeMs     = 0;
static const uint32_t DEBOUNCE_MS   = 40;

// Buffers
static char lineFmtBuf[160];
static char smallBuf[64];

// Batch buffer
static char    batchBuf[BATCH_MAX_LINES][160];
static uint8_t batchCount = 0;

// -------------------- MAX6675 objects (bit-banged) --------------------
MAX6675* TCs[6] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };
MAX6675  tcObjs[6] = {
  MAX6675(TC_SCK, TC_CS_pins[0], TC_SO),
  MAX6675(TC_SCK, TC_CS_pins[1], TC_SO),
  MAX6675(TC_SCK, TC_CS_pins[2], TC_SO),
  MAX6675(TC_SCK, TC_CS_pins[3], TC_SO),
  MAX6675(TC_SCK, TC_CS_pins[4], TC_SO),
  MAX6675(TC_SCK, TC_CS_pins[5], TC_SO),
};

// -------------------- Forward Declarations --------------------
bool   initDisplay();
void   drawScreen(const float tempsC[], int nCh, bool recording, uint32_t elapsedMs);

#if ENABLE_SD_LOGGING
bool   initSD();
void   buildSessionPaths();
bool   hasRealTime(time_t *nowOut = nullptr);
void   formatDate(char *dst, size_t n, time_t t);
void   formatTime(char *dst, size_t n, time_t t);
bool   openNewSessionFile(bool isRotation);
void   closeSessionFile();
bool   flushBatchToFile();
void   rotateIfNeeded();
#endif

bool   appendLineToBatch(const float tempsC[], int nCh, uint32_t elapsedMs, uint32_t seq);

bool   buttonPressedEdge();
void   setRecording(bool on);

bool   readMAX6675CelsiusIdx(int idx, float &outC);

// ======================================================================

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Thermolog v2.1 starting...");

  // I2C (OLED)
  Wire.begin(I2C_SDA, I2C_SCL);
  initDisplay();

  // UI
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Thermocouple shared pins and per-channel CS pins
  pinMode(TC_SCK, OUTPUT);
  digitalWrite(TC_SCK, LOW);
  pinMode(TC_SO, INPUT);
  for (int i = 0; i < 6; ++i) {
    pinMode(TC_CS_pins[i], OUTPUT);
    digitalWrite(TC_CS_pins[i], HIGH); // idle high
  }

  // Bind pointers for the number of channels in use
  int use = constrain(NUM_TC, 1, 6);
  for (int i = 0; i < use; ++i) TCs[i] = &tcObjs[i];

  // Optional SD init
  #if ENABLE_SD_LOGGING
    // SD bus (hardware SPI)
    SPI.begin(18, 19, 23, SD_CS);
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);

    if (!initSD()) {
      Serial.println("SD init failed; logging will be disabled this run.");
    }
  #endif

  appStartMs   = millis();
  lastSampleMs = appStartMs;
  lastBatchMs  = appStartMs;
  state        = IDLE;

  // Splash
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Thermolog v2.1"));
  #if ENABLE_SD_LOGGING
    display.println(F("SD logging: ON"));
  #else
    display.println(F("SD logging: OFF"));
  #endif
  display.println(F("Press BTN to start"));
  display.display();
}

void loop() {
  const int nCh = constrain(NUM_TC, 1, 6);

  // Read all temps via Adafruit library
  float tempsC[6];
  for (int i = 0; i < nCh; ++i) {
    if (!readMAX6675CelsiusIdx(i, tempsC[i])) tempsC[i] = NAN;
  }

  // UI
  uint32_t elapsed = (state == RECORDING) ? (millis() - recStartMs) : 0;
  drawScreen(tempsC, nCh, state == RECORDING, elapsed);

  // Button
  if (buttonPressedEdge()) setRecording(state != RECORDING);

  const uint32_t now = millis();

  // Sampling & batching
  if (state == RECORDING) {
    if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
      lastSampleMs = now;
      uint32_t seq = (now - recStartMs) / SAMPLE_INTERVAL_MS;
      uint32_t elapsedMs = now - recStartMs;

      if (!appendLineToBatch(tempsC, nCh, elapsedMs, seq)) {
        #if ENABLE_SD_LOGGING
          if (flushBatchToFile()) {
            appendLineToBatch(tempsC, nCh, elapsedMs, seq);
          } else {
            Serial.println("Batch flush failed; stopping recording.");
            setRecording(false);
          }
        #endif
      }
    }

    #if ENABLE_SD_LOGGING
      if ((now - lastBatchMs >= BATCH_WINDOW_MS) || (batchCount >= BATCH_MAX_LINES)) {
        if (!flushBatchToFile()) {
          Serial.println("Batch flush failed; stopping recording.");
          setRecording(false);
        } else {
          lastBatchMs = now;
          rotateIfNeeded();
        }
      }
    #endif
  }

  delay(40);
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
  display.println(F("Thermolog v2.1"));
  display.println(F("Booting..."));
  display.display();
  delay(200);
  return true;
}

void drawScreen(const float tempsC[], int nCh, bool recording, uint32_t elapsedMs) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Header
  display.setCursor(0, 0);
  display.print(F("MAX6675 x"));
  display.print(nCh);
  display.println(F(" (C)"));

  // Grid: up to 3 rows x 2 cols
  for (int i = 0; i < nCh; ++i) {
    int col = (i % 2) * 64;
    int row = 12 + (i / 2) * 16;
    display.setCursor(col, row);

    if (isnan(tempsC[i])) {
      snprintf(lineFmtBuf, sizeof(lineFmtBuf), "T%d: ---", i + 1);
    } else {
      snprintf(lineFmtBuf, sizeof(lineFmtBuf), "T%d:%6.2f", i + 1, tempsC[i]);
    }
    display.print(lineFmtBuf);
  }

  // Footer with uptime
  display.setCursor(0, 56);
  uint32_t up = millis() / 1000;
  if (recording) {
    uint32_t secs = elapsedMs / 1000;
    uint32_t mm = secs / 60, ss = secs % 60;
    snprintf(lineFmtBuf, sizeof(lineFmtBuf), "REC %02lu:%02lu | UP %lus",
             (unsigned long)mm, (unsigned long)ss, (unsigned long)up);
  } else {
    snprintf(lineFmtBuf, sizeof(lineFmtBuf), "IDLE | UP %lus", (unsigned long)up);
  }
  display.print(lineFmtBuf);

  display.display();
}

// ======================================================================
// MAX6675 helper
// ======================================================================
bool readMAX6675CelsiusIdx(int idx, float &outC) {
  if (idx < 0 || idx >= NUM_TC || TCs[idx] == nullptr) { outC = NAN; return false; }
  float t = TCs[idx]->readCelsius();       // NAN on open/fault
  if (!isnan(t) && (t < -10.0f || t > 1100.0f)) t = NAN; // sanity clamp
  outC = t;
  return true;
}

// ======================================================================
// SD / Session / CSV (only when ENABLE_SD_LOGGING == 1)
// ======================================================================
#if ENABLE_SD_LOGGING

bool initSD() {
  if (!SD.begin(SD_CS, SPI, 25000000)) {
    if (!SD.begin(SD_CS, SPI, 8000000)) return false;
  }
  return true;
}

bool hasRealTime(time_t *nowOut) {
  time_t now = time(nullptr);
  if (nowOut) *nowOut = now;
  return now > 1609459200; // after 2021-01-01
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
  char sessionDirLocal[64] = "/logs";
  time_t now;
  if (hasRealTime(&now)) {
    char dateStr[16], timeStr[16];
    formatDate(dateStr, sizeof(dateStr), now);
    formatTime(timeStr, sizeof(timeStr), now);
    snprintf(sessionDir, sizeof(sessionDir), "/logs/%s", dateStr);
    snprintf(sessionFile, sizeof(sessionFile), "SESSION_%s.csv", timeStr);
  } else {
    strcpy(sessionDir, "/logs");
    snprintf(sessionFile, sizeof(sessionFile), "SESSION_ms%lu.csv", (unsigned long)millis());
  }

  if (!SD.exists(sessionDir)) SD.mkdir(sessionDir);
  snprintf(sessionPath, sizeof(sessionPath), "%s/%s", sessionDir, sessionFile);
}

static void buildCsvHeader(int nCh, char *dst, size_t n) {
  // elapsed_s,seq,T1_C,...,TN_C
  size_t pos = 0;
  pos += snprintf(dst + pos, n - pos, "elapsed_s,seq");
  for (int i = 1; i <= nCh; ++i) {
    pos += snprintf(dst + pos, n - pos, ",T%d_C", i);
  }
}

bool openNewSessionFile(bool isRotation) {
  buildSessionPaths();

  if (isRotation) {
    for (int i = 1; i <= 99; ++i) {
      snprintf(smallBuf, sizeof(smallBuf), "%s/%.*s_%02d.csv",
               sessionDir, (int)strlen(sessionFile) - 4, sessionFile, i);
      if (!SD.exists(smallBuf)) { strncpy(sessionPath, smallBuf, sizeof(sessionPath)-1); break; }
    }
  }

  logFile = SD.open(sessionPath, FILE_WRITE);
  if (!logFile) return false;

  const int nCh = constrain(NUM_TC, 1, 6);

  // Manifest header
  time_t now;
  bool timeOk = hasRealTime(&now);

  logFile.println(F("# manifest_begin"));
  logFile.println(F("# firmware: Thermolog v2.1"));
  snprintf(lineFmtBuf, sizeof(lineFmtBuf), "# channels_count: %d", nCh); logFile.println(lineFmtBuf);
  if (timeOk) {
    char dateStr[16], timeStr[16];
    formatDate(dateStr, sizeof(dateStr), now);
    formatTime(timeStr, sizeof(timeStr), now);
    snprintf(lineFmtBuf, sizeof(lineFmtBuf), "# start_date: %s", dateStr); logFile.println(lineFmtBuf);
    snprintf(lineFmtBuf, sizeof(lineFmtBuf), "# start_time: %s", timeStr); logFile.println(lineFmtBuf);
  } else {
    logFile.println(F("# start_time: unknown (no RTC/NTP)"));
  }
  snprintf(lineFmtBuf, sizeof(lineFmtBuf), "# sample_period_s: %.2f", SAMPLE_INTERVAL_MS / 1000.0);
  logFile.println(lineFmtBuf);

  // Channels list
  lineFmtBuf[0] = 0;
  size_t pos = snprintf(lineFmtBuf, sizeof(lineFmtBuf), "# channels: ");
  for (int i = 1; i <= nCh; ++i) {
    pos += snprintf(lineFmtBuf + pos, sizeof(lineFmtBuf) - pos, "T%d_C%s", i, (i < nCh ? "," : ""));
  }
  logFile.println(lineFmtBuf);

  logFile.println(F("# units: C"));

  // CSV header line
  buildCsvHeader(nCh, lineFmtBuf, sizeof(lineFmtBuf));
  snprintf(smallBuf, sizeof(smallBuf), "# csv_header: %s", lineFmtBuf);
  logFile.println(smallBuf);

  logFile.println(F("# manifest_end"));

  // Write the actual CSV header
  logFile.println(lineFmtBuf);
  logFile.flush();

  batchCount  = 0;
  lastBatchMs = millis();

  Serial.print("Opened: ");
  Serial.println(sessionPath);
  return true;
}

void closeSessionFile() {
  if (logFile) {
    logFile.flush();
    logFile.close();
  }
}

bool flushBatchToFile() {
  if (!logFile) return false;
  if (batchCount == 0) return true;

  for (uint8_t i = 0; i < batchCount; ++i) {
    if (logFile.print(batchBuf[i]) != (int)strlen(batchBuf[i])) {
      return false;
    }
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
#endif // ENABLE_SD_LOGGING

// ======================================================================
// CSV batching (works both with and without SD enabled; no writes if off)
// ======================================================================
bool appendLineToBatch(const float tempsC[], int nCh, uint32_t elapsedMs, uint32_t seq) {
  if (batchCount >= BATCH_MAX_LINES) return false;

  float elapsedS = elapsedMs / 1000.0f;
  int n = snprintf(lineFmtBuf, sizeof(lineFmtBuf), "%.2f,%lu", elapsedS, (unsigned long)seq);

  for (int i = 0; i < nCh; ++i) {
    n += snprintf(lineFmtBuf + n, sizeof(lineFmtBuf) - n, ",");
    if (isnan(tempsC[i])) n += snprintf(lineFmtBuf + n, sizeof(lineFmtBuf) - n, "NaN");
    else                  n += snprintf(lineFmtBuf + n, sizeof(lineFmtBuf) - n, "%.2f", tempsC[i]);
  }
  n += snprintf(lineFmtBuf + n, sizeof(lineFmtBuf) - n, "\n");

  strncpy(batchBuf[batchCount], lineFmtBuf, sizeof(batchBuf[0]) - 1);
  batchBuf[batchCount][sizeof(batchBuf[0]) - 1] = '\0';
  batchCount++;
  return true;
}

// ======================================================================
// Button / Recording
// ======================================================================
bool buttonPressedEdge() {
  bool lvl = digitalRead(PIN_BUTTON);
  uint32_t now = millis();

  if (lvl != lastBtnLevel) {
    lastBtnLevel = lvl;
    lastBtnChangeMs = now;
  }

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
    #if ENABLE_SD_LOGGING
      if (!SD.begin(SD_CS, SPI)) {
        Serial.println("SD not ready; logging disabled this session.");
      }
      openNewSessionFile(false); // if SD is bad, calls will just fail quietly later
    #endif

    recStartMs   = millis();
    lastSampleMs = recStartMs;
    lastBatchMs  = recStartMs;
    batchCount   = 0;

    digitalWrite(PIN_LED, HIGH);
    state = RECORDING;
    Serial.println("Recording started (timer active).");
  } else {
    #if ENABLE_SD_LOGGING
      if (!flushBatchToFile()) {
        Serial.println("Warning: flush failed on stop; last batch may be lost.");
      }
      closeSessionFile();
    #endif
    digitalWrite(PIN_LED, LOW);
    state = IDLE;
    Serial.println("Recording stopped.");
  }
}
