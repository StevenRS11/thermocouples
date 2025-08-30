/*
  Thermolog v2.0 (ESP32-S3 + 6x MAX6675 + SD + SSD1306)

  - Thermocouples: Adafruit MAX6675 library (bit-banged 3-wire: SCK, CS, SO)
    * No hardware SPI class used for thermocouples; avoids SD bus contention.
  - SD logging on hardware SPI (SPI) with append-only CSV, manifest header.
  - 60-second batching (samples every 5 s), flush() after each batch.
  - Size-based rotation (~10 MB per file).
  - Adafruit_SSD1306 (I2C) for live readout + recording timer.
  - One button toggles recording; LED indicates state.

  Adjust pin defines to your wiring.
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

// -------------------- Thermocouples (bit-banged via Adafruit lib) ----
// Library uses pins directly: MAX6675(SCK, CS, SO)
#define TC_SCK          20     // shared SCK to all MAX6675
#define TC_SO           21     // shared SO/DO from MAX6675 to ESP32
// Unique CS per thermocouple breakout:
const int TC_CS[6] = { 12, 13, 14, 15, 16, 17 };

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
static const uint32_t ROTATE_BYTES       = 10UL * 1024UL * 1024UL;               // ~10 MB

// -------------------- App State --------------------
enum RecState { IDLE, RECORDING };
static RecState state = IDLE;

static File     logFile;
static char     sessionPath[128] = {0};
static char     sessionDir[64]   = {0};
static char     sessionFile[64]  = {0};

static uint32_t appStartMs   = 0;
static uint32_t recStartMs   = 0;
static uint32_t lastSampleMs = 0;
static uint32_t lastBatchMs  = 0;

// Button debounce
static bool     lastBtnLevel        = HIGH;
static bool     lastStableBtnLevel  = HIGH;
static uint32_t lastBtnChangeMs     = 0;
static const uint32_t DEBOUNCE_MS   = 40;

// Fixed formatting buffers
static char lineFmtBuf[96];
static char smallBuf[48];

// Batch buffer
static char    batchBuf[BATCH_MAX_LINES][96];
static uint8_t batchCount = 0;

// -------------------- MAX6675 objects (bit-banged) --------------------
MAX6675 tc0(TC_SCK, TC_CS[0], TC_SO);
MAX6675 tc1(TC_SCK, TC_CS[1], TC_SO);
MAX6675 tc2(TC_SCK, TC_CS[2], TC_SO);
MAX6675 tc3(TC_SCK, TC_CS[3], TC_SO);
MAX6675 tc4(TC_SCK, TC_CS[4], TC_SO);
MAX6675 tc5(TC_SCK, TC_CS[5], TC_SO);

MAX6675* TCs[6] = { &tc0, &tc1, &tc2, &tc3, &tc4, &tc5 };

// -------------------- Forward Declarations --------------------
bool   initDisplay();
void   drawScreen(const float tempsC[6], bool recording, uint32_t elapsedMs);
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

  // UI
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // SD bus (hardware SPI)
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  // Thermocouple CS pins default HIGH
  for (int i = 0; i < 6; ++i) {
    pinMode(TC_CS[i], OUTPUT);
    digitalWrite(TC_CS[i], HIGH);
  }
  // Shared SCK/SO pins
  pinMode(TC_SCK, OUTPUT);
  digitalWrite(TC_SCK, LOW);
  pinMode(TC_SO, INPUT);

  if (!initSD()) {
    Serial.println("SD init failed; realtime works but recording will fail.");
  }

  appStartMs   = millis();
  lastSampleMs = appStartMs;
  lastBatchMs  = appStartMs;
  state        = IDLE;

  // Splash
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Thermolog v2.0"));
  display.println(F("Ready. Press BTN"));
  display.println(F("to start logging."));
  display.display();
}

void loop() {
  // Read all temps via Adafruit library
  float tempsC[6];
  for (int i = 0; i < 6; ++i) {
    if (!readMAX6675CelsiusIdx(i, tempsC[i])) tempsC[i] = NAN;
  }

  // UI
  uint32_t elapsed = (state == RECORDING) ? (millis() - recStartMs) : 0;
  drawScreen(tempsC, state == RECORDING, elapsed);

  // Button
  if (buttonPressedEdge()) setRecording(state != RECORDING);

  const uint32_t now = millis();

  // Sampling & batching
  if (state == RECORDING) {
    if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
      lastSampleMs = now;
      uint32_t seq = (now - recStartMs) / SAMPLE_INTERVAL_MS;
      uint32_t elapsedMs = now - recStartMs;

      if (!appendLineToBatch(tempsC, elapsedMs, seq)) {
        if (flushBatchToFile()) {
          appendLineToBatch(tempsC, elapsedMs, seq);
        } else {
          Serial.println("Batch flush failed; stopping recording.");
          setRecording(false);
        }
      }
    }

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

  delay(40);
}

// ======================================================================
// Display (Adafruit_GFX + Adafruit_SSD1306)
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
  display.println(F("Thermolog v2.0"));
  display.println(F("Booting..."));
  display.display();
  delay(250);
  return true;
}

void drawScreen(const float tempsC[6], bool recording, uint32_t elapsedMs) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Header
  display.setCursor(0, 0);
  display.println(F("MAX6675 x6 (C)"));

  // 3 rows x 2 columns
  for (int i = 0; i < 6; ++i) {
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

  // Footer
  display.setCursor(0, 56);
  if (recording) {
    uint32_t secs = elapsedMs / 1000;
    uint32_t mm = secs / 60;
    uint32_t ss = secs % 60;
    snprintf(lineFmtBuf, sizeof(lineFmtBuf), "REC %02lu:%02lu",
             (unsigned long)mm, (unsigned long)ss);
    display.print(lineFmtBuf);
  } else {
    display.print(F("IDLE"));
  }

  display.display();
}

// ======================================================================
// SD / Session / CSV (append-only, manifest, 60 s batching)
// ======================================================================
bool initSD() {
  // SD on SPI bus
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
  struct tm tmv;
  localtime_r(&t, &tmv);
  strftime(dst, n, "%Y-%m-%d", &tmv);
}

void formatTime(char *dst, size_t n, time_t t) {
  struct tm tmv;
  localtime_r(&t, &tmv);
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
               sessionDir,
               (int)strlen(sessionFile) - 4, sessionFile,
               i);
      if (!SD.exists(smallBuf)) {
        strncpy(sessionPath, smallBuf, sizeof(sessionPath));
        sessionPath[sizeof(sessionPath)-1] = '\0';
        break;
      }
    }
  }

  logFile = SD.open(sessionPath, FILE_WRITE);
  if (!logFile) return false;

  // Manifest header
  time_t now;
  bool timeOk = hasRealTime(&now);

  logFile.println(F("# manifest_begin"));
  logFile.println(F("# firmware: Thermolog v2.0"));
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
  logFile.println(F("# channels: T1_C,T2_C,T3_C,T4_C,T5_C,T6_C"));
  logFile.println(F("# units: C"));
  logFile.println(F("# csv_header: elapsed_s,seq,T1_C,T2_C,T3_C,T4_C,T5_C,T6_C"));
  logFile.println(F("# manifest_end"));

  // CSV header
  logFile.println(CSV_HEADER);
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

// ======================================================================
// MAX6675 helper (Adafruit lib)
// ======================================================================
bool readMAX6675CelsiusIdx(int idx, float &outC) {
  // Library returns NAN on open-circuit / fault
  float t = TCs[idx]->readFahrenheit();

  // Optional sanity clamp (MAX6675 valid 0..~1024 C)
  if (!isnan(t) && (t < -10.0f || t > 1100.0f)) {
    outC = NAN;
    return true;
  }
  outC = t;
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
    if (!SD.begin(SD_CS, SPI)) {
      Serial.println("SD not ready; cannot start recording.");
      digitalWrite(PIN_LED, LOW);
      state = IDLE;
      return;
    }
    if (!openNewSessionFile(false)) {
      Serial.println("Failed to open session file.");
      digitalWrite(PIN_LED, LOW);
      state = IDLE;
      return;
    }

    recStartMs   = millis();
    lastSampleMs = recStartMs;
    lastBatchMs  = recStartMs;
    batchCount   = 0;

    digitalWrite(PIN_LED, HIGH);
    state = RECORDING;
    Serial.print("Recording → ");
    Serial.println(sessionPath);
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
