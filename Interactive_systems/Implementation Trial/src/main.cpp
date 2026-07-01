#include <Arduino.h>
#include <Wire.h>
#include "esp_camera.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PN532.h>

#define OLED_ADDRESS  0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_PN532 nfc(-1, -1, &Wire);

// ───────────── CAMERA PINS (XIAO ESP32-S3) ─────────────
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// ───────────── ULTRASONIC PINS ─────────────
#define TRIG_PIN D2
#define ECHO_PIN D3

// ───────────── BUZZER PIN ─────────────
#define BUZZER_PIN D0   // I/O pin of buzzer module

// ───────────── DURATION-SELECT BUTTON PINS ─────────────
// NOTE: I2C (SDA/SCL) = D4/D5, TRIG=D2, ECHO=D3, BUZZER=D0,
// and the camera FPC uses GPIO 10-18/38-40/47-48.
// D1, D8, D9 are free on the XIAO ESP32-S3 — double check
// against your actual board silkscreen before wiring, since
// pin numbering can vary slightly by board revision.
#define BTN_UP     D1
#define BTN_DOWN   D8
#define BTN_SELECT D9

// ───────────── TUNING PARAMETERS ─────────────
#define CHECK_INTERVAL_MS     5000  // Run the Fusion Check every 5 seconds
#define ABSENCE_TIMEOUT_MS    30000 // Pause if missing for 30 seconds

#define CAM_DIFF_THRESHOLD    30
#define CAM_TOLERANCE_PCT     0.30f

#define ROI_X   110
#define ROI_Y   80
#define ROI_W   100
#define ROI_H   80

#define BTN_DEBOUNCE_MS       200

// ───────────── STATE & GLOBALS ─────────────
enum State { WAITING_FOR_NFC, SELECTING_DURATION, SHOWING_WELCOME, MONITORING_PRESENCE, SESSION_COMPLETE };
State currentState = WAITING_FOR_NFC;

bool          wasAbsent      = false;  // true once 30s timeout fired (session paused)
bool          isCountingDown = false;  // true while inside the 0-30s grace window
unsigned long lastPresence   = 0;
unsigned long lastCheck      = 0;

int      baselineDistanceCM = 0;
uint8_t* baselineFrame      = nullptr;
size_t   frameSize          = 0;

// ── Pomodoro duration menu state ──
const int durationOptions[] = { 25, 45, 60 };  // minutes
const int numDurationOptions = 3;
int  selectedDurationIndex = 0;
unsigned long lastBtnTime = 0;

unsigned long sessionDurationMs = 0;   // total target study time for this session
unsigned long sessionStudyMs    = 0;   // accumulated *active* study time
unsigned long lastTickMs        = 0;   // for accumulating sessionStudyMs

// ───────────── FUNCTION DECLARATIONS ─────────────
void showMessage(const char* l1, const char* l2, const char* l3);
void showCountdown(unsigned long absentFor);
void captureBaselines();
bool isCameraPresent();
bool isSonarPresent();
void handlePresenceMonitoring();
void buzzerBeepOnce();
void buzzerBeepPattern(int times);
void drawDurationMenu();
void handleDurationMenuInput();
void showStudyingScreen();

// ───────────── SETUP ─────────────
void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(5, 6);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("FATAL: OLED not found!");
    while (true);
  }
  display.clearDisplay(); display.display();

  showMessage("Booting", "Checking", "Hardware...");

  nfc.begin();
  if (!nfc.getFirmwareVersion()) {
    showMessage("FATAL ERROR", "NFC Chip", "Dead");
    while (true);
  }
  nfc.SAMConfig();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QVGA;
  config.fb_count     = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    showMessage("FATAL ERROR", "Camera", "Init Failed");
    while (true);
  }

  showMessage("FocusStudy", "Fusion Ready", "Tap NFC");
}

// ───────────── MAIN LOOP ─────────────
void loop() {
  switch (currentState) {
    case WAITING_FOR_NFC: {
      uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
      uint8_t uidLength;

      if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100)) {
        selectedDurationIndex = 0;     // reset menu cursor each new tap
        drawDurationMenu();
        currentState = SELECTING_DURATION;
      }
      break;
    }

    case SELECTING_DURATION:
      handleDurationMenuInput();
      break;

    case SHOWING_WELCOME:
      showMessage("Welcome!", "Locking", "Sensors...");
      delay(2000);

      captureBaselines();

      lastPresence = millis();
      lastCheck    = millis();
      lastTickMs   = millis();
      wasAbsent      = false;
      isCountingDown = false;
      sessionStudyMs = 0;

      showStudyingScreen();
      currentState = MONITORING_PRESENCE;
      break;

    case MONITORING_PRESENCE:
      handlePresenceMonitoring();
      break;

    case SESSION_COMPLETE: {
      static unsigned long completeShownAt = 0;
      if (completeShownAt == 0) {
        showMessage("Session", "Complete!", "Nice work");
        buzzerBeepPattern(3);
        completeShownAt = millis();
      }
      // Hold the message for a few seconds, then return to NFC wait
      if (millis() - completeShownAt > 4000) {
        completeShownAt = 0;
        showMessage("FocusStudy", "Fusion Ready", "Tap NFC");
        currentState = WAITING_FOR_NFC;
      }
      break;
    }
  }
}

// ───────────── DURATION SELECTION MENU ─────────────
void drawDurationMenu() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Select Duration:");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  for (int i = 0; i < numDurationOptions; i++) {
    int y = 18 + (i * 14);
    display.setCursor(10, y);
    display.print(i == selectedDurationIndex ? "> " : "  ");
    display.print(durationOptions[i]);
    display.println(" min");
  }
  display.display();
}

void handleDurationMenuInput() {
  unsigned long now = millis();
  if (now - lastBtnTime < BTN_DEBOUNCE_MS) return;

  if (digitalRead(BTN_UP) == LOW) {
    selectedDurationIndex = (selectedDurationIndex - 1 + numDurationOptions) % numDurationOptions;
    drawDurationMenu();
    lastBtnTime = now;
  } else if (digitalRead(BTN_DOWN) == LOW) {
    selectedDurationIndex = (selectedDurationIndex + 1) % numDurationOptions;
    drawDurationMenu();
    lastBtnTime = now;
  } else if (digitalRead(BTN_SELECT) == LOW) {
    sessionDurationMs = (unsigned long)durationOptions[selectedDurationIndex] * 60UL * 1000UL;

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Session set:");
    display.setTextSize(2);
    display.setCursor(10, 25);
    display.print(durationOptions[selectedDurationIndex]);
    display.println(" min");
    display.display();
    delay(1000);

    lastBtnTime = now;
    currentState = SHOWING_WELCOME;
  }
}

// ───────────── BUZZER UTIL ─────────────
void buzzerBeepOnce() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
}

void buzzerBeepPattern(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);
  }
}


// ── Cat animation frame bitmaps (8x8 pixel art) ──────────────
// Frame 1 — legs forward
const uint8_t CAT_FRAME1[] PROGMEM = {
  0b00111100,
  0b01111110,
  0b01011010,
  0b01111110,
  0b00111100,
  0b00111100,
  0b01000010,
  0b01000010
};

// Frame 2 — legs back
const uint8_t CAT_FRAME2[] PROGMEM = {
  0b00111100,
  0b01111110,
  0b01011010,
  0b01111110,
  0b00111100,
  0b00111100,
  0b00100100,
  0b00100100
};

// ── Cat walk state ────────────────────────────────────────────
int catX          = 0;
int catFrame      = 0;
int catDirection  = 1;           // 1 = right, -1 = left
unsigned long lastCatMove = 0;

void showStudyingScreen() {
  unsigned long now = millis();

  // Calculate remaining time
  unsigned long elapsed  = sessionStudyMs;
  unsigned long totalMs  = sessionDurationMs;
  unsigned long remaining = (totalMs > elapsed) ? (totalMs - elapsed) : 0;

  unsigned long remainMins = remaining / 60000;
  unsigned long remainSecs = (remaining % 60000) / 1000;

  display.clearDisplay();

  // ── "STUDYING" label top left ──
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("STUDYING");

  // ── Big timer in center ──
  display.setTextSize(3);
  display.setCursor(14, 18);
  if (remainMins < 10) display.print("0");
  display.print(remainMins);
  display.print(":");
  if (remainSecs < 10) display.print("0");
  display.print(remainSecs);

  // ── Animate cat ──
  // Move cat every 150ms
  if (now - lastCatMove > 150) {
    lastCatMove = now;
    catX += catDirection * 2;
    catFrame = !catFrame;

    // Bounce at edges
    if (catX > 100) catDirection = -1;
    if (catX < 0)   catDirection =  1;
  }

  // Draw cat body (8x8 bitmap, scaled x1)
  if (catFrame == 0) {
    display.drawBitmap(catX, 55, CAT_FRAME1, 8, 8, SSD1306_WHITE);
  } else {
    display.drawBitmap(catX, 55, CAT_FRAME2, 8, 8, SSD1306_WHITE);
  }

  display.display();
}

void showMessage(const char* l1, const char* l2, const char* l3) {
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2); display.setCursor(0, 5); display.println(l1);
  display.setTextSize(1); display.setCursor(0, 35); display.println(l2);
  display.setCursor(0, 50); display.println(l3);
  display.display();
}

void showCountdown(unsigned long absentFor) {
  unsigned long remaining = (ABSENCE_TIMEOUT_MS > absentFor) ? (ABSENCE_TIMEOUT_MS - absentFor) / 1000 : 0;
  display.clearDisplay();
  display.setTextSize(1); display.setCursor(0, 0); display.println("Are you there?");
  display.setTextSize(2); display.setCursor(0, 20); display.printf("%lus", remaining);
  display.display();
}

// ───────────── SENSOR CAPTURE & LOGIC ─────────────
void captureBaselines() {
  long total = 0; int validReadings = 0;
  for (int i = 0; i < 5; i++) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10); digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    int d = (duration == 0) ? 999 : duration * 0.034 / 2;
    if (d < 400) { total += d; validReadings++; }
    delay(50);
  }
  baselineDistanceCM = (validReadings > 0) ? (total / validReadings) : 50;

  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) {
    frameSize = fb->len;
    if (baselineFrame == nullptr) baselineFrame = (uint8_t*)malloc(frameSize);
    memcpy(baselineFrame, fb->buf, frameSize);
    esp_camera_fb_return(fb);
  }

  Serial.println("\n=== BASELINES LOCKED ===");
  Serial.printf("SONAR Baseline  : %d cm\n", baselineDistanceCM);
  Serial.printf("CAMERA Baseline : Frame Captured (%d bytes)\n", frameSize);
  Serial.println("========================\n");
}

int calculateDynamicTolerance(int baseline) {
  int tolerance = baseline * 0.25;
  return (tolerance < 15) ? 15 : tolerance;
}

bool isSonarPresent() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10); digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int currentDistance = (duration == 0) ? 999 : duration * 0.034 / 2;

  if (currentDistance >= 999) return false;

  int dynamicTolerance = calculateDynamicTolerance(baselineDistanceCM);
  int upperBound = baselineDistanceCM + dynamicTolerance;
  int lowerBound = 10;

  bool isSafe = (currentDistance >= lowerBound && currentDistance <= upperBound);

  Serial.printf("SONAR  -> Baseline: %dcm | Current: %dcm | Zone: %dcm to %dcm | Status: %s\n",
                baselineDistanceCM, currentDistance, lowerBound, upperBound, isSafe ? "SAFE" : "TRIPPED");

  return isSafe;
}

static uint8_t blurPixel(const uint8_t* buf, int x, int y) {
  long sum = 0; int cnt = 0;
  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      int nx = x + dx, ny = y + dy;
      if (nx >= 0 && nx < 320 && ny >= 0 && ny < 240) {
        sum += buf[ny * 320 + nx]; cnt++;
      }
    }
  }
  return sum / cnt;
}

bool isCameraPresent() {
  if (!baselineFrame) return false;
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return false;

  long changedPixels = 0;
  for (int y = ROI_Y; y < ROI_Y + ROI_H; y++) {
    for (int x = ROI_X; x < ROI_X + ROI_W; x++) {
      uint8_t cur  = blurPixel(fb->buf, x, y);
      uint8_t base = blurPixel(baselineFrame, x, y);
      if (abs((int)cur - (int)base) > CAM_DIFF_THRESHOLD) changedPixels++;
    }
  }
  esp_camera_fb_return(fb);

  float ratio = (float)changedPixels / (ROI_W * ROI_H);
  bool isSafe = (ratio < CAM_TOLERANCE_PCT);

  Serial.printf("CAMERA -> Changed Pixels: %.1f%% | Allowed Change: %.1f%% | Status: %s\n",
                (ratio * 100), (CAM_TOLERANCE_PCT * 100), isSafe ? "SAFE" : "TRIPPED");

  return isSafe;
}

// ───────────── SENSOR FUSION MANAGER ─────────────
void handlePresenceMonitoring() {

  // ── Accumulate active study time (paused time doesn't count) ──
  unsigned long now = millis();
  if (!wasAbsent) {
    sessionStudyMs += (now - lastTickMs);
  }
  lastTickMs = now;

  // ── Session duration reached → wrap up ──
  if (sessionDurationMs > 0 && sessionStudyMs >= sessionDurationMs) {
    currentState = SESSION_COMPLETE;
    return;
  }

  // ── If session is already PAUSED, wait for NFC tap to resume ──
  if (wasAbsent) {
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
    uint8_t uidLength;
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100)) {
      wasAbsent      = false;
      isCountingDown = false;
      showMessage("Welcome", "back!", "Resuming...");
      delay(1500);
      captureBaselines();
      lastPresence = millis(); lastCheck = millis(); lastTickMs = millis();
      showStudyingScreen();
    }
    return; // don't run fusion checks while paused
  }

  // ── Run the fusion check every CHECK_INTERVAL_MS ──
  if (millis() - lastCheck >= CHECK_INTERVAL_MS) {
    lastCheck = millis();

    Serial.println("\n--- FUSION CHECK ---");
    bool sonarSafe  = isSonarPresent();
    bool cameraSafe = isCameraPresent();
    bool present    = sonarSafe || cameraSafe;

    Serial.printf("FUSION RESULT -> Depth: [%s] | Movement: [%s] | Present: %s\n",
                  sonarSafe ? "PRESENT" : "ABSENT",
                  cameraSafe ? "PRESENT" : "ABSENT",
                  present ? "YES" : "NO");

    if (present) {
      lastPresence = millis();

      if (isCountingDown) {
        isCountingDown = false;
        Serial.println("User returned within grace period");
      }

      showStudyingScreen();
    }
  }

  // ── Track how long the user has been absent ──
  unsigned long timeSinceLastSeen = millis() - lastPresence;

if (timeSinceLastSeen > CHECK_INTERVAL_MS) {
  if (timeSinceLastSeen >= ABSENCE_TIMEOUT_MS) {
    if (!wasAbsent) {
      wasAbsent      = true;
      isCountingDown = false;
      showMessage("Session", "Paused", "Tap NFC to resume");
      Serial.println("SYSTEM: Session Paused. Waiting for NFC...");
    }
  } else {
    if (!isCountingDown) {
      isCountingDown = true;
      buzzerBeepOnce();
      Serial.println("User missing — single beep, countdown started");
    }
    showCountdown(timeSinceLastSeen);
  }
} else {
  // Person present — update screen every single loop tick
  showStudyingScreen();
}
}
