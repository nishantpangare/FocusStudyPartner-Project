#include <Arduino.h>
#include <Wire.h>
#include "esp_camera.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PN532.h>

// ─────────────────────────────────────────────────────────────
// OLED CONFIGURATION
// 128x64 pixel monochrome display over I2C
// Address 0x3C is standard for this display
// OLED_RESET = -1 means we don't use a reset pin
// ─────────────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDRESS  0x3C

// ─────────────────────────────────────────────────────────────
// NFC CONFIGURATION
// PN532 in I2C mode — no SS or RST pins needed
// Shares the same SDA/SCL lines as the OLED (I2C bus)
// ─────────────────────────────────────────────────────────────
Adafruit_PN532 nfc(-1, -1, &Wire);

// ─────────────────────────────────────────────────────────────
// CAMERA PIN DEFINITIONS
// These are fixed by the Seeed XIAO ESP32-S3 Sense expansion
// board hardware — same for both OV2640 and OV3660
// ─────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────
// TUNING PARAMETERS
// Adjust these to change sensitivity and timing behaviour
// ─────────────────────────────────────────────────────────────
#define DIFF_THRESHOLD      30      // how much a pixel must change to count
#define PRESENCE_THRESHOLD  1500    // how many changed pixels = person present
#define CHECK_INTERVAL_MS   1000    // check camera once per second
#define ABSENCE_TIMEOUT_MS  30000   // 30 seconds absent → warning shown

// ─────────────────────────────────────────────────────────────
// STATE MACHINE
// The system is always in exactly one of these 3 states.
// States transition in one direction: NFC → Welcome → Monitor
// ─────────────────────────────────────────────────────────────
enum State {
  WAITING_FOR_NFC,      // idle, waiting for card tap
  SHOWING_WELCOME,      // briefly shows welcome screen
  MONITORING_PRESENCE   // actively watching camera
};
State currentState = WAITING_FOR_NFC;

// ─────────────────────────────────────────────────────────────
// PRESENCE TRACKING VARIABLES
// ─────────────────────────────────────────────────────────────
bool          facePresent   = false;  // is someone currently visible?
bool          wasAbsent     = false;  // were they absent before?
unsigned long lastFaceSeen  = 0;      // timestamp of last detection
unsigned long lastCheck     = 0;      // timestamp of last camera check

// ─────────────────────────────────────────────────────────────
// FRAME BUFFER FOR MOTION COMPARISON
// previousFrame stores the last captured image
// We compare each new frame against it pixel by pixel
// ─────────────────────────────────────────────────────────────
uint8_t* previousFrame = nullptr;
size_t   frameSize     = 0;

// ─────────────────────────────────────────────────────────────
// OBJECTS
// ─────────────────────────────────────────────────────────────
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// put function declarations here:
void   initOLED();
void   initNFC();
void   initCamera();
void   showMessage(const char* l1, const char* l2, const char* l3);
void   showCountdown(unsigned long absentFor);
bool   cardTapped(uint8_t* uid, uint8_t* uidLength);
String uidToString(uint8_t* uid, uint8_t uidLength);
bool   checkPresence();
void   handlePresenceMonitoring();

// ─────────────────────────────────────────────────────────────
// SETUP — runs once on boot
// Initialises all hardware in order:
// 1. Serial (for debug output)
// 2. I2C bus (shared by OLED and NFC)
// 3. OLED display
// 4. NFC reader
// 5. Camera
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  // SDA=GPIO5 (D4 on XIAO), SCL=GPIO6 (D5 on XIAO)
  Wire.begin(5, 6);

  initOLED();
  initNFC();
  initCamera();

  showMessage("FocusStudy", "Partner", "Tap NFC to start");
  Serial.println("System ready — waiting for NFC tap...");
}

// ─────────────────────────────────────────────────────────────
// LOOP — runs repeatedly
// Acts as a dispatcher — checks current state and
// calls the right logic for that state
// ─────────────────────────────────────────────────────────────
void loop() {
  uint8_t uid[7];
  uint8_t uidLength;

  switch (currentState) {

    // ── State 1: Do nothing until NFC card is tapped ─────────
    case WAITING_FOR_NFC:
      if (cardTapped(uid, &uidLength)) {
        String cardID = uidToString(uid, uidLength);
        Serial.println("Card tapped — UID: " + cardID);
        currentState = SHOWING_WELCOME;
      }
      break;

    // ── State 2: Show welcome screen, capture baseline frame ──
    case SHOWING_WELCOME:
      showMessage("Welcome!", "Session", "starting...");
      delay(2000);

      // Capture the first frame as reference point.
      // Without this, the first comparison has nothing to
      // compare against and would always trigger motion.
      {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
          frameSize     = fb->len;
          previousFrame = (uint8_t*)malloc(frameSize);
          memcpy(previousFrame, fb->buf, frameSize);
          esp_camera_fb_return(fb);
          Serial.println("Baseline frame captured");
        }
      }

      lastFaceSeen = millis(); // assume present at session start
      showMessage("Studying", "Session", "Active");
      currentState = MONITORING_PRESENCE;
      break;

    // ── State 3: Monitor presence every second ────────────────
    case MONITORING_PRESENCE:
      handlePresenceMonitoring();
      break;
  }
}

// ─────────────────────────────────────────────────────────────
// initOLED()
// Starts the OLED display over I2C.
// Halts the program if display not found — no point
// continuing if the screen isn't working.
// ─────────────────────────────────────────────────────────────
void initOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED FAILED — check wiring");
    while (true) delay(1000);
  }
  display.clearDisplay();
  display.display();
  Serial.println("OLED ready");
}

// ─────────────────────────────────────────────────────────────
// initNFC()
// Starts the PN532 over I2C and verifies it responds.
// getFirmwareVersion() is used as a "ping" — if 0 is returned
// the chip isn't responding (wrong wiring or wrong address).
// SAMConfig() puts the chip into normal card-reading mode.
// ─────────────────────────────────────────────────────────────
void initNFC() {
  nfc.begin();
  uint32_t version = nfc.getFirmwareVersion();
  if (!version) {
    Serial.println("PN532 not found — check wiring");
    showMessage("NFC", "FAILED", "check wiring");
    while (true) delay(1000);
  }
  Serial.printf("PN532 found — firmware v%d.%d\n",
    (version >> 16) & 0xFF,
    (version >> 8)  & 0xFF);
  nfc.SAMConfig();
  Serial.println("NFC ready");
}

// ─────────────────────────────────────────────────────────────
// initCamera()
// Configures and starts the OV3660 camera.
// PIXFORMAT_GRAYSCALE — no colour data needed, saves RAM
// FRAMESIZE_QVGA — 320x240, enough for motion detection
// fb_count=1 — single frame buffer, sufficient for our use
// Halts if camera fails — can't do presence detection without it
// ─────────────────────────────────────────────────────────────
void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera FAILED — check expansion board");
    showMessage("Camera", "FAILED", "check board");
    while (true) delay(1000);
  }
  Serial.println("Camera ready");
}

// ─────────────────────────────────────────────────────────────
// showMessage(l1, l2, l3)
// Clears the display and shows 3 lines of text:
// l1 = large text (size 2) at top
// l2 = small text (size 1) in middle
// l3 = small text (size 1) at bottom
// Always call display.display() at the end to push to screen
// ─────────────────────────────────────────────────────────────
void showMessage(const char* l1, const char* l2, const char* l3) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 4);
  display.println(l1);
  display.setTextSize(1);
  display.setCursor(0, 36);
  display.println(l2);
  display.setCursor(0, 50);
  display.println(l3);
  display.display();
}

// ─────────────────────────────────────────────────────────────
// showCountdown(absentFor)
// Special display for the grace period.
// Shows how many seconds remain before the warning triggers.
// Uses printf-style formatting directly onto the display buffer.
// ─────────────────────────────────────────────────────────────
void showCountdown(unsigned long absentFor) {
  unsigned long remaining = (ABSENCE_TIMEOUT_MS - absentFor) / 1000;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 4);
  display.println("Are you there?");
  display.setTextSize(2);
  display.setCursor(0, 22);
  display.printf("%lus left", remaining);
  display.setTextSize(1);
  display.setCursor(0, 52);
  display.println("Come back to resume");
  display.display();
}

// ─────────────────────────────────────────────────────────────
// cardTapped(uid, uidLength)
// Non-blocking NFC read with 100ms timeout.
// Returns true if a card was found and its UID read.
// uid[] is filled with the card's unique ID bytes.
// uidLength tells you how many bytes the UID is (4 or 7).
// 100ms timeout means loop() stays responsive while waiting.
// ─────────────────────────────────────────────────────────────
bool cardTapped(uint8_t* uid, uint8_t* uidLength) {
  return nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, uidLength, 100);
}

// ─────────────────────────────────────────────────────────────
// uidToString(uid, uidLength)
// Converts the raw UID bytes into a readable hex string.
// e.g. {0xA3, 0xF2, 0x91} → "A3:F2:91"
// Useful for Serial debug output and future user identification.
// ─────────────────────────────────────────────────────────────
String uidToString(uint8_t* uid, uint8_t uidLength) {
  String result = "";
  for (uint8_t i = 0; i < uidLength; i++) {
    if (uid[i] < 0x10) result += "0";
    result += String(uid[i], HEX);
    if (i < uidLength - 1) result += ":";
  }
  result.toUpperCase();
  return result;
}

// ─────────────────────────────────────────────────────────────
// checkPresence()
// Captures a new camera frame and compares it pixel-by-pixel
// against the previous frame stored in previousFrame[].
// If enough pixels changed (> PRESENCE_THRESHOLD) we say
// someone is present — their body/head is blocking/moving
// in the frame, causing pixel differences.
// After comparing, saves current frame as new reference.
// Returns: true = person present, false = no one detected
// ─────────────────────────────────────────────────────────────
bool checkPresence() {
  if (!previousFrame) return false;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return false;

  int changedPixels = 0;
  for (size_t i = 0; i < frameSize; i++) {
    if (abs((int)fb->buf[i] - (int)previousFrame[i]) > DIFF_THRESHOLD) {
      changedPixels++;
    }
  }

  // Store current frame as reference for next comparison
  memcpy(previousFrame, fb->buf, frameSize);
  esp_camera_fb_return(fb);

  Serial.printf("Changed pixels: %d  (threshold: %d)\n",
    changedPixels, PRESENCE_THRESHOLD);

  return changedPixels > PRESENCE_THRESHOLD;
}

// ─────────────────────────────────────────────────────────────
// handlePresenceMonitoring()
// Called every loop() tick during MONITORING_PRESENCE state.
// Uses millis() timing so it only actually checks once per
// CHECK_INTERVAL_MS (1 second) — non-blocking, no delay().
//
// Logic flow:
//   Person detected  → reset absence timer, show "Active"
//   Person absent    → check how long they've been gone
//     < 30 seconds   → show countdown on OLED
//     ≥ 30 seconds   → show warning "Come back!"
//   Person returns   → show "Welcome back!" briefly
// ─────────────────────────────────────────────────────────────
void handlePresenceMonitoring() {
  // Non-blocking interval check — only run every 1 second
  if (millis() - lastCheck < CHECK_INTERVAL_MS) return;
  lastCheck = millis();

  facePresent = checkPresence();

  if (facePresent) {
    // ── Person is present ────────────────────────────────────
    lastFaceSeen = millis();

    if (wasAbsent) {
      // They just came back after being absent
      wasAbsent = false;
      Serial.println("Person returned — resuming session");
      showMessage("Welcome", "back!", "Session resumed");
      delay(1500);
    }

    showMessage("Studying", "Session", "Active");
    Serial.println("Person present — session active");

  } else {
    // ── Person is absent ─────────────────────────────────────
    unsigned long absentFor = millis() - lastFaceSeen;

    if (absentFor >= ABSENCE_TIMEOUT_MS) {
      // Over 30 seconds — show persistent warning
      wasAbsent = true;
      Serial.printf("ABSENT %lu sec — warning shown\n", absentFor / 1000);
      showMessage("Come", "back!", "Session paused");

    } else {
      // Grace period — show countdown
      Serial.printf("Not detected — %lu sec until warning\n",
        (ABSENCE_TIMEOUT_MS - absentFor) / 1000);
      showCountdown(absentFor);
    }
  }
}