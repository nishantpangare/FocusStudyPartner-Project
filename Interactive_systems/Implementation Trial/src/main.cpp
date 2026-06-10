#include <Arduino.h>
#include <Wire.h>
#include "esp_camera.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PN532.h>

#define OLED_ADDRESS  0x3C

Adafruit_PN532 nfc(-1, -1, &Wire);

// CAMERA PIN DEFINITIONS
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

// TUNING PARAMETERS

#define DIFF_THRESHOLD      30      // how much a pixel must change to count
#define PRESENCE_THRESHOLD  3000   // how many changed pixels = person present
#define CHECK_INTERVAL_MS   1000    // check camera once per second
#define ABSENCE_TIMEOUT_MS  30000   // 30 seconds absent, show warning


enum State {
  WAITING_FOR_NFC,      
  SHOWING_WELCOME, 
  MONITORING_PRESENCE   
};
State currentState = WAITING_FOR_NFC;


// PRESENCE TRACKING VARIABLES
bool          facePresent   = false;  
bool          wasAbsent     = false;  
unsigned long lastFaceSeen  = 0;     
unsigned long lastCheck     = 0;     

uint8_t* previousFrame = nullptr;
size_t   frameSize     = 0;


Adafruit_SSD1306 display(128, 64, &Wire, -1);

void   initOLED();
void   initNFC();
void   initCamera();
void   showMessage(const char* l1, const char* l2, const char* l3);
void   showCountdown(unsigned long absentFor);
bool   cardTapped(uint8_t* uid, uint8_t* uidLength);
String uidToString(uint8_t* uid, uint8_t uidLength);
bool   checkPresence();
void   handlePresenceMonitoring();


// Initialises all hardware in order:
// 1. Serial (for debug output)
// 2. I2C bus (shared by OLED and NFC)
// 3. OLED display
// 4. NFC reader
// 5. Camera
void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(5, 6);

  initOLED();
  initNFC();
  initCamera();

  showMessage("FocusStudy", "Partner", "Tap NFC to start");
  Serial.println("System ready — waiting for NFC tap...");
}

void loop() {
  uint8_t uid[7];
  uint8_t uidLength;

  switch (currentState) {

    //State 1: Do nothing until NFC card is tapped
    case WAITING_FOR_NFC:
      if (cardTapped(uid, &uidLength)) {
        String cardID = uidToString(uid, uidLength);
        Serial.println("Card tapped — UID: " + cardID);
        currentState = SHOWING_WELCOME;
      }
      break;

    // State 2: Show welcome screen, capture baseline frame 
    case SHOWING_WELCOME:
      showMessage("Welcome!", "Session", "starting...");
      delay(2000);

      // Capture the first frame as reference point.
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

      lastFaceSeen = millis();
      showMessage("Studying", "Session", "Active");
      currentState = MONITORING_PRESENCE;
      break;

    // ── State 3: Monitor presence every second ────────────────
    case MONITORING_PRESENCE:
      handlePresenceMonitoring();
      break;
  }
}


void initOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED FAILED — check wiring");
    while (true) delay(1000);
  } 
  display.clearDisplay();
  display.display();
  Serial.println("OLED ready");
}


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


bool cardTapped(uint8_t* uid, uint8_t* uidLength) {
  return nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, uidLength, 100);
}


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

  memcpy(previousFrame, fb->buf, frameSize);
  esp_camera_fb_return(fb);

  bool present = (changedPixels > 50 && changedPixels < PRESENCE_THRESHOLD);

  Serial.printf("Changed pixels: %d | Present: %s\n",changedPixels,present ? "YES" : "NO");

  return present;
}

void handlePresenceMonitoring() {
  if (millis() - lastCheck < CHECK_INTERVAL_MS) return;
  lastCheck = millis();

  facePresent = checkPresence();

  if (facePresent) {
    lastFaceSeen = millis();

    if (wasAbsent) {
      wasAbsent = false;
      Serial.println("Person returned — resuming session");
      showMessage("Welcome", "back!", "Session resumed");
      delay(1500);
    }

    showMessage("Studying", "Session", "Active");
    Serial.println("Person present — session active");

  } else {
    unsigned long absentFor = millis() - lastFaceSeen;

    if (absentFor >= ABSENCE_TIMEOUT_MS) {
      wasAbsent = true;
      Serial.printf("ABSENT %lu sec — warning shown\n", absentFor / 1000);
      showMessage("Come", "back!", "Session paused");

    } else {
      Serial.printf("Not detected — %lu sec until warning\n",
        (ABSENCE_TIMEOUT_MS - absentFor) / 1000);
      showCountdown(absentFor);
    }
  }
}