#include <Arduino.h>
#include <Wire.h>
#include "esp_camera.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PN532.h>

Adafruit_PN532 nfc(-1, -1, &Wire);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// Camera Pins
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


#define CHECK_INTERVAL_MS     10000  // camera check every 10 seconds
#define ABSENCE_TIMEOUT_MS    30000  // 30 seconds absent - session paused
#define DIFF_THRESHOLD        30     // pixel brightness change to count
#define TOLERANCE_THRESHOLD   0.50f  // 50% change = person gone
#define FRAME_WIDTH           320
#define FRAME_HEIGHT          240

// ROI — region of interest (center of frame, where we assume the person is sitting in front)
#define ROI_X   110
#define ROI_Y    80
#define ROI_W   100
#define ROI_H    80


enum State { WAITING_FOR_NFC, SHOWING_WELCOME, MONITORING_PRESENCE };
State currentState = WAITING_FOR_NFC;

bool          wasAbsent    = false;
unsigned long lastFaceSeen = 0;
unsigned long lastCamCheck = 0;

uint8_t* baselineFrame = nullptr;
size_t   frameSize     = 0;


void   initOLED();
void   initNFC();
void   initCamera();
void   showMessage(const char* l1, const char* l2, const char* l3);
void   showCountdown(unsigned long absentFor);
bool   checkPresence();
void   handlePresenceMonitoring();


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Booting...");

  Wire.begin(5, 6);

  initOLED();
  initNFC();
  initCamera();

  showMessage("FocusStudy", "Partner", "Tap NFC");
  Serial.println("System ready");
}


void loop() {
  switch (currentState) {

    case WAITING_FOR_NFC:
{
    uint8_t uid[7];
    uint8_t uidLength = 0;

    if (nfc.readPassiveTargetID(
            PN532_MIFARE_ISO14443A,
            uid,
            &uidLength,
            100))
    {
        Serial.println("NFC tapped");

        for (uint8_t i = 0; i < uidLength; i++) {
            Serial.printf("%02X ", uid[i]);
        }
        Serial.println();

        currentState = SHOWING_WELCOME;
    }
}
break;

Serial.println("Entering SHOWING_WELCOME");
    case SHOWING_WELCOME:
      showMessage("Welcome!", "Capturing", "baseline...");
      delay(2000);

      {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
          frameSize = fb->len;

          // Free previous allocation if session restarted
          if (baselineFrame != nullptr) {
            heap_caps_free(baselineFrame);
            baselineFrame = nullptr;
          }

          // We need to allocate PSRAM
          baselineFrame = (uint8_t*)heap_caps_malloc(frameSize, MALLOC_CAP_SPIRAM);

          if (baselineFrame == nullptr) {
            Serial.println("PSRAM malloc FAILED");
            showMessage("Memory", "FAILED", "restart board");
            esp_camera_fb_return(fb);
            while (true) delay(1000);
          }

          Serial.println("Capturing baseline...");
          memcpy(baselineFrame, fb->buf, frameSize);
          esp_camera_fb_return(fb);
          Serial.printf("Baseline captured — %d bytes in PSRAM\n", frameSize);
          Serial.println("Baseline capture complete");
        } else {
          Serial.println("Camera frame FAILED");
          showMessage("Camera", "Error", "no frame");
        }
      }

      lastFaceSeen = millis();
      lastCamCheck = millis();
      showMessage("Studying", "Active", "");
      currentState = MONITORING_PRESENCE;
      break;

    case MONITORING_PRESENCE:
      handlePresenceMonitoring();
      break;
  }
}


void initOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
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
    Serial.println("PN532 FAILED — check wiring");
    showMessage("NFC", "FAILED", "check wiring");
    while (true) delay(1000);
  }
  Serial.printf("PN532 ready — v%d.%d\n",
    (version >> 16) & 0xFF,
    (version >> 8)  & 0xFF);
  nfc.SAMConfig();
}

void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer  = LEDC_TIMER_0;
  config.pin_d0  = Y2_GPIO_NUM;
  config.pin_d1  = Y3_GPIO_NUM;
  config.pin_d2  = Y4_GPIO_NUM;
  config.pin_d3  = Y5_GPIO_NUM;
  config.pin_d4  = Y6_GPIO_NUM;
  config.pin_d5  = Y7_GPIO_NUM;
  config.pin_d6  = Y8_GPIO_NUM;
  config.pin_d7  = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera FAILED — check expansion board");
    showMessage("Camera", "FAILED", "check board");
    while (true) delay(1000);
  }
  Serial.println("Camera ready");
}

// Display Message function
void showMessage(const char* l1, const char* l2, const char* l3) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.println(l1);
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.println(l2);
  display.setCursor(0, 50);
  display.println(l3);
  display.display();
}

void showCountdown(unsigned long absentFor) {
  unsigned long remaining = 0;
  if (ABSENCE_TIMEOUT_MS > absentFor)
    remaining = (ABSENCE_TIMEOUT_MS - absentFor) / 1000;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Are you there?");
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.printf("%lus left", remaining);
  display.setTextSize(1);
  display.setCursor(0, 52);
  display.println("Come back to resume");
  display.display();
}

// Just to avoid capturing noise, we blur using the 3x3 kernel. 
static uint8_t blurPixel(const uint8_t* buf, int x, int y) {
  long sum = 0;
  int  cnt = 0;
  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      int nx = x + dx, ny = y + dy;
      if (nx >= 0 && nx < FRAME_WIDTH && ny >= 0 && ny < FRAME_HEIGHT) {
        sum += buf[ny * FRAME_WIDTH + nx];
        cnt++;
      }
    }
  }
  return (uint8_t)(sum / cnt);
}

// Presence checking function
bool checkPresence() {
  if (!baselineFrame) return false;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return false;

  long changedPixels = 0;
  long totalPixels   = ROI_W * ROI_H;

  for (int y = ROI_Y; y < ROI_Y + ROI_H; y++) {
    for (int x = ROI_X; x < ROI_X + ROI_W; x++) {
      uint8_t cur  = blurPixel(fb->buf,       x, y);
      uint8_t base = blurPixel(baselineFrame, x, y);
      if (abs((int)cur - (int)base) > DIFF_THRESHOLD)
        changedPixels++;
    }
  }

  esp_camera_fb_return(fb);

  float ratio = (float)changedPixels / totalPixels;
  Serial.printf("Baseline diff: %.1f%% | %s\n",
    ratio * 100,
    ratio < TOLERANCE_THRESHOLD ? "PRESENT" : "ABSENT");

  return ratio < TOLERANCE_THRESHOLD;
}


void handlePresenceMonitoring() {

  // Camera check every 10 seconds
  if (millis() - lastCamCheck >= CHECK_INTERVAL_MS) {
    lastCamCheck = millis();

    bool present = checkPresence();
    if (present) {
      lastFaceSeen = millis();
      if (wasAbsent) {
        wasAbsent = false;
        showMessage("Welcome", "back!", "");
        delay(1500);
      }
      showMessage("Studying", "Active", "");
    }
  }


  unsigned long timeSinceLastSeen = millis() - lastFaceSeen;

  if (timeSinceLastSeen > CHECK_INTERVAL_MS) {
    if (timeSinceLastSeen >= ABSENCE_TIMEOUT_MS) {
      if (!wasAbsent) {
        wasAbsent = true;
        showMessage("Session", "Paused", "Button Functionality Pending To Resume Session");
        Serial.println("Session paused — absent too long");
      }
    } else {
      showCountdown(timeSinceLastSeen);
    }
  }
}
