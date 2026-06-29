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

// ───────────── TUNING PARAMETERS ─────────────
#define CHECK_INTERVAL_MS     5000  // Run the Fusion Check every 5 seconds
#define ABSENCE_TIMEOUT_MS    20000 // Pause if missing for 20 seconds

#define CAM_DIFF_THRESHOLD    30    // Brightness shift to count a pixel as "changed"
#define CAM_TOLERANCE_PCT     0.30f // Allowed pixel change (30%)

#define ROI_X   110
#define ROI_Y   80
#define ROI_W   100
#define ROI_H   80

// ───────────── STATE & GLOBALS ─────────────
enum State { WAITING_FOR_NFC, SHOWING_WELCOME, MONITORING_PRESENCE };
State currentState = WAITING_FOR_NFC;

bool          wasAbsent     = false;
unsigned long lastPresence  = 0;
unsigned long lastCheck     = 0;

// Baselines
int      baselineDistanceCM = 0;
uint8_t* baselineFrame      = nullptr;
size_t   frameSize          = 0;

// ───────────── FUNCTION DECLARATIONS ─────────────
void showMessage(const char* l1, const char* l2, const char* l3);
void showCountdown(unsigned long absentFor);
void captureBaselines();
bool isCameraPresent();
bool isSonarPresent();
void handlePresenceMonitoring();

// ───────────── SETUP ─────────────
void setup() {
  Serial.begin(115200);
  delay(2000); 

  Wire.begin(5, 6);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("FATAL: OLED not found!");
    while(true);
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
    while(true);
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
        currentState = SHOWING_WELCOME;
      }
      break;
    }

    case SHOWING_WELCOME:
      showMessage("Welcome!", "Locking", "Sensors...");
      delay(2000); 

      captureBaselines(); 

      lastPresence = millis();
      lastCheck    = millis();
      
      showMessage("Studying", "Active", "");
      currentState = MONITORING_PRESENCE;
      break;

    case MONITORING_PRESENCE:
      handlePresenceMonitoring();
      break;
  }
}

// ───────────── DISPLAY UTILS ─────────────
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
  // 1. Capture Sonar
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
  
  // 2. Capture Camera
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) {
    frameSize = fb->len;
    if (baselineFrame == nullptr) baselineFrame = (uint8_t*)malloc(frameSize);
    memcpy(baselineFrame, fb->buf, frameSize);
    esp_camera_fb_return(fb);
  }

  // Serial.printf("SYSTEM: Fusion Locked! Depth: %dcm\n", baselineDistanceCM);

  // NEW TELEMETRY READOUT
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
  
  // NEW ASYMMETRIC LOGIC
  int upperBound = baselineDistanceCM + dynamicTolerance;
  int lowerBound = 10; // 10cm physical buffer (sonar struggles if objects touch the mesh)
  
  // You are safe as long as you are between the device and the upper threshold
  bool isSafe = (currentDistance >= lowerBound && currentDistance <= upperBound);

  // NEW TELEMETRY READOUT
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

  // NEW TELEMETRY READOUT
  Serial.printf("CAMERA -> Changed Pixels: %.1f%% | Allowed Change: %.1f%% | Status: %s\n", 
                (ratio * 100), (CAM_TOLERANCE_PCT * 100), isSafe ? "SAFE" : "TRIPPED");

  return isSafe;
}

// ───────────── SENSOR FUSION MANAGER ─────────────
void handlePresenceMonitoring() {
  
  if (wasAbsent) {
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
    uint8_t uidLength;
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100)) {
      wasAbsent = false;
      showMessage("Welcome", "back!", "Resuming...");
      delay(1500); 
      captureBaselines(); 
      lastPresence = millis(); lastCheck = millis(); 
      showMessage("Studying", "Active", "");
    }
    return;
  }

  // FUSION CHECK LOGIC
  if (millis() - lastCheck >= CHECK_INTERVAL_MS) {
    lastCheck = millis();
    
    Serial.println("\n--- 5-SECOND FUSION CHECK ---");
    bool sonarSafe = isSonarPresent();
    bool cameraSafe = isCameraPresent();

    Serial.printf("FUSION RESULT -> Depth: [%s] | Movement: [%s]\n", 
                  sonarSafe ? "PRESENT" : "ABSENT", 
                  cameraSafe ? "PRESENT" : "ABSENT");

    if (sonarSafe || cameraSafe) {
      lastPresence = millis(); 
      showMessage("Studying", "Active", "");
    }
  }

  unsigned long timeSinceLastSeen = millis() - lastPresence;
  if (timeSinceLastSeen > CHECK_INTERVAL_MS) {
    if (timeSinceLastSeen >= ABSENCE_TIMEOUT_MS) {

      showMessage("Session", "Paused", "Tap NFC to resume"); 
      Serial.println("SYSTEM: Session Paused. Waiting for NFC...");
    } else {
      showCountdown(timeSinceLastSeen);
    }
  }


  // --------------------------Implement logic to resume session with NFC TAP------------------------------------
}