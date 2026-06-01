// ldr.cpp
#include <Arduino.h>
#define LDR_PIN 1

void ldr_setup() {
  // ldr init buraya
}

void ldr_loop() {
  int raw = analogRead(LDR_PIN);
  float voltage = raw * (3.3 / 4095.0);

  String tier;
  if (raw > 3000) tier = "Good";
  else if (raw > 1500) tier = "Marginal";
  else tier = "Poor - Room too dark";

  Serial.print("Raw: "); Serial.print(raw);
  Serial.print("  Voltage: "); Serial.print(voltage, 2);
  Serial.print("V  Tier: "); Serial.println(tier);
}