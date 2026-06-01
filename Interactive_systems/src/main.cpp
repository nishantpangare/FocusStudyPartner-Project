// main.cpp
#include <Arduino.h>
#include "ldr.h"
#include "buzzer.h"  // buzzer da aynı şekilde düzenlenmişse

void setup() {
  Serial.begin(115200);
  ldr_setup();
  // buzzer_setup();
}

void loop() {
  ldr_loop();
  // buzzer_loop();
  delay(1000);
}