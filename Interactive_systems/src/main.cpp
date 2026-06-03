// main.cpp
#include <Arduino.h>
#include "ldr.h"
#include "buzzer.h"


void setup() {
  Serial.begin(115200);
  ldr_setup();
}

void loop() {
  ldr_loop();
  delay(1000);
}
