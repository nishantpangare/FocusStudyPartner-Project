#include <Arduino.h>
#include "buzzer.h"

#define BUZZER_PIN 1

int melody[] = {659, 659, 0, 659, 0, 523, 659, 0};
int durations[] = {125, 125, 125, 125, 125, 125, 250, 125};
int noteCount = 8;

unsigned long startTime;
unsigned long playDuration = 3000;

bool finished = false;

void buzzer_setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  startTime = millis();
}

void buzzer_loop() {
  if (finished) {
    noTone(BUZZER_PIN);
    return;
  }

  for (int i = 0; i < noteCount; i++) {
    if (millis() - startTime >= playDuration) {
      noTone(BUZZER_PIN);
      finished = true;
      return;
    }

    if (melody[i] == 0) {
      noTone(BUZZER_PIN);
    } else {
      tone(BUZZER_PIN, melody[i]);
    }

    delay(durations[i]);
    noTone(BUZZER_PIN);
    delay(30);
  }
}