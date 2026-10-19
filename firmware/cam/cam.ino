// =============================================================================================
//  CAM - sketch for the ESP32-CAM (AI-Thinker) mounted at the front of the box
// =============================================================================================
//
//  STATUS: placeholder. Compiles, blinks the on-board LED, prints a heartbeat on Serial.
//  The camera is not started yet; see docs/architecture.md for the planned role of the CAM
//  (stream to the phone for driving, later: lane / marker detection reported to the box).
//
//  BOARD: "AI Thinker ESP32-CAM", partition "Huge APP (3MB No OTA/1MB SPIFFS)".
//  FLASH: USB-TTL adapter on U0R/U0T, IO0 to GND during power-up, then reset. See README.md.
//
//  The CAM is a separate device with its own port and its own upload. It never talks to the
//  rover; everything it finds goes to the box (or straight to the phone page).
// =============================================================================================

#include <Arduino.h>
#include "CamConfig.h"

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(CAM_LED_PIN, OUTPUT);
  digitalWrite(CAM_LED_PIN, LOW);
  Serial.println("[cam] placeholder sketch, camera not started");
}

void loop() {
  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last >= 1000) {
    last = now;
    digitalWrite(CAM_LED_PIN, !digitalRead(CAM_LED_PIN));
    Serial.printf("[cam] alive %lu s, heap %u kB\n", (unsigned long)(now / 1000), ESP.getFreeHeap() / 1024);
  }
  delay(10);
}
