#pragma once
// CamConfig - settings and pin map of the AI-Thinker ESP32-CAM.
//
// The camera pins are fixed by the module (OV2640 on the AI-Thinker board). They are listed
// here so that the sketch never has magic numbers and docs/hardware/pinout.md can point here.

// ---- identity: must match the box it belongs to (same kit number) ---------------------------
const int CAM_KIT_ID = 6;  // 1..10, kit number on the sticker; same as BOX_ID in firmware/box

// ---- on-board LEDs ------------------------------------------------------------------------------
const int CAM_LED_PIN   = 33;  // small red LED on the back (active LOW on most boards)
const int CAM_FLASH_PIN = 4;   // white flash LED, also SD card D1: keep LOW unless needed

// ---- OV2640 camera pins (AI-Thinker ESP32-CAM) -------------------------------------------------
#define CAM_PIN_PWDN   32
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD   26
#define CAM_PIN_SIOC   27
#define CAM_PIN_D7     35
#define CAM_PIN_D6     34
#define CAM_PIN_D5     39
#define CAM_PIN_D4     36
#define CAM_PIN_D3     21
#define CAM_PIN_D2     19
#define CAM_PIN_D1     18
#define CAM_PIN_D0      5
#define CAM_PIN_VSYNC  25
#define CAM_PIN_HREF   23
#define CAM_PIN_PCLK   22
