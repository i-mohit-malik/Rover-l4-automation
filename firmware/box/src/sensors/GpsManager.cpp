#include "GpsManager.h"

static const int kRxPin = 16;  // ESP32 RX2 <- GPS TX
static const int kTxPin = 17;  // ESP32 TX2 -> GPS RX
static const uint32_t kBaudRate = 9600;
static const uint32_t kMaxFixAgeMs = 3000;

void GpsManager::begin() { Serial2.begin(kBaudRate, SERIAL_8N1, kRxPin, kTxPin); }

void GpsManager::poll() {
  while (Serial2.available()) _parser.encode(Serial2.read());
}

bool GpsManager::hasFix() { return _parser.location.isValid() && _parser.location.age() < kMaxFixAgeMs; }
double GpsManager::latDeg() { return _parser.location.lat(); }
double GpsManager::lonDeg() { return _parser.location.lng(); }
int GpsManager::satellites() { return _parser.satellites.isValid() ? (int)_parser.satellites.value() : 0; }
float GpsManager::hdop() { return _parser.hdop.isValid() ? (float)_parser.hdop.hdop() : 0.0f; }
uint32_t GpsManager::charsReceived() { return _parser.charsProcessed(); }
