#include "RoverLink.h"
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <esp_netif.h>

static const uint16_t kHttpTimeoutMs = 400;  // a command normally takes 20-200 ms
static const uint8_t kStopRepeats = 3;       // confirmed stops after driving
static const uint8_t kLostAfterFails = 10;   // consecutive failures -> search the rover again
static const uint32_t kDeadmanMs = 500;      // set point older than this -> stop
static const uint32_t kSearchEveryMs = 2000;
static const uint32_t kIdleEveryMs = 500;    // idle: stop or status poll, alternately

// Percent-encodes everything except unreserved URL characters (the JSON goes into a query string).
static String urlEncode(const String& text) {
  const char* hexDigits = "0123456789ABCDEF";
  String encoded;
  for (size_t i = 0; i < text.length(); i++) {
    const char character = text[i];
    if (isalnum((unsigned char)character) || character == '-' || character == '_' || character == '.' || character == '~') {
      encoded += character;
    } else {
      encoded += '%';
      encoded += hexDigits[(character >> 4) & 0xF];
      encoded += hexDigits[character & 0xF];
    }
  }
  return encoded;
}

void RoverLink::begin(uint32_t periodMs) {
  _periodMs = periodMs;
  xTaskCreatePinnedToCore(_taskEntry, "roverLink", 6144, this, 1, nullptr, 0);
}

void RoverLink::setDrive(float left, float right) {
  portENTER_CRITICAL(&_setPointMux);
  _setLeft = left;
  _setRight = right;
  _setTimeMs = millis();
  portEXIT_CRITICAL(&_setPointMux);
}

String RoverLink::statusText() const {
  if (!connected()) return "Rover: searching ...";
  String statusLine = "Rover " + ip().toString();
  const float voltageV = _voltage;
  if (voltageV > 0) statusLine += ", " + String(voltageV, 2) + " V";
  statusLine += ", " + String((float)_rttMs, 0) + " ms";
  return statusLine;
}

void RoverLink::_taskEntry(void* self) { static_cast<RoverLink*>(self)->_task(); }

// Sends one JSON command to ip. Returns the HTTP code (200 = ok, < 0 = connection error).
// Counts successes/failures; after kLostAfterFails failures in a row the rover counts as lost.
int RoverLink::_sendCommand(uint32_t roverIp, const String& json, String* reply) {
  HTTPClient http;
  http.setTimeout(kHttpTimeoutMs);
  http.setConnectTimeout(kHttpTimeoutMs);
  const uint32_t requestStartMs = millis();
  if (!http.begin("http://" + IPAddress(roverIp).toString() + "/js?json=" + urlEncode(json))) return -100;
  const int httpCode = http.GET();
  if (httpCode == 200) {
    const String body = http.getString();
    if (reply) *reply = body;
    const float rttMs = millis() - requestStartMs;
    _rttMs = (_rttMs == 0) ? rttMs : 0.8f * _rttMs + 0.2f * rttMs;  // smoothed
    _okCount++;
    _failsInRow = 0;
  } else {
    _failCount++;
    if (++_failsInRow >= kLostAfterFails && _roverIp != 0) {
      Serial.println("[rover] lost, searching again");
      _roverIp = 0;
    }
  }
  http.end();
  return httpCode;
}

// The rover is the station on our access point that answers {"T":405} with its Wi-Fi config.
// The IP is only published (connected() = true) after that check.
bool RoverLink::_find() {
  wifi_sta_list_t stationList;
  if (esp_wifi_ap_get_sta_list(&stationList) != ESP_OK) return false;
  static uint32_t lastLogMs = 0;
  if (millis() - lastLogMs > 10000) {
    lastLogMs = millis();
    Serial.printf("[rover] searching, %d station(s) on the AP\n", stationList.num);
  }
  esp_netif_t* apNetif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  for (int i = 0; i < stationList.num; i++) {
    esp_netif_pair_mac_ip_t stationAddress = {};
    memcpy(stationAddress.mac, stationList.sta[i].mac, 6);
    if (esp_netif_dhcps_get_clients_by_mac(apNetif, 1, &stationAddress) != ESP_OK || stationAddress.ip.addr == 0) continue;
    String reply;
    if (_sendCommand(stationAddress.ip.addr, "{\"T\":405}", &reply) == 200 && reply.indexOf("wifi_mode_on_boot") >= 0) {
      _failsInRow = 0;
      _roverIp = stationAddress.ip.addr;
      Serial.printf("[rover] found at %s\n", IPAddress(_roverIp).toString().c_str());
      return true;
    }
  }
  return false;
}

void RoverLink::_task() {
  uint32_t lastSearchMs = 0, lastIdleMs = 0;
  uint8_t stopsLeft = 0;
  bool idleStatus = false;
  for (;;) {
    const uint32_t cycleStartMs = millis();
    const uint32_t roverIp = _roverIp;
    if (roverIp == 0) {
      if (cycleStartMs - lastSearchMs >= kSearchEveryMs) {
        lastSearchMs = cycleStartMs;
        _find();
      }
      vTaskDelay(pdMS_TO_TICKS(_periodMs));
      continue;
    }

    float driveLeft, driveRight;
    uint32_t setTimeMs;
    portENTER_CRITICAL(&_setPointMux);
    driveLeft = _setLeft; driveRight = _setRight; setTimeMs = _setTimeMs;
    portEXIT_CRITICAL(&_setPointMux);
    if (millis() - setTimeMs > kDeadmanMs) driveLeft = driveRight = 0;  // millis() read after setTimeMs: never negative

    if (driveLeft != 0 || driveRight != 0) {
      _sendCommand(roverIp, String("{\"T\":1,\"L\":") + String(driveLeft, 2) + ",\"R\":" + String(driveRight, 2) + "}", nullptr);
      stopsLeft = kStopRepeats;
    } else if (stopsLeft) {
      // a stop only counts when the rover confirmed it (HTTP 200); lost stops are repeated
      if (_sendCommand(roverIp, "{\"T\":1,\"L\":0,\"R\":0}", nullptr) == 200) stopsLeft--;
    } else if (cycleStartMs - lastIdleMs >= kIdleEveryMs) {
      // idle: alternately repeat the stop (clears anything stale) and read the battery voltage
      lastIdleMs = cycleStartMs;
      idleStatus = !idleStatus;
      String reply;
      if (!idleStatus) {
        _sendCommand(roverIp, "{\"T\":1,\"L\":0,\"R\":0}", nullptr);
      } else if (_sendCommand(roverIp, "{\"T\":130}", &reply) == 200) {
        const int voltageIndex = reply.indexOf("\"v\":");
        if (voltageIndex >= 0) _voltage = reply.substring(voltageIndex + 4).toFloat();
      }
    }
    // keep the period; one command can take up to ~200 ms (rover Wi-Fi power save)
    const uint32_t usedMs = millis() - cycleStartMs;
    vTaskDelay(pdMS_TO_TICKS(usedMs < _periodMs ? _periodMs - usedMs : 1));
  }
}
