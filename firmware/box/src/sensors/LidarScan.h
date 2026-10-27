#pragma once
// LidarScan - LD20 LiDAR reader with a stable distance map, one value per degree.
//
// Why a filter: the raw LD20 points are steady (2-60 mm), but about a third of all points are
// "no return" (distance 0). An older reader shrank the stored value on every such point, so the
// view jumped by up to 1 m although nothing moved. Here a "no return" point changes nothing;
// instead:
//   - a bin keeps its last 3 valid hits; its value is their median (single outliers drop out)
//   - a bin counts only with >= 2 hits younger than expireMs (removes ghost points in open
//     directions), otherwise it reads 0 = no return
//   - hits closer than minMm (the box itself) are ignored
// Angle convention: raw angle + 90, rounded to 1 degree -> front = bin 270, clockwise.
// LD20 protocol: 47-byte frames, header 0x54 0x2C, 12 points, CRC8 at the end; ~340 frames/s.
// Header-only (the whole class is in this file).
#include <Arduino.h>

class LidarScan {
public:
  struct Config {
    int rxPin = 27, txPin = 33;   // ESP32 RX <- LD20 TX, ESP32 TX -> LD20 RX (unused by the LD20)
    uint32_t baud = 230400;       // LD20 default
    int angleShiftDeg = 90;   // raw angle + 90 -> front = bin 270
    uint16_t minMm = 200;     // ignore the box itself
    uint16_t maxMm = 12000;   // LD20 range
    uint16_t expireMs = 500;  // LD20 turns at ~10 Hz -> ~5 revolutions
  };

  // Builds the CRC table, so feed() works even without begin().
  LidarScan() {
    for (int i = 0; i < 256; i++) {  // CRC8, poly 0x4D (LD20 protocol)
      uint8_t crc = i;
      for (int bit = 0; bit < 8; bit++) crc = (crc & 0x80) ? (crc << 1) ^ 0x4D : crc << 1;
      _crcTable[i] = crc;
    }
  }

  // Opens the UART. Use Serial1 (Serial2 is the GPS).
  void begin(HardwareSerial& port, const Config& cfg) {
    _config = cfg;
    _port = &port;
    _port->setRxBufferSize(4096);  // ~23 kB/s: survive slow loop passes
    _port->begin(_config.baud, SERIAL_8N1, _config.rxPin, _config.txPin);
  }

  void update() {  // drain the UART, call every loop pass
    if (!_port) return;
    while (_port->available()) feed(_port->read());
  }

  // Frame parser, one byte at a time. Valid frames go into the distance map.
  void feed(uint8_t byteIn) {
    if (_frameIndex == 0 && byteIn != 0x54) return;  // wait for header
    if (_frameIndex == 1 && byteIn != 0x2C) {        // second byte must be ver_len
      _frameIndex = (byteIn == 0x54) ? 1 : 0;        // ...unless this byte is the next header
      return;
    }
    _frameBuffer[_frameIndex++] = byteIn;
    if (_frameIndex < sizeof(Frame)) return;
    _frameIndex = 0;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(Frame) - 1; i++) crc = _crcTable[crc ^ _frameBuffer[i]];
    if (crc != _frameBuffer[sizeof(Frame) - 1]) { _crcErrorCount++; return; }
    _packetCount++;
    _handle(reinterpret_cast<const Frame*>(_frameBuffer));
  }

  // Distance of one bin in mm, 0 = no (reliable) return.
  uint16_t distanceMm(int binIndex) const {
    const Bin& binData = _bins[((binIndex % 360) + 360) % 360];
    uint32_t nowMs = millis();
    uint16_t validMm[3];
    int validCount = 0;
    for (int k = 0; k < 3; k++)
      if (binData.hitMm[k] && nowMs - binData.hitMs[k] <= _config.expireMs) validMm[validCount++] = binData.hitMm[k];
    if (validCount < 2) return 0;
    if (validCount == 2) return (validMm[0] + validMm[1]) / 2;
    if (validMm[0] > validMm[1]) { uint16_t swap = validMm[0]; validMm[0] = validMm[1]; validMm[1] = swap; }  // median of 3
    if (validMm[1] > validMm[2]) { validMm[1] = validMm[2]; }
    return validMm[0] > validMm[1] ? validMm[0] : validMm[1];
  }

  uint32_t packets() const { return _packetCount; }  // valid frames since start
  float crcErrorPct() const {                     // share of frames with a wrong checksum
    uint32_t totalFrames = _packetCount + _crcErrorCount;
    return totalFrames ? 100.0f * _crcErrorCount / totalFrames : 0;
  }

private:
  struct __attribute__((packed)) Pt { uint16_t distanceMm; uint8_t intensity; };
  struct __attribute__((packed)) Frame {
    uint8_t header, verLen; uint16_t speed, startAngleCentiDeg; Pt points[12]; uint16_t endAngleCentiDeg, timestamp; uint8_t crc;
  };
  struct Bin { uint16_t hitMm[3] = {0, 0, 0}; uint32_t hitMs[3] = {0, 0, 0}; uint8_t nextSlot = 0; };

  // Spreads the 12 points between start and end angle and stores the valid ones.
  void _handle(const Frame* frame) {
    float startAngleDeg = frame->startAngleCentiDeg / 100.0f, endAngleDeg = frame->endAngleCentiDeg / 100.0f;
    if (endAngleDeg < startAngleDeg) endAngleDeg += 360;
    float stepDeg = (endAngleDeg - startAngleDeg) / 11;
    uint32_t nowMs = millis();
    for (int k = 0; k < 12; k++) {
      uint16_t pointMm = frame->points[k].distanceMm;
      if (pointMm < _config.minMm || pointMm >= _config.maxMm) continue;  // no return / too close: no change
      int binIndex = ((int)lroundf(startAngleDeg + stepDeg * k) + _config.angleShiftDeg) % 360;
      Bin& binData = _bins[binIndex];
      binData.hitMm[binData.nextSlot] = pointMm;
      binData.hitMs[binData.nextSlot] = nowMs;
      binData.nextSlot = (binData.nextSlot + 1) % 3;
    }
  }

  Config _config;
  HardwareSerial* _port = nullptr;
  uint8_t _crcTable[256];
  uint8_t _frameBuffer[sizeof(Frame)];
  uint8_t _frameIndex = 0;
  uint32_t _packetCount = 0, _crcErrorCount = 0;
  Bin _bins[360];
};
