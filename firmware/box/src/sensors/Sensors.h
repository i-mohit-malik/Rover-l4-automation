#pragma once
// Sensors - all box sensors behind one object.
//
// Load budget: the ultrasonic sensors run in their own task on core 0 (UsManager); LiDAR, GPS and
// IMU are read in update() on core 1. The web page never touches a sensor: it gets a compact
// snapshot (toJson) that is rebuilt 4 times per second; the LiDAR is reduced to 90 points of 4 deg.
// Missing sensors never block: the IMU is started only once it answers on I2C (retry every 5 s).
//
// Two kinds of accessors:
//   snapshot (usCm, toJson)  4 Hz, for the page
//   live (everything else)   latest value: IMU read every 20 ms, ultrasonic as measured by its
//                            task, LiDAR per degree from the filtered map. For the control step
//                            and the log row. Call from loop() only (the LiDAR map and the IMU
//                            are updated there; LidarScan and the I2C bus are not thread-safe).
#include <Arduino.h>
#include "UsManager.h"
#include "LidarScan.h"
#include "GpsManager.h"
#include "ImuManager.h"

class Sensors {
public:
  static const uint8_t kLidarPoints = 90;  // LiDAR points of 4 deg, index 0 = front, clockwise
  static const int kUsMaxCm = 300;         // ultrasonic: "nothing in range" reads as this value

  // Starts I2C and all sensors. lidarOffsetDeg rotates the LiDAR view so that point 0 is the box
  // front (90 for the box as mounted). useCompass = false: IMU in IMUPLUS mode (gyro only, yaw
  // relative to power-on, no magnetometer calibration needed), see ImuManager.
  void begin(int lidarOffsetDeg, bool useCompass = true);
  void update();               // call every loop pass (core 1)
  String toJson() const;       // snapshot for the web page (~650 bytes)

  // Ultrasonic distance in cm, updated 4 times per second (kUsMaxCm = nothing in range).
  // i: 0 front, 1 left, 2 right, 3 back, 4 front angled
  int usCm(uint8_t sensorIndex) const { return sensorIndex < 5 ? _ultrasonicCm[sensorIndex] : kUsMaxCm; }

  // ---- live accessors (control step, log row) ---------------------------------------------
  int usNowCm(uint8_t sensorIndex) const;              // ultrasonic as last measured, same order as usCm()
  bool isImuOk() const { return _isImuReady; }
  float yawDeg() const { return _isImuReady ? _imu.yawDeg() : 0; }        // ISO 8855, + = turning left
  float pitchDeg() const { return _isImuReady ? _imu.pitchDeg() : 0; }    // + = front down
  float rollDeg() const { return _isImuReady ? _imu.rollDeg() : 0; }      // + = right side down
  float yawRateDps() const { return _isImuReady ? _imu.yawRateDps() : 0; }  // + = turning left
  uint16_t imuCalPacked() const;             // calibration as 4 digits sys/gyro/accel/mag, 3333 = full
  bool imuCalibrationLoaded() const { return _isImuReady && _imu.calibrationLoaded(); }
  bool saveImuCalibration();                 // stores the BNO055 profile in flash (all levels 3, ~50 ms)
  bool isLidarOk() const { return _isLidarOk; }
  // LiDAR distance in cm at one degree, vehicle frame (0 = front, 90 = right, 180 = back,
  // 270 = left; any integer, wrapped). 0 = no reliable return.
  uint16_t lidarCm(int vehicleDeg) const;
  // Nearest LiDAR return in cm within +-halfWidthDeg of centerDeg (1-degree bins). 0 = none.
  uint16_t lidarMinCm(int centerDeg, int halfWidthDeg) const;
  bool hasGpsFix() const { return _hasGpsFix; }
  double latDeg() const { return _latitudeDeg; }     // last fix position (0 before the first fix)
  double lonDeg() const { return _longitudeDeg; }

  // Stops the ultrasonic task (firmware update: give the Wi-Fi all the time). Not resumed.
  void suspendUltrasonic() { _ultrasonic.suspend(); }

private:
  void _snapshotLidar();
  void _snapshotGps();
  void _startImuIfPresent();

  UsManager _ultrasonic;
  LidarScan _lidar;
  GpsManager _gps;
  ImuManager _imu;

  int _lidarOffsetDeg = 0;
  bool _useCompass = true;
  uint32_t _lastSnapshotMs = 0, _lastImuAttemptMs = 0, _lastPacketRateMs = 0, _lastPacketCount = 0;

  // snapshot values (loop() only)
  int _ultrasonicCm[5] = {kUsMaxCm, kUsMaxCm, kUsMaxCm, kUsMaxCm, kUsMaxCm};
  bool _isLidarOk = false;
  float _lidarPacketsPerS = 0, _lidarCrcErrorPct = 0;
  uint16_t _lidarCm[kLidarPoints] = {0};  // closest reliable return per 4 deg in cm, 0 = none
  bool _isImuReady = false;
  float _yawSnapshotDeg = 0, _pitchSnapshotDeg = 0, _rollSnapshotDeg = 0;
  uint8_t _imuCalibration[4] = {0, 0, 0, 0};
  bool _hasGpsFix = false;
  int _satelliteCount = 0;
  float _hdop = 0;
  double _latitudeDeg = 0, _longitudeDeg = 0;
  uint32_t _gpsCharCount = 0;
};

bool i2cPresent(uint8_t address);  // true if a device acknowledges on the I2C bus
