#include "Sensors.h"
#include <Wire.h>

static const uint8_t kImuI2cAddress = 0x28;
static const uint32_t kSnapshotMs = 250;
static const uint32_t kImuRetryMs = 5000;

bool i2cPresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void Sensors::begin(int lidarOffsetDeg, bool useCompass) {
  _lidarOffsetDeg = lidarOffsetDeg;
  _useCompass = useCompass;
  Wire.begin(21, 22);                 // I2C: IMU + OLED

  _ultrasonic.begin(kUsMaxCm, 17500);         // 17.5 ms echo timeout ~ 3 m

  LidarScan::Config lidarConfig;               // GPIO27/33, 230400 baud, filtering see LidarScan.h
  _lidar.begin(Serial1, lidarConfig);

  _gps.begin();                       // Serial2, GPIO16/17, 9600 baud

  // IMU right away, as the CampusLogger sketch did in setup() (retried every kImuRetryMs if missing)
  _lastImuAttemptMs = millis() - kImuRetryMs;
  _startImuIfPresent();
}

void Sensors::update() {
  _lidar.update();  // drain both UARTs every pass
  _gps.poll();
  if (_isImuReady) _imu.update(); else _startImuIfPresent();

  const uint32_t nowMs = millis();
  if (nowMs - _lastSnapshotMs < kSnapshotMs) return;
  _lastSnapshotMs = nowMs;

  _ultrasonicCm[0] = _ultrasonic.readFrontCm();
  _ultrasonicCm[1] = _ultrasonic.readLeftCm();
  _ultrasonicCm[2] = _ultrasonic.readRightCm();
  _ultrasonicCm[3] = _ultrasonic.readBackCm();
  _ultrasonicCm[4] = _ultrasonic.readFrontAngledCm();
  _snapshotLidar();
  _snapshotGps();
  if (_isImuReady) {
    _yawSnapshotDeg = _imu.yawDeg();
    _pitchSnapshotDeg = _imu.pitchDeg();
    _rollSnapshotDeg = _imu.rollDeg();
    _imu.calibration(_imuCalibration[0], _imuCalibration[1], _imuCalibration[2], _imuCalibration[3]);
  }
}

// With the compass the IMU start blocks ~3.5 s once (north alignment); RoverLink's deadman stops the
// rover meanwhile. Without it (IMUPLUS) the start takes well under a second.
void Sensors::_startImuIfPresent() {
  const uint32_t nowMs = millis();
  if (nowMs - _lastImuAttemptMs < kImuRetryMs) return;
  _lastImuAttemptMs = nowMs;
  if (!i2cPresent(kImuI2cAddress)) return;
  Serial.println(_useCompass ? "[sensors] BNO055 found, starting IMU (~3.5 s)"
                             : "[sensors] BNO055 found, starting IMU (compass off)");
  _isImuReady = _imu.begin(_useCompass);
}

// Packet rate, checksum errors and the 90-point view (closest reliable bin per 4 deg).
void Sensors::_snapshotLidar() {
  const uint32_t nowMs = millis();
  if (nowMs - _lastPacketRateMs >= 1000) {
    const uint32_t packetCount = _lidar.packets();
    _lidarPacketsPerS = (packetCount - _lastPacketCount) * 1000.0f / (nowMs - _lastPacketRateMs);
    _lastPacketCount = packetCount;
    _lastPacketRateMs = nowMs;
  }
  _isLidarOk = _lidarPacketsPerS > 1;
  _lidarCrcErrorPct = _lidar.crcErrorPct();

  const int degreesPerPoint = 360 / kLidarPoints;
  for (uint8_t pointIndex = 0; pointIndex < kLidarPoints; pointIndex++) _lidarCm[pointIndex] = 0;
  for (int binIndex = 0; binIndex < 360; binIndex++) {
    const uint16_t distanceMm = _lidar.distanceMm(binIndex);
    if (!distanceMm) continue;
    const int vehicleDeg = ((binIndex + _lidarOffsetDeg) % 360 + 360) % 360;
    const uint8_t pointIndex = vehicleDeg / degreesPerPoint;
    const uint16_t distanceCm = distanceMm / 10;
    if (_lidarCm[pointIndex] == 0 || distanceCm < _lidarCm[pointIndex]) _lidarCm[pointIndex] = distanceCm;
  }
}

int Sensors::usNowCm(uint8_t sensorIndex) const {
  switch (sensorIndex) {
    case 0: return _ultrasonic.readFrontCm();
    case 1: return _ultrasonic.readLeftCm();
    case 2: return _ultrasonic.readRightCm();
    case 3: return _ultrasonic.readBackCm();
    case 4: return _ultrasonic.readFrontAngledCm();
    default: return kUsMaxCm;
  }
}

uint16_t Sensors::imuCalPacked() const {
  if (!_isImuReady) return 0;
  uint8_t systemLevel, gyroLevel, accelLevel, magLevel;
  _imu.calibration(systemLevel, gyroLevel, accelLevel, magLevel);
  return systemLevel * 1000 + gyroLevel * 100 + accelLevel * 10 + magLevel;
}

bool Sensors::saveImuCalibration() {
  return _isImuReady && _imu.saveCalibration();
}

// Vehicle degree -> LidarScan bin (the inverse of _snapshotLidar: deg = bin + offset).
uint16_t Sensors::lidarCm(int vehicleDeg) const {
  const uint16_t distanceMm = _lidar.distanceMm(vehicleDeg - _lidarOffsetDeg);  // distanceMm wraps the bin itself
  return distanceMm ? distanceMm / 10 : 0;
}

uint16_t Sensors::lidarMinCm(int centerDeg, int halfWidthDeg) const {
  uint16_t nearestCm = 0;
  for (int vehicleDeg = centerDeg - halfWidthDeg; vehicleDeg <= centerDeg + halfWidthDeg; vehicleDeg++) {
    const uint16_t distanceCm = lidarCm(vehicleDeg);
    if (distanceCm && (nearestCm == 0 || distanceCm < nearestCm)) nearestCm = distanceCm;
  }
  return nearestCm;
}

void Sensors::_snapshotGps() {
  _hasGpsFix = _gps.hasFix();
  if (_hasGpsFix) {
    _latitudeDeg = _gps.latDeg();
    _longitudeDeg = _gps.lonDeg();
  }
  _satelliteCount = _gps.satellites();
  _hdop = _gps.hdop();
  _gpsCharCount = _gps.charsReceived();
}

// {"us":[5 x cm],"usMax":..,"lidar":{ok,pps,crc,d:[90 x cm]},
//  "imu":{ok,yaw,pitch,roll,rate,compass,calSaved,cal:[4]},
//  "gps":{fix,sats,hdop,lat,lon,chars}}
String Sensors::toJson() const {
  String json;
  json.reserve(780);
  json += "{\"us\":[";
  for (int i = 0; i < 5; i++) { if (i) json += ','; json += _ultrasonicCm[i]; }
  json += "],\"usMax\":"; json += kUsMaxCm;
  json += ",\"lidar\":{\"ok\":"; json += _isLidarOk ? "true" : "false";
  json += ",\"pps\":"; json += String(_lidarPacketsPerS, 0);
  json += ",\"crc\":"; json += String(_lidarCrcErrorPct, 1);
  json += ",\"d\":[";
  for (int i = 0; i < kLidarPoints; i++) { if (i) json += ','; json += _lidarCm[i]; }
  json += "]},\"imu\":{\"ok\":"; json += _isImuReady ? "true" : "false";
  json += ",\"yaw\":"; json += String(_yawSnapshotDeg, 1);
  json += ",\"pitch\":"; json += String(_pitchSnapshotDeg, 1);
  json += ",\"roll\":"; json += String(_rollSnapshotDeg, 1);
  json += ",\"rate\":"; json += String(yawRateDps(), 1);
  json += ",\"compass\":"; json += _useCompass ? "true" : "false";
  json += ",\"calSaved\":"; json += imuCalibrationLoaded() ? "true" : "false";
  json += ",\"cal\":["; json += _imuCalibration[0]; json += ','; json += _imuCalibration[1]; json += ','; json += _imuCalibration[2]; json += ','; json += _imuCalibration[3];
  json += "]},\"gps\":{\"fix\":"; json += _hasGpsFix ? "true" : "false";
  json += ",\"sats\":"; json += _satelliteCount;
  json += ",\"hdop\":"; json += String(_hdop, 1);
  json += ",\"lat\":"; json += String(_latitudeDeg, 6);
  json += ",\"lon\":"; json += String(_longitudeDeg, 6);
  json += ",\"chars\":"; json += _gpsCharCount;
  json += "}}";
  return json;
}
