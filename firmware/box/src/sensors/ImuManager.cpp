#include "ImuManager.h"
#include <Wire.h>
#include <math.h>
#include <Adafruit_BNO055.h>
#include <Preferences.h>

static const uint8_t kBnoI2cAddress = 0x28;
static const uint32_t kUpdatePeriodMs = 20;   // 50 Hz: the control step always gets a fresh value
static const uint16_t kNorthAlignMs = 3000;
static const char* kPrefsNamespace = "imu";   // flash storage for the calibration profile
static const char* kPrefsKey = "offsets";

static Adafruit_BNO055 bnoSensor(55, kBnoI2cAddress, &Wire);

// Wraps an angle to -180..180 degrees.
static float wrap180(float angleDeg) {
  angleDeg = fmodf(angleDeg + 180.0f, 360.0f);
  if (angleDeg < 0) angleDeg += 360.0f;
  return angleDeg - 180.0f;
}

// Tilt-compensated magnetic heading: projects the magnetometer vector into the horizontal plane
// using roll and pitch. Returns false if the horizontal field is ~0.
static bool magneticHeadingDeg(float rollDeg, float pitchDeg, float& headingDeg) {
  imu::Vector<3> magField = bnoSensor.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  const float rollRad = rollDeg * DEG_TO_RAD, pitchRad = pitchDeg * DEG_TO_RAD;
  const float horizontalX = magField.x() * cosf(pitchRad) + magField.z() * sinf(pitchRad);
  const float horizontalY = magField.x() * sinf(rollRad) * sinf(pitchRad) + magField.y() * cosf(rollRad) - magField.z() * sinf(rollRad) * cosf(pitchRad);
  if (sqrtf(horizontalX * horizontalX + horizontalY * horizontalY) < 1e-6f) return false;
  headingDeg = wrap180(atan2f(horizontalY, horizontalX) * RAD_TO_DEG);
  return true;
}

bool ImuManager::begin(bool useCompass) {
  if (!bnoSensor.begin()) return false;
  bnoSensor.setExtCrystalUse(true);
  _isCalibrationLoaded = _loadCalibration();   // before the north alignment, so it uses the calibrated compass
  Serial.println(_isCalibrationLoaded ? "[imu] saved calibration loaded" : "[imu] no saved calibration");
  _isReady = true;
  if (useCompass) {
    bnoSensor.setMode(OPERATION_MODE_NDOF);
    _alignNorth(kNorthAlignMs);
  } else {
    // IMUPLUS: gyro + accelerometer only. Yaw is relative to power-on (no north), but metal
    // nearby cannot bend it. The magnetometer is off, so there is nothing to align.
    bnoSensor.setMode(OPERATION_MODE_IMUPLUS);
    _isNorthAnchored = true;
    _northOffsetDeg = 0;
    Serial.println("[imu] IMUPLUS mode: compass off, yaw relative to start");
  }
  return true;
}

void ImuManager::update() {
  if (!_isReady) return;
  const uint32_t nowMs = millis();
  if (nowMs - _lastUpdateMs < kUpdatePeriodMs) return;
  _lastUpdateMs = nowMs;

  // Euler angles from the fusion, sensor frame: x = heading (clockwise), y = roll, z = pitch
  imu::Vector<3> euler = bnoSensor.getVector(Adafruit_BNO055::VECTOR_EULER);
  const float sensorRoll = euler.y(), sensorPitch = euler.z();
  _fusionYawDeg = wrap180(euler.x());
  float magHeadingDeg;
  if (magneticHeadingDeg(sensorRoll, sensorPitch, magHeadingDeg)) _magYawDeg = magHeadingDeg;  // tilt comp. in sensor frame

  // sensor frame -> vehicle frame (ISO 8855), see header: sensor turned by 90 deg
  _pitchDeg = -sensorRoll;   // front down = sensor roll negative  -> ISO pitch positive
  _rollDeg = -sensorPitch;   // right down = sensor pitch negative -> ISO roll positive

  // Turn rate about the vertical axis. Gyro z is counter-clockwise positive (chip z up, as
  // roll/pitch ~0 show), the same sense as ISO yaw -> no sign flip.
  _yawRateDps = bnoSensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z();
  bnoSensor.getCalibration(&_calibration[0], &_calibration[1], &_calibration[2], &_calibration[3]);
}

float ImuManager::yawDeg() const {
  // heading is clockwise from north; ISO yaw is counter-clockwise -> sign flip.
  // Without a north anchor (alignment failed) fall back to the raw magnetic heading.
  const float heading = _isNorthAnchored ? wrap180(_fusionYawDeg + _northOffsetDeg) : _magYawDeg;
  return wrap180(-heading);
}

void ImuManager::calibration(uint8_t& systemLevel, uint8_t& gyroLevel, uint8_t& accelLevel, uint8_t& magLevel) const {
  systemLevel = _calibration[0]; gyroLevel = _calibration[1]; accelLevel = _calibration[2]; magLevel = _calibration[3];
}

// Reads the 22 offset/radius bytes from the BNO055 (library: only when fully calibrated) and
// stores them in flash.
bool ImuManager::saveCalibration() {
  if (!_isReady) return false;
  uint8_t offsets[NUM_BNO055_OFFSET_REGISTERS];
  if (!bnoSensor.getSensorOffsets(offsets)) return false;
  Preferences preferences;
  if (!preferences.begin(kPrefsNamespace, false)) return false;
  const bool isSaved = preferences.putBytes(kPrefsKey, offsets, sizeof(offsets)) == sizeof(offsets);
  preferences.end();
  if (isSaved) Serial.println("[imu] calibration saved to flash");
  return isSaved;
}

// Writes a saved profile back into the BNO055. Note: the CALIB_STAT levels may still read low
// for a while afterwards, although the data is already corrected.
bool ImuManager::_loadCalibration() {
  Preferences preferences;
  if (!preferences.begin(kPrefsNamespace, true)) return false;  // nothing saved yet
  uint8_t offsets[NUM_BNO055_OFFSET_REGISTERS];
  const size_t bytesRead = preferences.getBytes(kPrefsKey, offsets, sizeof(offsets));
  preferences.end();
  if (bytesRead != sizeof(offsets)) return false;
  bnoSensor.setSensorOffsets(offsets);
  return true;
}

// Averages magnetic and fusion heading for durationMs (as unit vectors, so 179/-179 average
// correctly) and stores their difference as north offset.
void ImuManager::_alignNorth(uint16_t durationMs) {
  float sinMag = 0, cosMag = 0, sinFus = 0, cosFus = 0;
  uint16_t sampleCount = 0;
  const uint32_t startMs = millis();
  while (millis() - startMs < durationMs) {
    imu::Vector<3> euler = bnoSensor.getVector(Adafruit_BNO055::VECTOR_EULER);
    float magHeadingDeg;
    if (magneticHeadingDeg(euler.y(), euler.z(), magHeadingDeg)) {
      sinMag += sinf(magHeadingDeg * DEG_TO_RAD);
      cosMag += cosf(magHeadingDeg * DEG_TO_RAD);
      sinFus += sinf(wrap180(euler.x()) * DEG_TO_RAD);
      cosFus += cosf(wrap180(euler.x()) * DEG_TO_RAD);
      sampleCount++;
    }
    delay(20);
  }
  _isNorthAnchored = sampleCount >= 5;
  if (!_isNorthAnchored) {
    Serial.println("[imu] north alignment failed, yaw = raw magnetic heading");
    return;
  }
  _northOffsetDeg = wrap180((atan2f(sinMag, cosMag) - atan2f(sinFus, cosFus)) * RAD_TO_DEG);
  Serial.printf("[imu] north offset %.1f deg\n", _northOffsetDeg);
}
