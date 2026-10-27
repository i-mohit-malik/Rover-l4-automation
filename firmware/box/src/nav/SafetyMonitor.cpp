#include "SafetyMonitor.h"
#include "NavConfig.h"
#include "NavMath.h"

void SafetyMonitor::observe(int cliffRawCm, bool isRunning) {
  // median of the last 3 readings drops single bad echoes
  _cliffHistoryCm[0] = _cliffHistoryCm[1]; _cliffHistoryCm[1] = _cliffHistoryCm[2]; _cliffHistoryCm[2] = cliffRawCm;
  _cliffFilteredCm = medianInt(_cliffHistoryCm, 3);
  if (!isRunning) {  // idle: collect ground readings
    _baselineSamplesCm[_baselineSampleIndex] = cliffRawCm;
    _baselineSampleIndex = (_baselineSampleIndex + 1) % kBaselineSamples;
    if (_baselineSampleCount < kBaselineSamples) _baselineSampleCount++;
  }
}

void SafetyMonitor::beginRun() {
  // ground baseline = median of the last 1.5 s of idle readings (one bad reading cannot shift it)
  const int medianCm = _baselineSampleCount ? medianInt(_baselineSamplesCm, _baselineSampleCount) : BASELINE_DEFAULT_CM;
  _baselineCm = (medianCm >= BASELINE_MIN_CM && medianCm <= BASELINE_MAX_CM) ? medianCm : BASELINE_DEFAULT_CM;
  for (int k = 0; k < 3; k++) _cliffHistoryCm[k] = _baselineCm;
  _obstacleCount = _clearCount = 0;
  _stepDropCount = _noEchoCount = 0;
  _isPaused = false;
  _eventText[0] = 0;
}

int SafetyMonitor::frontDistance(uint16_t corridorCm, int usFrontCm) {
  // the LiDAR lane check decides; the ultrasonic only for very close things (its sound cone hits
  // the ground at ~35-45 cm, which used to cause endless pauses)
  int frontCm = corridorCm;  // 0 = lane clear
  if (usFrontCm > 2 && usFrontCm < US_CLOSE_CM && (frontCm == 0 || usFrontCm < frontCm)) frontCm = usFrontCm;
  return frontCm;
}

bool SafetyMonitor::update(int frontCm, int cliffRawCm, bool isStraight) {
  // obstacle pause & auto-resume (only while driving straight; turning in place is not moving ahead)
  const bool isObstacleClose = frontCm > 0 && frontCm < FRONT_STOP_CM;
  if (isObstacleClose) {
    _obstacleCount++;
    _clearCount = 0;
  } else if (frontCm == 0 || frontCm >= FRONT_RESUME_CM) {
    _clearCount++;
    _obstacleCount = 0;
  }
  if (!_isEnabled || !isStraight) {
    _isPaused = false;
  } else if (_obstacleCount >= FRONT_DEBOUNCE) {
    if (!_isPaused) {
      _isPaused = true;
      Serial.printf("[safety] OBSTACLE PAUSE (distance %d cm)\n", frontCm);
      _note("obstacle");
    }
  } else if (_clearCount >= FRONT_DEBOUNCE) {
    if (_isPaused) {
      _isPaused = false;
      Serial.println("[safety] obstacle cleared, resuming");
      _note("clear");
    }
  }

  // cliff, filtered reading, straight phases only:
  // A) step down: echo returns, but clearly deeper (baseline + CLIFF_DELTA_CM .. CLIFF_MAX_STEP_CM)
  const int stepLimitCm = _baselineCm + CLIFF_DELTA_CM;
  const bool isStepDrop = CLIFF_CHECK_ENABLED && isStraight && _cliffFilteredCm >= stepLimitCm && _cliffFilteredCm <= CLIFF_MAX_STEP_CM;
  _stepDropCount = isStepDrop ? _stepDropCount + 1 : 0;
  // B) prolonged void: no echo (300) for VOID_TIMEOUT_CYCLES steps; rejects short specular
  //    reflections over tactile metal strips and floor lights
  const bool isNoEcho = CLIFF_CHECK_ENABLED && isStraight && cliffRawCm >= 300;
  _noEchoCount = isNoEcho ? _noEchoCount + 1 : 0;

  if (_isEnabled && (_stepDropCount >= CLIFF_DEBOUNCE || _noEchoCount >= VOID_TIMEOUT_CYCLES)) {
    Serial.printf("[safety] CLIFF DETECTED (reading %d cm, baseline %d cm)\n", _cliffFilteredCm, _baselineCm);
    _note("cliff");
    return true;
  }
  return false;
}

void SafetyMonitor::toggle() {
  _isEnabled = !_isEnabled;
  if (!_isEnabled) _isPaused = false;  // switching safety off must also release an active pause
  Serial.printf("[safety] safety stops %s\n", _isEnabled ? "ENABLED" : "DISABLED");
  _note(_isEnabled ? "safety_on" : "safety_off");
}

void SafetyMonitor::_note(const char* text) {
  if (_eventText[0]) strlcat(_eventText, ";", sizeof(_eventText));
  strlcat(_eventText, text, sizeof(_eventText));
}

const char* SafetyMonitor::takeEvent() {
  strlcpy(_eventOutput, _eventText, sizeof(_eventOutput));
  _eventText[0] = 0;
  return _eventOutput;
}
