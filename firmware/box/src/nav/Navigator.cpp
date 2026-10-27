#include "Navigator.h"
#include <math.h>
#include "NavConfig.h"
#include "NavMath.h"
#include "../core/Config.h"

static const char* const kPhaseNames[] = {"LEG1", "WAIT", "TURN", "SETTLE", "LEG2"};

void Navigator::start(const Sensors& sensors, uint32_t nowMs) {
  if (_runState == RUNNING) return;
  const float yawDeg = sensors.isImuOk() ? sensors.yawDeg() : 0;
  _lineTracker.reset(yawDeg);
  _safetyMonitor.beginRun();
  _phase = NAV_LEG1;
  _headingErrorIntegral = 0;
  _turnPulseCount = 0;
  _startedAtMs = _runClockMs = _phaseStartMs = nowMs;
  _runState = RUNNING;
  Serial.printf("[nav] run STARTED, target yaw %.1f deg, cliff baseline %d cm\n", yawDeg, _safetyMonitor.baselineCm());
  _note("run_start");
}

void Navigator::stop(RunState reason) {
  if (_runState != RUNNING || reason == RUNNING || reason == IDLE) return;
  _runState = reason;
  static const char* const stopEventNames[] = {"", "", "stop_manual", "stop_override", "stop_link", "stop_cliff", "stop_timeout"};
  Serial.printf("[nav] run %s after %.1f s, %d poles\n", stopEventNames[reason], (millis() - _startedAtMs) / 1000.0f, _lineTracker.poleCount());
  _note(stopEventNames[reason]);
}

void Navigator::step(const Sensors& sensors, uint32_t nowMs, float stepS, uint32_t pageAgeMs, float& driveLeft, float& driveRight) {
  const bool isImuOk = sensors.isImuOk();
  const float yawDeg = isImuOk ? sensors.yawDeg() : 0;
  const float yawRateDps = isImuOk ? sensors.yawRateDps() : 0;
  const int usFrontCm = sensors.usNowCm(0);
  const int cliffRawCm = sensors.usNowCm(4);

  _snapshot.yawRateDps = yawRateDps;
  _snapshot.corridorCm = Perception::corridorCm(sensors);
  _snapshot.frontCm = SafetyMonitor::frontDistance(_snapshot.corridorCm, usFrontCm);
  _snapshot.pole = PoleSeen{false, 0, 0, 0, 0};  // set in _straightStep (LEG1) or below (no run)
  _snapshot.wall = WallSeen{false, 0, 0, 0};

  // link watchdog: phone page silent for too long -> stop (the STOP button would not reach us)
  if (_runState == RUNNING && pageAgeMs > LINK_TIMEOUT_MS) {
    Serial.printf("[safety] PHONE LINK LOST (%lu ms silent)\n", (unsigned long)pageAgeMs);
    stop(STOPPED_LINK);
  }
  _safetyMonitor.observe(cliffRawCm, _runState == RUNNING);

  driveLeft = driveRight = 0;
  if (_runState == RUNNING) {
    const bool isStraight = _phase == NAV_LEG1 || _phase == NAV_WAIT || _phase == NAV_LEG2;
    _lineTracker.addOdometry(stepS);
    const bool isCliff = _safetyMonitor.update(_snapshot.frontCm, cliffRawCm, isStraight);

    if (isCliff) {
      stop(STOPPED_CLIFF);
    } else if (_safetyMonitor.isPaused()) {
      // obstacle ahead: stand and wait; do not wind up the integral, pause the clocks
      _headingErrorIntegral = 0;
      _runClockMs += (uint32_t)(stepS * 1000);
      _phaseStartMs += (uint32_t)(stepS * 1000);
    } else if (MAX_RUN_TIME_MS > 0 && nowMs - _runClockMs > MAX_RUN_TIME_MS) {
      stop(STOPPED_TIMEOUT);
    } else if (_phase == NAV_TURN) {
      // full-speed turn in place (slower does not rotate); stop early by rate x lead time, because
      // the stop command needs ~0.2 s to take effect
      const float remainingDeg = angleDiff(yawDeg, _turnTargetYawDeg);  // + = still have to turn right
      const float turnRateDps = isImuOk ? fabsf(yawRateDps) : 0.0f;
      if (fabsf(remainingDeg) <= turnRateDps * TURN_LEAD_S + 1.0f || nowMs - _phaseStartMs > TURN_PULSE_MAX_MS) {
        _phase = NAV_SETTLE;
        _phaseStartMs = nowMs;
      } else {
        driveLeft = remainingDeg > 0 ? TURN_SPEED : -TURN_SPEED;  // left wheels forward = turn right
        driveRight = -driveLeft;
      }
    } else if (_phase == NAV_SETTLE) {
      if (nowMs - _phaseStartMs >= TURN_SETTLE_MS) {
        const float errorDeg = angleDiff(yawDeg, _turnTargetYawDeg);
        Serial.printf("[nav] turn pulse %d done, error %.1f deg\n", _turnPulseCount + 1, errorDeg);
        if (fabsf(errorDeg) > TURN_TOLERANCE_DEG && ++_turnPulseCount < TURN_MAX_PULSES) {
          _phase = NAV_TURN;  // another full-speed pulse (either direction)
          _note("turn_pulse");
        } else {
          _phase = NAV_LEG2;  // new straight line along the turned heading
          _lineTracker.startLeg(_turnTargetYawDeg);
          _headingErrorIntegral = 0;
          _note("leg2");
        }
        _phaseStartMs = nowMs;
      }
    } else {
      _straightStep(sensors, nowMs, stepS, yawDeg, yawRateDps, driveLeft, driveRight);
    }
  }
  if (_runState != RUNNING) {  // no run: detect anyway, for the page and the log
    _snapshot.pole = Perception::detectPoleRight(sensors);
    _snapshot.wall = Perception::detectWallLeft(sensors);
  }
  _lineTracker.setCommand(driveLeft, driveRight);  // odometry of the next step
  _snapshot.headingRefDeg = _lineTracker.lastHeadingRefDeg();
  _snapshot.crossTrackCm = _lineTracker.crossTrackCm();

  // collect the events of the parts, so hasEvent() sees them
  const char* lineEvent = _lineTracker.takeEvent();
  if (lineEvent[0]) _note(lineEvent);
  const char* safetyEvent = _safetyMonitor.takeEvent();
  if (safetyEvent[0]) _note(safetyEvent);
}

// Straight phases. LEG1 counts poles; after the last one WAIT keeps driving, then TURN.
void Navigator::_straightStep(const Sensors& sensors, uint32_t nowMs, float stepS, float yawDeg, float yawRateDps,
                              float& driveLeft, float& driveRight) {
  if (_phase == NAV_LEG1) {
    _snapshot.pole = Perception::detectPoleRight(sensors);
    _lineTracker.updatePoles(_snapshot.pole);
    if (_lineTracker.poleCount() >= POLES_BEFORE_TURN) {
      _phase = NAV_WAIT;
      _phaseStartMs = nowMs;
      Serial.printf("[nav] %d poles passed, turning right in %lu ms\n", _lineTracker.poleCount(), (unsigned long)TURN_DELAY_MS);
    }
  } else if (_phase == NAV_WAIT && nowMs - _phaseStartMs >= TURN_DELAY_MS) {
    _phase = NAV_TURN;
    _phaseStartMs = nowMs;
    _turnPulseCount = 0;
    _turnTargetYawDeg = angleDiff(_lineTracker.targetYawDeg() - 90.0f, 0.0f);  // 90 deg right (ISO yaw: right = minus)
    _note("turn");
  }

  // left wall (if visible): keep our line parallel to it at a constant distance
  if (_phase != NAV_TURN) {
    _snapshot.wall = Perception::detectWallLeft(sensors);
    _lineTracker.updateWall(_snapshot.wall);
  }

  // virtual line, then PID heading hold toward its reference; the trim cancels the built-in pull
  const float headingRefDeg = _lineTracker.updateHeadingRefDeg(yawDeg, stepS);
  const float yawErrorDeg = angleDiff(yawDeg, headingRefDeg);
  _headingErrorIntegral = constrain(_headingErrorIntegral + yawErrorDeg * stepS, -HEADING_I_LIMIT, HEADING_I_LIMIT);
  const float steering = STEER_TRIM + yawErrorDeg * HEADING_KP + _headingErrorIntegral * HEADING_KI + yawRateDps * HEADING_KD;
  driveLeft = constrain(CRUISE_SPEED + steering, -DRIVE_LIMIT, DRIVE_LIMIT);
  driveRight = constrain(CRUISE_SPEED - steering, -DRIVE_LIMIT, DRIVE_LIMIT);
  if (_phase == NAV_TURN) driveLeft = driveRight = 0;  // WAIT just ended: the turn starts next step
}

const char* Navigator::stateName() const {
  return _runState == RUNNING ? kPhaseNames[_phase] : "MANUAL";
}

String Navigator::statusText() const {
  switch (_runState) {
    case RUNNING:          return _safetyMonitor.isPaused() ? "PAUSED (OBSTACLE AHEAD)" : "RUNNING";
    case STOPPED_MANUAL:   return "STOPPED (STOP BUTTON)";
    case STOPPED_OVERRIDE: return "STOPPED (DRIVE PAD OVERRIDE)";
    case STOPPED_LINK:     return "STOPPED (PHONE LINK LOST)";
    case STOPPED_CLIFF:    return "STOPPED (CLIFF DETECTED)";
    case STOPPED_TIMEOUT:  return "STOPPED (TIME LIMIT)";
    default:               return "IDLE";
  }
}

// {"state":"RUNNING","text":"..","nav":"..","phase":"LEG1","paused":false,"safety":true,
//  "poles":1,"polesTarget":4,"elapsedS":12.3,"headingRef":..,"crossTrack":..,"front":..,
//  "wall":cm|0,"wallDeg":..,"wallSteer":false,"baseline":17}
String Navigator::toJson() const {
  const char* runName = _runState == RUNNING ? "RUNNING" : (_runState == IDLE ? "IDLE" : "STOPPED");
  const char* navText = "-";
  if (_runState == RUNNING) {
    if (_phase == NAV_LEG1) navText = "DRIVING - counting poles";
    else if (_phase == NAV_WAIT) navText = "LAST POLE PASSED - turning soon";
    else if (_phase == NAV_TURN || _phase == NAV_SETTLE) navText = "TURNING RIGHT 90 deg";
    else navText = "DRIVING - after the turn";
  }
  const float elapsedS = _runState == RUNNING ? (millis() - _runClockMs) / 1000.0f : 0.0f;
  String json;
  json.reserve(360);
  json += "{\"state\":\""; json += runName;
  json += "\",\"text\":\""; json += statusText();
  json += "\",\"nav\":\""; json += navText;
  json += "\",\"phase\":\""; json += kPhaseNames[_phase];
  json += "\",\"paused\":"; json += isPaused() ? "true" : "false";
  json += ",\"safety\":"; json += _safetyMonitor.isEnabled() ? "true" : "false";
  json += ",\"poles\":"; json += _lineTracker.poleCount();
  json += ",\"polesTarget\":"; json += POLES_BEFORE_TURN;
  json += ",\"elapsedS\":"; json += String(elapsedS, 1);
  json += ",\"headingRef\":"; json += String(_snapshot.headingRefDeg, 1);
  json += ",\"crossTrack\":"; json += String(_snapshot.crossTrackCm, 1);
  json += ",\"front\":"; json += _snapshot.frontCm;
  json += ",\"wall\":"; json += _snapshot.wall.isFound ? (int)_snapshot.wall.distanceCm : 0;
  json += ",\"wallDeg\":"; json += String(_snapshot.wall.isFound ? _snapshot.wall.angleDeg : 0.0f, 1);
  json += ",\"wallSteer\":"; json += _lineTracker.isWallTracking() ? "true" : "false";
  json += ",\"baseline\":"; json += _safetyMonitor.baselineCm();
  json += '}';
  return json;
}

void Navigator::_note(const char* text) {
  if (_eventText[0]) strlcat(_eventText, ";", sizeof(_eventText));
  strlcat(_eventText, text, sizeof(_eventText));
}

// Events of Navigator, LineTracker and SafetyMonitor since the last call (merged in step()).
void Navigator::takeEvent(char* output, size_t capacity) {
  strlcpy(output, _eventText, capacity);
  _eventText[0] = 0;
}
