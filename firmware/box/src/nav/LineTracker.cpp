#include "LineTracker.h"
#include <math.h>
#include "NavConfig.h"
#include "NavMath.h"

void LineTracker::reset(float targetYawDeg) {
  _targetYawDeg = _headingRefDeg = targetYawDeg;
  _crossTrackCm = 0;
  _odometryCm = 0;
  _selfDriftLeftCm = _selfDriftAtLastPoleCm = 0;
  _lastDriveLeft = _lastDriveRight = 0;
  _poleCount = 0;
  _isPoleTracking = false;
  _poleAbeamCount = 0;
  _referencePoleYCm = _lastPoleYCm = NAN;
  _lastPoleOdometryCm = 0;
  _lastCountedPoleYCm = 0;
  _isWallLocked = _isWallTracking = false;
  _wallSeenCount = _wallMissedCount = 0;
  _eventText[0] = 0;
}

void LineTracker::startLeg(float targetYawDeg) {
  _isWallLocked = _isWallTracking = false;  // a wall after the turn is a new one
  _wallSeenCount = _wallMissedCount = 0;
  _targetYawDeg = _headingRefDeg = targetYawDeg;
  _crossTrackCm = 0;
}

void LineTracker::addOdometry(float stepS) {
  _odometryCm += CM_PER_S_PER_CMD * (_lastDriveLeft + _lastDriveRight) * 0.5f * stepS;  // turning in place adds ~0
}

// Virtual line: sideways drift from the start line out of speed and heading (+ = left of the line),
// then aim a few degrees back toward the line.
float LineTracker::updateHeadingRefDeg(float yawDeg, float stepS) {
  const float lineErrDeg = angleDiff(yawDeg, _targetYawDeg);
  const float speedCmS = CM_PER_S_PER_CMD * (_lastDriveLeft + _lastDriveRight) * 0.5f;
  const float sideStepCm = speedCmS * stepS * sinf(lineErrDeg * DEG_TO_RAD);
  _crossTrackCm += sideStepCm;
  _selfDriftLeftCm += sideStepCm;  // same, but never overwritten by wall / poles
  _headingRefDeg = _targetYawDeg - constrain(_crossTrackCm * LINE_GAIN_DEG_PER_CM, -LINE_MAX_DEG, LINE_MAX_DEG);
  return _headingRefDeg;
}

// Follows the next pole from ahead until it is directly to our right (x <= 0, 2 scans) and counts it
// only if it moved past us like a fixed object: its own travel must match the distance we drove.
// A person walking alongside (barely moves relative to us) or a jump between objects is rejected.
void LineTracker::updatePoles(const PoleSeen& pole) {
  if (!pole.isFound) { _poleAbeamCount = 0; return; }
  if (pole.xCm > 10) {
    if (!_isPoleTracking || pole.xCm > _lastTrackXCm + 50) {  // new candidate ahead
      _isPoleTracking = true;
      _trackStartXCm = pole.xCm;
      _trackStartOdometryCm = _odometryCm;
    }
    _lastTrackXCm = pole.xCm;
    _poleAbeamCount = 0;
    return;
  }
  if (!_isPoleTracking) return;
  _poleAbeamCount = (pole.xCm <= 0) ? _poleAbeamCount + 1 : 0;
  if (_poleAbeamCount < 2) return;

  _isPoleTracking = false;
  _poleAbeamCount = 0;
  const float roverTravelCm = _odometryCm - _trackStartOdometryCm, poleTravelCm = _trackStartXCm - pole.xCm;
  if (roverTravelCm < 30 || poleTravelCm < 0.5f * roverTravelCm || poleTravelCm > 1.6f * roverTravelCm) {
    Serial.printf("[nav] ignored a moving object (it moved %.0f cm, we drove %.0f cm)\n", poleTravelCm, roverTravelCm);
    _note("moving_object");
    return;
  }
  _poleCount++;
  Serial.printf("[nav] pole %d passed (%.0f cm to the right)\n", _poleCount, pole.yCm);
  char eventText[16];
  snprintf(eventText, sizeof(eventText), "pole_%d", _poleCount);
  _note(eventText);
  _onPolePassed(pole.yCm);
}

// Pole passed: its side distance is a ruler for our own position. If it grows from pole to pole we
// drift left; the virtual line takes the measured offset and the line heading is turned by the
// measured drift angle.
void LineTracker::_onPolePassed(float poleYCm) {
  _lastCountedPoleYCm = poleYCm;
  if (!POLE_LINE_CORRECTION) return;
  if (_isWallTracking) {  // the wall steers right now: only remember the pole as new reference
    _referencePoleYCm = _lastPoleYCm = poleYCm;
    _lastPoleOdometryCm = _odometryCm;
    _selfDriftAtLastPoleCm = _selfDriftLeftCm;
    return;
  }
  if (isnan(_referencePoleYCm)) {
    _referencePoleYCm = poleYCm;  // first pole defines "on the line"
  } else if (fabsf(poleYCm - _referencePoleYCm) > POLE_JUMP_CM) {
    // Much nearer / farther than the reference: one of them was not a real pole of the row. Never
    // steer on it, start a new reference here instead (26.10 run 12: 45 cm -> 9 deg swerve).
    Serial.printf("[nav] pole side distance jumped %.0f cm -> new reference, no correction\n", poleYCm - _referencePoleYCm);
    _note("pole_jump");
    _referencePoleYCm = poleYCm;
  } else {
    _crossTrackCm = constrain(poleYCm - _referencePoleYCm, -CROSS_TRACK_MAX_CM, CROSS_TRACK_MAX_CM);  // + = left of the line
    if (!isnan(_lastPoleYCm) && fabsf(poleYCm - _lastPoleYCm) <= POLE_JUMP_CM && _odometryCm - _lastPoleOdometryCm > 100) {
      // unseen drift = how far we really moved left minus what our own heading explains
      const float driftDeg =
          atan2f((poleYCm - _lastPoleYCm) - (_selfDriftLeftCm - _selfDriftAtLastPoleCm), _odometryCm - _lastPoleOdometryCm) * RAD_TO_DEG;  // + = left
      _targetYawDeg = angleDiff(_targetYawDeg - constrain(driftDeg, -DRIFT_MAX_DEG, DRIFT_MAX_DEG), 0.0f);
      Serial.printf("[nav] pole drift %.1f cm, %.2f deg -> line turned by that\n", poleYCm - _referencePoleYCm, driftDeg);
    }
  }
  _lastPoleYCm = poleYCm;
  _lastPoleOdometryCm = _odometryCm;
  _selfDriftAtLastPoleCm = _selfDriftLeftCm;
}

// Uses the wall DISTANCE to hold our line (the angle is not trusted, see WALL_MAX_ANGLE_DEG):
//  - offset: the distance it had when first seen means "on our line"; closer = we are left of it
//  - heading: every WALL_SLOPE_CM of driving, compare how much the distance really changed with how
//    much our own heading should have changed it; the difference is drift the IMU cannot see
//    (start misalignment, gyro bias) and turns the line heading by that angle.
// A sudden distance jump (recess, door, another object) gives a new reference, no steering on it.
void LineTracker::updateWall(const WallSeen& wall) {
  if (!WALL_ALIGNMENT) return;
  if (!wall.isFound || fabsf(wall.angleDeg) > WALL_MAX_ANGLE_DEG) {
    _wallSeenCount = 0;
    _isWallTracking = false;
    if (++_wallMissedCount > 20) _isWallLocked = false;  // gone for 2 s: forget the reference
    return;
  }
  _wallMissedCount = 0;
  if (++_wallSeenCount < 5) return;  // seen steadily for 0.5 s
  if (!_isWallLocked || fabsf((_wallReferenceCm - wall.distanceCm) - _crossTrackCm) > WALL_JUMP_CM) {
    _wallReferenceCm = wall.distanceCm + _crossTrackCm;  // keep our current line: left of it -> the wall is closer
    _isWallLocked = true;
    _wallFilteredCm = _slopeStartWallCm = wall.distanceCm;
    _slopeStartOdometryCm = _odometryCm;
    _slopeStartSelfDriftCm = _selfDriftLeftCm;
    Serial.printf("[nav] wall locked at %.0f cm (line at %.0f cm)\n", wall.distanceCm, _wallReferenceCm);
    _note("wall_lock");
  }
  _isWallTracking = true;
  _wallFilteredCm += 0.3f * (wall.distanceCm - _wallFilteredCm);  // smooth single-scan noise
  _crossTrackCm += WALL_DIST_GAIN * ((_wallReferenceCm - _wallFilteredCm) - _crossTrackCm);  // + = left of the line
  const float drivenCm = _odometryCm - _slopeStartOdometryCm;
  if (drivenCm >= WALL_SLOPE_CM) {
    const float movedLeftCm = _slopeStartWallCm - _wallFilteredCm;  // wall came closer = we moved left
    const float expectedLeftCm = _selfDriftLeftCm - _slopeStartSelfDriftCm;  // what our own heading explains
    const float driftDeg = atan2f(movedLeftCm - expectedLeftCm, drivenCm) * RAD_TO_DEG;  // + = unseen drift left
    _targetYawDeg = angleDiff(_targetYawDeg - constrain(driftDeg, -DRIFT_MAX_DEG, DRIFT_MAX_DEG), 0.0f);
    Serial.printf("[nav] wall drift %.2f deg -> line heading corrected\n", driftDeg);
    _slopeStartOdometryCm = _odometryCm; _slopeStartWallCm = _wallFilteredCm; _slopeStartSelfDriftCm = _selfDriftLeftCm;
  }
}

void LineTracker::_note(const char* text) {
  if (_eventText[0]) strlcat(_eventText, ";", sizeof(_eventText));
  strlcat(_eventText, text, sizeof(_eventText));
}

const char* LineTracker::takeEvent() {
  strlcpy(_eventOutput, _eventText, sizeof(_eventOutput));
  _eventText[0] = 0;
  return _eventOutput;
}
