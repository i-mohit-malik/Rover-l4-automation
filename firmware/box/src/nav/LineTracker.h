#pragma once
// LineTracker - where are we relative to the straight line we want to drive?
//
//  - odometry: distance driven from the drive commands (CM_PER_S_PER_CMD)
//  - virtual line: sideways offset from the start line out of speed and heading error
//    (crossTrack, + = left of the line); the heading reference aims back toward the line
//  - poles on the right: counted when they pass us like a fixed object; their side distance is a
//    ruler for our own drift and turns the line heading by the measured drift angle
//  - wall on the left: while it is visible, its DISTANCE holds the line (the angle is not trusted)
//
// Owned by Navigator, called once per control step (loop(), core 1). Values: see NavConfig.h.
#include <Arduino.h>
#include "Perception.h"

class LineTracker {
public:
  void reset(float targetYawDeg);      // new run: line along targetYawDeg, all counters 0
  void startLeg(float targetYawDeg);   // new straight line after a turn (counters kept)

  void addOdometry(float stepS);                   // distance from the last command over stepS seconds
  void updatePoles(const PoleSeen& pole);          // pole counting + drift correction
  void updateWall(const WallSeen& wall);           // wall alignment
  float updateHeadingRefDeg(float yawDeg, float stepS);     // integrates the virtual line, returns the heading to steer to
  void setCommand(float driveLeft, float driveRight) { _lastDriveLeft = driveLeft; _lastDriveRight = driveRight; }  // what the rover does during the next step

  float targetYawDeg() const { return _targetYawDeg; }
  float lastHeadingRefDeg() const { return _headingRefDeg; }
  float crossTrackCm() const { return _crossTrackCm; }
  float odometryCm() const { return _odometryCm; }
  int poleCount() const { return _poleCount; }
  float lastPoleYCm() const { return _lastCountedPoleYCm; }  // side distance of the last counted pole
  bool isWallTracking() const { return _isWallTracking; }

  // Event of the last calls for the log note column ("pole_2", "wall_lock", ...); cleared by reading.
  const char* takeEvent();

private:
  void _onPolePassed(float poleYCm);
  void _note(const char* text);

  float _targetYawDeg = 0, _headingRefDeg = 0;
  float _crossTrackCm = 0;         // + = left of the line
  float _odometryCm = 0;                // distance driven this run
  float _selfDriftLeftCm = 0;             // sideways motion our own heading caused, + = left (never overwritten)
  float _lastDriveLeft = 0, _lastDriveRight = 0;    // last command

  // poles
  int _poleCount = 0;
  bool _isPoleTracking = false;      // a candidate is followed from ahead to beside us
  float _trackStartXCm = 0, _trackStartOdometryCm = 0, _lastTrackXCm = 0;
  int _poleAbeamCount = 0;
  float _referencePoleYCm = NAN, _lastPoleYCm = NAN, _lastPoleOdometryCm = 0, _lastCountedPoleYCm = 0, _selfDriftAtLastPoleCm = 0;

  // wall
  float _wallFilteredCm = 0, _slopeStartOdometryCm = 0, _slopeStartWallCm = 0, _slopeStartSelfDriftCm = 0;
  bool _isWallLocked = false;        // a wall reference distance is set
  bool _isWallTracking = false;      // the wall is seen right now and steers us
  float _wallReferenceCm = 0;            // wall distance that means "on our line"
  int _wallSeenCount = 0, _wallMissedCount = 0; // debounce (scans)

  char _eventText[24] = "";
  char _eventOutput[24] = "";
};
