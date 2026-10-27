#pragma once
// Navigator - the autonomous run ("brain"): drive straight along the pole row, count the poles on
// the right, turn right by 90 deg after the last one, drive straight again. Ported from the
// CampusLogger sketch of 26.10.2026; behaviour and tuning in docs/autonomous_run.md / NavConfig.h.
//
//   phases:  LEG1 (straight, counting poles) -> WAIT (TURN_DELAY_MS straight) -> TURN (full-speed
//            pulse in place) <-> SETTLE (stand, measure error) -> LEG2 (straight on the new heading)
//   stops:   STOP button, manual override (drive pad), phone link lost (LINK_TIMEOUT_MS),
//            cliff, run time limit. An obstacle in the lane only pauses the run.
//
// step() runs once per control step (ROVER_PERIOD_MS) in loop(), core 1, whether a run is active
// or not. During a run it does exactly the work of the CampusLogger control step (pole detection
// only in LEG1, wall detection only in the straight phases). Outside a run it detects poles and the
// wall every step for the page and the log, returns l = r = 0, and box.ino uses the drive pad.
#include <Arduino.h>
#include "../sensors/Sensors.h"
#include "Perception.h"
#include "LineTracker.h"
#include "SafetyMonitor.h"

class Navigator {
public:
  enum RunState : uint8_t { IDLE, RUNNING, STOPPED_MANUAL, STOPPED_OVERRIDE, STOPPED_LINK, STOPPED_CLIFF, STOPPED_TIMEOUT };
  enum Phase : uint8_t { NAV_LEG1, NAV_WAIT, NAV_TURN, NAV_SETTLE, NAV_LEG2 };

  // What the last step saw and decided (for the log row and the page).
  struct Snapshot {
    float yawRateDps = 0;           // deg/s, + = turning left
    float headingRefDeg = 0;        // heading the controller steered to
    float crossTrackCm = 0;        // cm, + = left of the line
    uint16_t corridorCm = 0;     // LiDAR lane check, 0 = clear
    int frontCm = 0;             // value the obstacle check used, 0 = clear
    PoleSeen pole = {false, 0, 0, 0, 0};   // pole seen in this scan
    WallSeen wall = {false, 0, 0, 0};      // left wall seen in this scan
  };

  void start(const Sensors& sensors, uint32_t nowMs);   // starts a run along the current heading
  void stop(RunState reason);                      // ends a running run (no effect otherwise)
  void toggleSafety() { _safetyMonitor.toggle(); }

  // One control step. pageAgeMs: time since the phone page last polled (link watchdog).
  // l, r: drive command for this step (0 outside a run).
  void step(const Sensors& sensors, uint32_t nowMs, float stepS, uint32_t pageAgeMs, float& driveLeft, float& driveRight);

  bool isRunning() const { return _runState == RUNNING; }
  RunState runState() const { return _runState; }
  Phase phase() const { return _phase; }
  uint32_t startedAtMs() const { return _startedAtMs; }  // real start time of the current run
  bool isSafetyEnabled() const { return _safetyMonitor.isEnabled(); }
  bool isPaused() const { return _runState == RUNNING && _safetyMonitor.isPaused(); }
  bool isWallSteering() const { return _lineTracker.isWallTracking(); }
  int poleCount() const { return _lineTracker.poleCount(); }
  float lastPoleYCm() const { return _lineTracker.lastPoleYCm(); }
  int cliffBaselineCm() const { return _safetyMonitor.baselineCm(); }
  const Snapshot& lastSnapshot() const { return _snapshot; }

  const char* stateName() const;  // log "state" column: LEG1..LEG2 in a run, otherwise MANUAL
  String statusText() const;      // one line for the page, e.g. "PAUSED (OBSTACLE AHEAD)"
  String toJson() const;          // run card on the page

  // Events since the last call ("run_start;pole_1", "stop_link", ...) for the log note column.
  bool hasEvent() const { return _eventText[0] != 0; }
  void takeEvent(char* output, size_t capacity);

private:
  void _note(const char* text);
  void _straightStep(const Sensors& sensors, uint32_t nowMs, float stepS, float yawDeg, float yawRateDps,
                     float& driveLeft, float& driveRight);

  RunState _runState = IDLE;
  Phase _phase = NAV_LEG1;
  uint32_t _startedAtMs = 0;    // real start
  uint32_t _runClockMs = 0;     // start, shifted forward by obstacle pauses (run time limit, elapsed)
  uint32_t _phaseStartMs = 0;
  float _turnTargetYawDeg = 0;
  int _turnPulseCount = 0;
  float _headingErrorIntegral = 0;          // heading error integral (deg*s)

  LineTracker _lineTracker;
  SafetyMonitor _safetyMonitor;
  Snapshot _snapshot;
  char _eventText[48] = "";
};
