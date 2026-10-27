#pragma once
// SafetyMonitor - stops that belong to the autonomous run (values in NavConfig.h):
//  - obstacle pause: something in our own lane closer than FRONT_STOP_CM (LiDAR corridor, front
//    ultrasonic only below US_CLOSE_CM) pauses the run; it resumes once the lane is clear again.
//    Debounced over FRONT_DEBOUNCE control steps both ways.
//  - cliff: the 45 deg ultrasonic reads much deeper than the ground baseline (step down) or has no
//    echo for VOID_TIMEOUT_CYCLES. The baseline is the median of the readings while idle.
//    Off by default (CLIFF_CHECK_ENABLED), still computed and logged.
// Both can be switched off from the page ("Safety stops"). The link watchdog and the drive-pad /
// RoverLink / rover stops are not here: they work in every mode (box.ino, RoverLink).
#include <Arduino.h>
#include "NavConfig.h"

class SafetyMonitor {
public:
  // Every control step, running or not: feeds the 45 deg ultrasonic into the median filter and,
  // while idle, into the ground baseline.
  void observe(int cliffRawCm, bool isRunning);

  // New run: baseline from the idle readings, counters reset.
  void beginRun();

  // Distance the obstacle check uses: LiDAR lane check, front ultrasonic only for very close things.
  // 0 = lane clear.
  static int frontDistance(uint16_t corridorCm, int usFrontCm);

  // One running step: obstacle pause state and cliff check; true = cliff, stop the run.
  // straight = driving along a line (no checks while turning in place).
  bool update(int frontCm, int cliffRawCm, bool isStraight);

  void toggle();                                  // safety stops on / off (page button)
  void releasePause() { _isPaused = false; }        // e.g. while turning
  bool isEnabled() const { return _isEnabled; }
  bool isPaused() const { return _isPaused; }
  int baselineCm() const { return _baselineCm; }
  int cliffFilteredCm() const { return _cliffFilteredCm; }

  const char* takeEvent();  // "obstacle", "clear", "cliff" for the log note column

private:
  void _note(const char* text);

  static const int kBaselineSamples = 15;   // 1.5 s of idle readings
  int _baselineSamplesCm[kBaselineSamples];
  int _baselineSampleCount = 0, _baselineSampleIndex = 0;
  int _baselineCm = BASELINE_DEFAULT_CM;
  int _cliffHistoryCm[3] = {BASELINE_DEFAULT_CM, BASELINE_DEFAULT_CM, BASELINE_DEFAULT_CM};
  int _cliffFilteredCm = BASELINE_DEFAULT_CM;
  int _obstacleCount = 0, _clearCount = 0;
  int _stepDropCount = 0, _noEchoCount = 0;
  bool _isPaused = false;
  bool _isEnabled = true;
  char _eventText[24] = "";
  char _eventOutput[24] = "";
};
