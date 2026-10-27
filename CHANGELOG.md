# Changelog

Notable changes to the box firmware, CAM firmware and tools. Newest on top.
`drive-YYYY-MM-DD` tags mark versions that were driven on the real rover.

## [Unreleased]

### Added
- Autonomous run from the CampusLogger sketch (26.10.2026, "passed open sea"), split into
  `firmware/box/src/nav/`: `Perception` (pole / wall / lane detection from one LiDAR scan),
  `LineTracker` (odometry, virtual line, pole counting with drift correction, left-wall
  alignment), `SafetyMonitor` (obstacle pause, cliff check), `Navigator` (run state, phases
  LEG1 → WAIT → TURN/SETTLE → LEG2, PID heading hold, 90° turn), `NavConfig.h` (all tuned values
  unchanged), `NavMath`. Description: `docs/autonomous_run.md`.
- Page: "Autonomous run" card (start, stop, safety stops, save IMU calibration, poles, wall,
  cross-track), link-lost banner, firmware version. Routes `/run/start`, `/run/stop`,
  `/run/safety`, `/imu/save` (queued, executed in `loop()`), `/update` (firmware over Wi-Fi,
  refused during a run).
- Run stops: STOP RUN and the pad's stop key, drive-pad override, page silent for 3 s
  (`LINK_TIMEOUT_MS`), cliff (off by default), time limit (off by default).
- `ImuManager`: compass-off mode (IMUPLUS, `USE_COMPASS = false` in `Config.h`), turn rate,
  50 Hz read, calibration saved to / loaded from flash. `UsManager::suspend()`.
- `Sensors`: live accessors for the control step (`usNowCm`, `yawRateDps`, `lidarCm(deg)`,
  IMU calibration); `lidarMinCm` now uses the 1° map.
- Timeline logger on the box: LittleFS CSV, `/log` list / download / clear / mark.

### Changed
- Autonomous run timing brought back to the CampusLogger sketch (all values and control lines were
  already identical): no flash access during a run (log rows kept in RAM, written to a new
  `run_<n>.csv` after the run; `/log` downloads wait), no `[sys]` print during a run, pole / wall
  detection only in the phases where the sketch ran it, IMU started in `setup()`.
- Box firmware: all variables, members and parameters renamed to descriptive names with unit
  suffixes (`age` → `padCommandAgeMs`, `now` → `nowMs`, `l`/`r` → `driveLeft`/`driveRight`, …),
  booleans with `is`/`has`. Naming rules, every variable and the old → new table:
  `firmware/box/VARIABLES.md`. No behaviour change; same binary size. Web page, JSON keys and
  CSV columns unchanged.
- Log format **v2**: the 24 v1 columns unchanged, 20 run columns appended (heading reference,
  cross-track, lane check, poles, pole and wall detections, ...). One file per boot and one per
  START RUN; 1 row/s while standing without a run. v1 columns now carry the live values the
  decision used instead of the 4 Hz page snapshot.
- `tools/plot_run.py` reads v2 (heading reference, cross-track / wall / poles panel) and old
  CampusLogger CSVs. `tools/fetch_log.py` picks the newest file by number (`run_10` after `run_9`).
- `firmware/box/` modules moved into `src/core/`, `src/sensors/`, `src/comms/`, `src/display/`;
  `box.ino` includes them as `src/<folder>/<File>.h`. No code changes.
- Removed `data/`, `reference/` and `submission/` from the repo. Drive logs and the test log
  live in `L4_Rover/04_SAFETY_AND_TESTING/Test_Results/`, the source zip is written to
  `L4_Rover/06_REPORTS_AND_DEMO/Final_Submission/`, the reference pointers moved to
  `docs/project_folder.md`. `fetch_log.py` gained `--out`.

### Planned
- ESP32-CAM sketch with a working stream (`firmware/cam/`)
- Drive test of the split firmware on the rover, then tag `drive-YYYY-MM-DD`

## [starter-v1.0] – 2026-10-19

### Added
- Repository structure: firmware/, tools/, docs/, data/, reference/, submission/
- `firmware/box/`: CAS starter program v1.0 (2026-10-15), `cas_box.ino` renamed to `box.ino`,
  kit number and tunables moved into `Config.h`; sensor and link modules unchanged
- `firmware/cam/`: placeholder sketch and flashing notes for the ESP32-CAM
- Documentation: architecture, pinout, rover interface, logging design, test log, ADR-0001
- `tools/plot_run.py` for the timeline CSV, `tools/make_submission.ps1` for the CAS zip
