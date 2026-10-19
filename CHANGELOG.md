# Changelog

Notable changes to the box firmware, CAM firmware and tools. Newest on top.
`drive-YYYY-MM-DD` tags mark versions that were driven on the real rover.

## [Unreleased]

### Changed
- `firmware/box/` modules moved into `src/core/`, `src/sensors/`, `src/comms/`, `src/display/`;
  `box.ino` includes them as `src/<folder>/<File>.h`. No code changes.
- Removed `data/`, `reference/` and `submission/` from the repo. Drive logs and the test log
  live in `L4_Rover/04_SAFETY_AND_TESTING/Test_Results/`, the source zip is written to
  `L4_Rover/06_REPORTS_AND_DEMO/Final_Submission/`, the reference pointers moved to
  `docs/project_folder.md`. `fetch_log.py` gained `--out`.

### Planned
- Timeline logger on the box (LittleFS CSV + `/log` download), see docs/logging.md
- ESP32-CAM sketch with a working stream (`firmware/cam/`)
- First autonomous behaviour: stop in front of an obstacle using the front ultrasonic

## [starter-v1.0] – 2026-10-19

### Added
- Repository structure: firmware/, tools/, docs/, data/, reference/, submission/
- `firmware/box/`: CAS starter program v1.0 (2026-10-15), `cas_box.ino` renamed to `box.ino`,
  kit number and tunables moved into `Config.h`; sensor and link modules unchanged
- `firmware/cam/`: placeholder sketch and flashing notes for the ESP32-CAM
- Documentation: architecture, pinout, rover interface, logging design, test log, ADR-0001
- `tools/plot_run.py` for the timeline CSV, `tools/make_submission.ps1` for the CAS zip
