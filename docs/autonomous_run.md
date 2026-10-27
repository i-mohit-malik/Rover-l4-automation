# Autonomous run (pole row, turn right)

The first autonomous behaviour of the box. It was developed as one sketch, *CampusLogger*
(26.10.2026, "passed open sea"), and is split here into modules under `firmware/box/src/nav/`
so that perception, line keeping, safety and control can be worked on separately.
The behaviour and every tuned value are unchanged from the sketch that drove successfully on
26.10.2026; only the structure, the page and the log changed.

## What it does

1. **START RUN** (page): the current IMU yaw becomes the line heading. The 45° ultrasonic's
   median of the last 1.5 s idle becomes the ground baseline of the cliff check.
2. **LEG1**: drive straight at `CRUISE_SPEED`, PID heading hold toward a virtual line, count
   the poles that pass on the right.
3. **WAIT**: after `POLES_BEFORE_TURN` poles keep driving straight for `TURN_DELAY_MS`.
4. **TURN / SETTLE**: full-speed pulse in place toward line heading − 90°, stop early by
   turn rate × `TURN_LEAD_S`, stand `TURN_SETTLE_MS`, measure. Up to `TURN_MAX_PULSES` pulses
   until the error is below `TURN_TOLERANCE_DEG`.
5. **LEG2**: straight on along the new heading until STOP.

Driving straight uses three sources:

| Source | What it corrects | Where |
|---|---|---|
| IMU yaw (gyro only, `USE_COMPASS = false`) + turn rate | heading, damped | `Navigator::_straightStep` |
| virtual line: speed × sin(heading error) integrated | sideways drift from the start line | `LineTracker::updateHeadingRefDeg` |
| poles on the right: side distance pole to pole | drift the IMU cannot see; turns the line heading by up to 2° per pole | `LineTracker::updatePoles` |
| straight wall on the left: its distance | same, while a wall is visible (the wall *angle* is never trusted) | `LineTracker::updateWall` |

A pole counts only if it passes us like a fixed object: its own travel from first sight
to abeam must be 0.5–1.6 × the distance we drove (rejects people walking alongside).
Shape check: the arc of points must fit a circle of 21–53 cm diameter with < 2.5 cm error.

## Stops and pauses

| Event | Effect | Where |
|---|---|---|
| STOP RUN button, or the drive pad's ■ key | run ends | page → `WebHMI` command → `box.ino` |
| any drive-pad key pressed after the start | run ends, the pad drives (override) | `box.ino` |
| page has not polled `/values` for `LINK_TIMEOUT_MS` (3 s) | run ends | `Navigator::step` |
| obstacle in our own lane (LiDAR corridor 26 cm wide, 150 cm ahead) < 45 cm, 3 steps | pause, resume when > 60 cm for 3 steps; run clock paused | `SafetyMonitor` |
| front ultrasonic < 25 cm | counts as obstacle (higher readings are the ground) | `SafetyMonitor::frontDistance` |
| 45° ultrasonic: step down or 1.4 s no echo | run ends; **off** (`CLIFF_CHECK_ENABLED`), unreliable on campus pavement | `SafetyMonitor` |
| `MAX_RUN_TIME_MS` (0 = off) | run ends | `Navigator::step` |
| firmware upload starts | run ends, rover stops | `box.ino` |

The manual-driving stops (key release, 0.4 s pad timeout), RoverLink's 0.5 s deadman and the
rover's 3 s heartbeat stay active in every mode. "Safety stops" on the page switches only the
obstacle pause and the cliff check.

## Modules

| File | Job | Typical change |
|---|---|---|
| `src/nav/NavConfig.h` | every tuned value with its origin | field tuning |
| `src/nav/Perception.*` | pole, wall, lane detection from one LiDAR scan; pure functions | new feature detectors (markers, gaps) |
| `src/nav/LineTracker.*` | odometry, virtual line, pole counting, drift correction, wall alignment | localisation |
| `src/nav/SafetyMonitor.*` | obstacle pause, cliff check, safety switch | safety behaviour |
| `src/nav/Navigator.*` | run state, phases, PID heading hold, turn; owns LineTracker and SafetyMonitor | new phases or checkpoints |
| `src/nav/NavMath.*` | angle difference, median, circle fit | – |
| `box.ino` | wiring: commands, control step every `ROVER_PERIOD_MS`, pad override, log row | – |

Perception and the safety filters run in **every** control step, also while you drive by
hand. The page and the log then show what a run would see, which is how the pole and wall
detectors can be checked on the real track without driving autonomously.

## Page

The "Autonomous run" card shows the run state, poles passed, run time, turn rate, heading
reference and cross-track offset, the lane check and the left wall. It has START RUN /
STOP RUN, the safety switch and "Save IMU calibration". Saving needs all four calibration
levels at 3 and only matters with `USE_COMPASS = true`. A red banner appears when the box
has not answered for 1 s.
"firmware … · update over Wi-Fi" opens `/update` for a new `.bin` without USB.

## Differences to the CampusLogger sketch

Checked line by line on 27.10.2026: every tuned value (`NavConfig.h`, `Config.h`) and every line
of the control step (heading PID, virtual line, pole counting and drift, wall alignment, turn,
settle, obstacle pause, cliff check, link watchdog) is the same as in the sketch. What runs around
the control step during a run was brought back to the sketch as well:

- no flash access during a run (log in RAM, written after the run), as the sketch did;
- no periodic `[sys]` print during a run (the sketch printed only events);
- pole detection only in LEG1 and wall detection only in the straight phases, as in the sketch;
- the IMU starts in `setup()`, as in the sketch (before: 5 s after boot).

What still differs (no influence on the control step that we know of):

- Async web server and the existing page instead of a second page; the drive pad stays.
- The run log is written to flash after the run (`/log`, format v2, one file per run) instead of
  being downloaded from RAM; the RAM holds fewer seconds than the sketch's 170 s when the heap is
  short (the start message says how many). The CampusLogger columns are in the v2 columns
  (`docs/logging.md`), and `tools/plot_run.py` also reads the old `campus_run.csv` files.
- LiDAR columns `lid_*` are the nearest return within ±10° (CampusLogger: one single degree).
- New: drive-pad override and the pad's ■ key also end a run.
- The firmware upload restarts the box from `loop()` after the upload instead of inside the
  web handler, and restarts after 15 s if an upload stalls.
