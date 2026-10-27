# Timeline logging

## Why

The phone page polls `/values` every 250 ms over Wi-Fi: good enough to watch, not good enough to
reason about timing. The log is written **on the box, in the loop that makes the decision**, with
the box millisecond clock. It is read **after** the drive.

## Where the data lives

| Stage | Place | Format |
|---|---|---|
| During the drive | box flash, LittleFS partition (default partition scheme: ~1.4 MB) | one CSV file per boot and one per autonomous run, `/log/run_<n>.csv` |
| During an autonomous run | box RAM (as in the CampusLogger sketch) | written to a new `run_<n>.csv` when the run ends |
| After the drive | laptop or phone via `http://192.168.(10+N).1/log` (download) | same CSV |
| Archived | `L4_Rover/04_SAFETY_AND_TESTING/Test_Results/YYYY-MM-DD_HHMM_<short-note>.csv` (outside this repo) | same CSV, renamed |
| Analysed | `tools/plot_run.py <file>.csv` | PNG next to the CSV |

Serial logging stays available when the box is tethered to the laptop, but the field runs are
untethered, so flash is the primary store.

## Row format (v2)

One row per control step (every 100 ms = the RoverLink period) while a run is active or the
rover moves (and 2 s after it stopped); standing without a run, one row per
`LOG_IDLE_PERIOD_MS` (1 s). A row with a note (event or mark) is always written at once.
First line `# log v2`, then the header. Fixed column order so that the Python side never guesses.
v2 = the 24 v1 columns unchanged, then 20 columns of the autonomous run (`docs/autonomous_run.md`).
They are filled also while driving by hand, so the detectors can be checked without a run.

```
t_ms,state,l,r,us_f,us_l,us_r,us_b,us_fa,yaw,pitch,roll,imu_ok,lid_ok,lid_f,lid_l,lid_r,lid_b,gps_fix,lat,lon,volt,loop_us,note,
yaw_rate,heading_ref,cross_track,imu_cal,front_cm,lid_corr,lid_fr,lid_fl,paused,safety,poles,pole_x,pole_y,pole_d,pole_fit,last_pole_y,wall_cm,wall_deg,wall_steer,cliff_base
```
(one line in the file)

| Column | Unit | Source |
|---|---|---|
| `t_ms` | ms since boot | `millis()` |
| `state` | string | who drives: `MANUAL` (drive pad / standing) or the run phase `LEG1`, `WAIT`, `TURN`, `SETTLE`, `LEG2` |
| `l`, `r` | −0.5…0.5 | what was sent to `roverLink.setDrive()` this tick |
| `us_f … us_fa` | cm | `Sensors::usNowCm(0..4)` (latest measurement, not the 4 Hz page snapshot); 300 = nothing |
| `yaw`, `pitch`, `roll` | deg | ISO 8855, latest IMU read (every 20 ms), `imu_ok` 0/1 |
| `lid_f`, `lid_l`, `lid_r`, `lid_b` | cm | nearest LiDAR return within ±10° of 0/270/90/180° (1° bins); 0 = none |
| `gps_fix`, `lat`, `lon` | 0/1, deg | GPS |
| `volt` | V | rover battery from `roverLink.voltage()` |
| `loop_us` | µs | duration of the previous loop pass |
| `note` | string | events of the run, `;`-separated: `run_start`, `pole_<n>`, `moving_object`, `pole_jump`, `wall_lock`, `turn`, `turn_pulse`, `leg2`, `obstacle`, `clear`, `cliff`, `safety_on/off`, `stop_manual`, `stop_override`, `stop_link`, `stop_cliff`, `stop_timeout`; plus the `/log?mark=` text. Empty otherwise, max 23 chars |
| **v2** | | |
| `yaw_rate` | deg/s | gyro z, + = turning left |
| `heading_ref` | deg | heading the controller steered to (line heading − line correction) |
| `cross_track` | cm | estimated offset from the virtual line, + = left |
| `imu_cal` | 4 digits | calibration sys/gyro/accel/mag, `3333` = full |
| `front_cm` | cm | value the obstacle check used (lane check or close front ultrasonic), 0 = clear |
| `lid_corr` | cm | LiDAR lane check: nearest point in the 26 cm wide lane up to 150 cm ahead, 0 = clear |
| `lid_fr`, `lid_fl` | cm | nearest LiDAR return within ±10° of 45° / 315°, 0 = none |
| `paused`, `safety` | 0/1 | obstacle pause active; safety stops enabled |
| `poles` | count | poles counted in this run |
| `pole_x`, `pole_y`, `pole_d`, `pole_fit` | cm | pole detected in this scan: centre (x forward, y right), fitted diameter, fit error; empty = none |
| `last_pole_y` | cm | side distance of the last counted pole |
| `wall_cm`, `wall_deg` | cm, deg | left wall in this scan: distance, our angle to it (+ = pointing left of it); empty = none |
| `wall_steer` | 0/1 | the wall steered in this step |
| `cliff_base` | cm | ground baseline of the cliff check |

Autonomous run: nothing touches the flash while the rover drives on its own. Every flash erase,
write or read freezes both CPU cores for up to ~50 ms, which shifts the control step and the rover
commands; the CampusLogger sketch therefore logged to RAM, and so does the Logger now. START RUN
takes as many 100-row blocks from the heap as fit above 80 kB free (at most 1700 rows = 170 s; the
start message on the page says how many seconds). When the run ends (stop button, override, link
lost, …) the rows are written to a new `run_<n>.csv`, followed by the rows after the run. A full
buffer drops the rest of the run's rows; the driving is not affected. While the run is active,
`/log?get` and `/log?clear` answer 409 and the log page lists no files.

Rules:
- Add columns only at the end, bump the version in the header comment line (`# log v2`).
- Never log the full 360-bin LiDAR at 10 Hz (3.6 kB/row). The four sector minima are enough
  for timing analysis; a full scan is logged only on `note` events if ever needed.
- Budget: ~200 bytes/row × 10 rows/s ≈ 120 kB/min of driving, so the partition holds ~10 min
  of driving (standing: ~12 kB/min). The logger stops writing (and says so on the page and in the
  `[sys]` line) when the partition is 90 % full: download and clear after each session.
- Old CampusLogger files (`campus_run.csv`, header `time_ms,elapsed_s,yaw_deg,…`) are read by
  `plot_run.py` too: it renames their columns to v2 names.

## Web endpoints (`WebHMI::attachLogger()`, file access in `Logger.*`)

All `GET`. They exist only when `box.ino` calls `webHmi.attachLogger(&logger)` before `webHmi.begin()`;
without it `/log` answers 404. The "Dev" view of the main page links to `/log`.

| Route | Does | Answer |
|---|---|---|
| `/log` | log page: logger status, file list `run_<n>.csv` with size (newest last, each a download link), mark field, "Clear all logs" button | HTML |
| `/log?get=<name>` | downloads one CSV; `<name>` must be `run_<digits>.csv` (no paths) | `text/csv` as attachment; 400 bad name, 404 no such file. The file currently written is cut at its size when the download starts |
| `/log?clear=1` | deletes all logs, the current run restarts as an empty file with the header (the page asks for confirmation first) | `text/plain` "cleared"; 503 if the flash stayed busy |
| `/log?mark=<text>` | writes `<text>` (max 23 chars, `,` and line breaks replaced) into the `note` column of the next row (manual checkpoint marker while driving) | `text/plain` "marked" |

There is no single-file delete: `clear` removes everything. `tools/fetch_log.py` reads the
`run_<n>.csv` names from `/log` and downloads with `/log?get=<name>`.

## Analysis (`tools/`)

`plot_run.py` reads the CSV with pandas and plots, against `t_ms`:
ultrasonic distances, LiDAR sector minima, yaw, commanded `l`/`r`, state changes as vertical
lines, notes as markers. Output `<file>.png` beside the CSV. See `tools/README.md`.
