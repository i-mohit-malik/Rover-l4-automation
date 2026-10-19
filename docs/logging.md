# Timeline logging

## Why

The phone page polls `/values` every 250 ms over Wi-Fi: good enough to watch, not good enough to
reason about timing. The log is written **on the box, in the loop that makes the decision**, with
the box millisecond clock. It is read **after** the drive.

## Where the data lives

| Stage | Place | Format |
|---|---|---|
| During the drive | box flash, LittleFS partition (default partition scheme: ~1.4 MB) | one CSV file per boot, `/log/run_<boot>.csv` |
| After the drive | laptop or phone via `http://192.168.(10+N).1/log` (download) | same CSV |
| Archived | `L4_Rover/04_SAFETY_AND_TESTING/Test_Results/YYYY-MM-DD_HHMM_<short-note>.csv` (outside this repo) | same CSV, renamed |
| Analysed | `tools/plot_run.py <file>.csv` | PNG next to the CSV |

Serial logging stays available when the box is tethered to the laptop, but the field runs are
untethered, so flash is the primary store.

## Row format (v1)

One row per log tick (default every 100 ms = the RoverLink period). Header line first.
Fixed column order so that the Python side never guesses.

```
t_ms,state,l,r,us_f,us_l,us_r,us_b,us_fa,yaw,pitch,roll,imu_ok,lid_ok,lid_f,lid_l,lid_r,lid_b,gps_fix,lat,lon,volt,loop_us,note
```

| Column | Unit | Source |
|---|---|---|
| `t_ms` | ms since boot | `millis()` |
| `state` | string | brain state (`MANUAL`, `ESTOP`, `CP1_STRAIGHT`, …) |
| `l`, `r` | −0.5…0.5 | what was sent to `rover.setDrive()` this tick |
| `us_f … us_fa` | cm | `Sensors::usCm(0..4)`; 300 = nothing |
| `yaw`, `pitch`, `roll` | deg | ISO 8855, `imu_ok` 0/1 |
| `lid_f`, `lid_l`, `lid_r`, `lid_b` | cm | nearest LiDAR return in the ±10° sector of each direction; 0 = none |
| `gps_fix`, `lat`, `lon` | 0/1, deg | GPS |
| `volt` | V | rover battery from `rover.voltage()` |
| `loop_us` | µs | duration of the previous loop pass |
| `note` | string | free text set by code, e.g. `cp_reached`, `estop_us_f` (empty otherwise) |

Rules:
- Add columns only at the end, bump the version in the header comment line (`# log v2`).
- Never log the full 360-bin LiDAR at 10 Hz (3.6 kB/row). The four sector minima are enough
  for timing analysis; a full scan is logged only on `note` events if ever needed.
- Budget: ~120 bytes/row × 10 rows/s ≈ 72 kB/min, so one 1.4 MB partition holds ~18 min.
  The logger stops writing (and says so on the page) when the partition is 90 % full.

## Web endpoints (`WebHMI::attachLogger()`, file access in `Logger.*`)

All `GET`. They exist only when `box.ino` calls `hmi.attachLogger(&logger)` before `hmi.begin()`;
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
