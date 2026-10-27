# Variables of the box firmware

Reference for every setting, state variable and struct field in `firmware/box/`, with unit and
meaning, plus the naming rules the code follows. Keep it in step with the code: a new or renamed
member gets a row here in the same change.

Comments in `.h` / `.cpp` / `.ino` files cost no flash and no RAM (the compiler drops them). Only
text inside string literals is stored on the chip, e.g. the web page in `WebHMI.cpp`.

## Naming rules

| Kind | Style | Example |
|---|---|---|
| local variable, parameter | `lowerCamelCase`, full words | `padCommandAgeMs`, `stepS` |
| private member | `_lowerCamelCase` | `_headingErrorIntegral` |
| user setting (`Config.h`, `NavConfig.h`) | `UPPER_SNAKE_CASE` | `PAD_TIMEOUT_MS` |
| file / class constant | `kUpperCamelCase` | `kDeadmanMs` |
| class, struct, enum type | `UpperCamelCase` | `LineTracker`, `PoleSeen` |
| enum value | `UPPER_SNAKE_CASE` | `STOPPED_LINK`, `NAV_LEG1` |
| boolean | starts with `is` / `has` | `isRunning`, `hasGpsFix` |
| loop counter | `i`, `j`, `k` (index only, nothing else) | `for (int i = 0; ...)` |

**Unit suffix** on every physical quantity, last part of the name:

| Suffix | Unit | Suffix | Unit |
|---|---|---|---|
| `Ms` | milliseconds | `Cm` | centimetres |
| `Us` | microseconds | `Mm` | millimetres |
| `S` | seconds | `Deg` | degrees |
| `Dps` | degrees per second | `Rad` | radians |
| `V` | volt | `Pct` | percent |
| `CentiDeg` | 1/100 degree (LiDAR protocol) | `PerS` | per second |

Angles follow ISO 8855 (vehicle frame) unless the name says otherwise: yaw + = turning left;
LiDAR degrees are clockwise from the front (0 front, 90 right, 180 back, 270 left).
Drive commands `driveLeft` / `driveRight` are −0.5 … 0.5 per side (0.5 = full speed).

---

## Settings — `src/core/Config.h`

| Name | Unit | Value | Meaning |
|---|---|---|---|
| `BOX_ID` | – | 6 | kit number; Wi-Fi name, password, IP and channel derive from it |
| `FW_VERSION` | text | – | firmware version shown on the page; change with every flash |
| `MAX_SPEED` | cmd | 0.5 | drive pad maximum per side |
| `PAD_TIMEOUT_MS` | ms | 400 | no drive-pad update for this long → stop |
| `ROVER_PERIOD_MS` | ms | 100 | RoverLink send period, control step and log tick |
| `LINK_TIMEOUT_MS` | ms | 3000 | autonomous run stops when the page has not polled for this long |
| `LIDAR_OFFSET_DEG` | deg | 90 | rotates the LiDAR so that 0 = box front |
| `USE_COMPASS` | bool | false | false = IMU without magnetometer (yaw relative to power-on) |
| `SNAPSHOT_MS` | ms | 250 | sensor snapshot and run card for the page |
| `UI_MS` | ms | 500 | state line and "Dev" values |
| `OLED_MS` | ms | 1000 | OLED refresh |
| `LOG_IDLE_PERIOD_MS` | ms | 1000 | log row period while standing without a run |
| `REPORT_MS` | ms | 10000 | `[sys]` line on Serial |

## Tuning of the autonomous run — `src/nav/NavConfig.h`

| Name | Unit | Value | Meaning |
|---|---|---|---|
| `CRUISE_SPEED` | cmd | 0.22 | straight-line speed |
| `HEADING_KP` | cmd/deg | 0.022 | heading hold, proportional gain |
| `HEADING_KI` | cmd/(deg·s) | 0.005 | heading hold, integral gain |
| `HEADING_KD` | cmd/(deg/s) | 0.004 | heading hold, damping on the gyro turn rate |
| `HEADING_I_LIMIT` | deg·s | 15 | anti-windup limit of the integral |
| `STEER_TRIM` | cmd | −0.05 | compensates the rover's pull to the right |
| `DRIVE_LIMIT` | cmd | 0.4 | per side while driving straight |
| `CM_PER_S_PER_CMD` | cm/s per cmd | 96 | speed per drive unit (odometry) |
| `LINE_GAIN_DEG_PER_CM` | deg/cm | 0.3 | heading correction per cm off the virtual line |
| `LINE_MAX_DEG` | deg | 6 | limit of that correction |
| `DRIFT_MAX_DEG` | deg | 2 | limit of one pole / wall drift correction |
| `CROSS_TRACK_MAX_CM` | cm | 20 | limit of the offset a pole may set |
| `MAX_RUN_TIME_MS` | ms | 0 | run time limit, 0 = none |
| `FRONT_STOP_CM` | cm | 45 | obstacle closer → pause |
| `FRONT_RESUME_CM` | cm | 60 | obstacle farther → resume |
| `FRONT_DEBOUNCE` | steps | 3 | steps in a row for pause / resume |
| `US_CLOSE_CM` | cm | 25 | front ultrasonic counts only below this |
| `CORRIDOR_HALF_WIDTH_CM` | cm | 13 | half width of the lane check |
| `CORRIDOR_LOOKAHEAD_CM` | cm | 150 | length of the lane check |
| `CLIFF_CHECK_ENABLED` | bool | false | cliff check on / off |
| `CLIFF_DELTA_CM` | cm | 12 | step down = baseline + this |
| `CLIFF_DEBOUNCE` | steps | 3 | steps in a row for a step down |
| `BASELINE_DEFAULT_CM` | cm | 17 | 45° ultrasonic on flat ground |
| `BASELINE_MIN_CM`, `BASELINE_MAX_CM` | cm | 8, 30 | valid range of a measured baseline |
| `CLIFF_MAX_STEP_CM` | cm | 150 | deepest echo that still counts as a step |
| `VOID_TIMEOUT_CYCLES` | steps | 14 | steps without echo before "cliff" |
| `POLE_DIAMETER_CM` | cm | 35 | real pole diameter |
| `POLE_MIN_RIGHT_CM`, `POLE_MAX_RIGHT_CM` | cm | 30, 250 | side distance window for poles |
| `POLES_BEFORE_TURN` | count | 4 | poles before the right turn |
| `TURN_DELAY_MS` | ms | 6000 | straight driving after the last pole |
| `POLE_SHAPE_CHECK` | bool | true | round-shape check of a pole |
| `POLE_DIAMETER_MIN`, `POLE_DIAMETER_MAX` | factor | 0.6, 1.5 | accepted fitted diameter × `POLE_DIAMETER_CM` |
| `POLE_FIT_MAX_CM` | cm | 2.5 | max. circle fit error |
| `POLE_LINE_CORRECTION` | bool | true | poles correct the line |
| `POLE_JUMP_CM` | cm | 25 | side distance jump → new reference, no steering |
| `WALL_ALIGNMENT` | bool | true | left wall holds the line |
| `WALL_MAX_CM` | cm | 250 | only walls closer than this |
| `WALL_MIN_POINTS` | count | 15 | LiDAR points on the wall line |
| `WALL_MIN_SPAN_CM` | cm | 60 | min. wall length |
| `WALL_FIT_MAX_CM` | cm | 3 | max. line fit error |
| `WALL_MAX_ANGLE_DEG` | deg | 20 | sanity limit of the wall angle |
| `WALL_SLOPE_CM` | cm | 150 | driving distance per wall drift correction |
| `WALL_DIST_GAIN` | 1/step | 0.2 | how fast the offset follows the wall distance |
| `WALL_JUMP_CM` | cm | 20 | wall distance jump → new reference |
| `TURN_SPEED` | cmd | 0.5 | in-place turn speed |
| `TURN_LEAD_S` | s | 0.25 | stop early by turn rate × this |
| `TURN_TOLERANCE_DEG` | deg | 8 | turn error accepted |
| `TURN_SETTLE_MS` | ms | 400 | stand still after a turn pulse |
| `TURN_PULSE_MAX_MS` | ms | 5000 | longest turn pulse |
| `TURN_MAX_PULSES` | count | 3 | turn pulses before driving on |

---

## `box.ino`

Global objects:

| Name | Type | Meaning |
|---|---|---|
| `webHmi` | `WebHMI` | Wi-Fi access point, web server, page |
| `roverLink` | `RoverLink` | drive commands to the rover, battery voltage |
| `sensors` | `Sensors` | all box sensors |
| `oledDisplay` | `DisplayManager` | OLED on the box |
| `logger` | `Logger` | timeline log on flash |
| `navigator` | `Navigator` | autonomous run |
| `isOledReady` | bool | OLED found and started |
| `boxSsid`, `boxPassword` | text | Wi-Fi name and password from `BOX_ID` |

`loop()`:

| Name | Unit | Meaning |
|---|---|---|
| `loopStartUs` | µs | start of this loop pass |
| `nowMs` | ms | time of this pass (`millis()`) |
| `isFirmwareUpdating` | bool | a firmware upload is running |
| `isUpdateHandled` | bool | the update start was handled once (rover stopped, ultrasonic suspended) |
| `startNewLogFile` | bool | next log row opens a new `run_<n>.csv` (set by START RUN) |
| `lastStepGridMs` | ms | grid time of the last control step (fixed 100 ms grid) |
| `lastStepTimeMs` | ms | real time of the last control step |
| `isControlStep` | bool | a control step runs in this pass |
| `stepS` | s | real duration of this control step (0.05 … 0.3) |
| `autoDriveLeft`, `autoDriveRight` | cmd | drive command of the autonomous run |
| `driveLeft`, `driveRight` | cmd | command sent to the rover in this pass |
| `padCommandAgeMs` | ms | age of the last drive-pad command |
| `previousLoopUs` | µs | duration of the previous pass (log column `loop_us`) |
| `lastLogRowMs` | ms | time of the last log row |
| `lastMotionMs` | ms | last time the rover was commanded to move |
| `isLogActive` | bool | log every step (run, motion, event) instead of once per second |
| `lastSnapshotMs`, `lastUiUpdateMs`, `lastOledUpdateMs`, `lastReportMs` | ms | last page snapshot / Dev values / OLED / Serial report |
| `loopCount`, `sumLoopUs`, `maxLoopUs` | –, µs, µs | loop statistics for the `[sys]` line |
| `loopDurationUs` | µs | duration of this pass |

Other functions: `logRow(timestampMs, driveLeft, driveRight, previousLoopUs, startNewFile)` fills
`logEntry` (a `Logger::Row`) from `navSnapshot` (`Navigator::Snapshot`); `handleCommands(nowMs,
isFirmwareUpdating)` reads `command` and returns `isRunStarted`; `updateOled()` uses
`lastInitAttemptMs`, `apAddress`, `statusLine`; `setup()` uses `wifiChannel`.

---

## Logger — `src/core/Logger.*`

`Logger::Row` (one CSV row; column name in brackets, see `docs/logging.md`):

| Field | Unit | Column |
|---|---|---|
| `timestampMs` | ms | `t_ms` |
| `state` | text | `state` |
| `driveLeft`, `driveRight` | cmd | `l`, `r` |
| `ultrasonicCm[5]` | cm | `us_f`, `us_l`, `us_r`, `us_b`, `us_fa` |
| `yawDeg`, `pitchDeg`, `rollDeg` | deg | `yaw`, `pitch`, `roll` |
| `isImuOk`, `isLidarOk` | 0/1 | `imu_ok`, `lid_ok` |
| `lidarSectorCm[4]` | cm | `lid_f`, `lid_l`, `lid_r`, `lid_b` |
| `hasGpsFix`, `latitudeDeg`, `longitudeDeg` | 0/1, deg | `gps_fix`, `lat`, `lon` |
| `batteryV` | V | `volt` |
| `loopUs` | µs | `loop_us` |
| `note` | text | `note` |
| `yawRateDps` | deg/s | `yaw_rate` |
| `headingRefDeg` | deg | `heading_ref` |
| `crossTrackCm` | cm | `cross_track` |
| `imuCalibration` | 4 digits | `imu_cal` |
| `frontCm` | cm | `front_cm` |
| `lidarCorridorCm` | cm | `lid_corr` |
| `lidarFrontRightCm`, `lidarFrontLeftCm` | cm | `lid_fr`, `lid_fl` |
| `isPaused`, `isSafetyEnabled` | 0/1 | `paused`, `safety` |
| `poleCount` | count | `poles` |
| `isPoleSeen`, `poleXCm`, `poleYCm`, `poleDiameterCm`, `poleFitErrorCm` | –, cm | `pole_x`, `pole_y`, `pole_d`, `pole_fit` (empty if not seen) |
| `lastPoleYCm` | cm | `last_pole_y` |
| `isWallSeen`, `wallCm`, `wallAngleDeg` | –, cm, deg | `wall_cm`, `wall_deg` (empty if not seen) |
| `isWallSteering` | 0/1 | `wall_steer` |
| `cliffBaselineCm` | cm | `cliff_base` |

Members:

| Name | Unit | Meaning |
|---|---|---|
| `_state` | enum | `NO_FS`, `LOGGING`, `FULL`, `FAILED` |
| `_fileName`, `_filePath` | text | current file `run_<n>.csv`, `/log/run_<n>.csv` |
| `_rowQueue` | queue | rows from `loop()` to the flush task (`Item` = row + `startsNewFile`) |
| `_fsMutex` | mutex | guards every LittleFS access |
| `_flushTaskHandle` | task | flush task on core 0 |
| `_markMux`, `_markText`, `_markPending` | –, text, bool | `/log?mark=` text for the next row |
| `_textBuffer`, `_textLength` | bytes | CSV text waiting for the flash |
| `_pendingRows` | count | rows in `_textBuffer` |
| `_lastFlushMs`, `_lastFillCheckMs` | ms | last flash write / last partition fill check |
| `_writeErrorCount` | count | failed writes in a row |
| `_fileBytes` | bytes | size of the current file |
| `_droppedRowCount` | count | rows lost (queue full, logger stopped) |
| `_usedBytes`, `_totalBytes` | bytes | LittleFS fill |
| `_isRunActive` | bool | autonomous run: rows go to RAM, no flash access |
| `_isRunWritePending` | bool | the flush task still writes the last run |
| `_runChunks[17]` | – | RAM run buffer, blocks of 100 `RunRow` (80 bytes each) |
| `_runCapacity`, `_runRowCount` | rows | size and fill of the RAM run buffer |
| `_runDroppedRows` | rows | run rows lost because the RAM buffer was full |
| `_runNotes`, `_runNotesLength` | text | note texts of the run rows |

`RunRow` holds one `Row` in small integers (`…10` = value × 10, `…100` = value × 100, the
decimals the CSV prints) plus `stateIndex`, `noteOffset` and the `flags` bits `FLAG_IMU`,
`FLAG_LIDAR`, `FLAG_GPS`, `FLAG_PAUSED`, `FLAG_SAFETY`, `FLAG_POLE`, `FLAG_WALL`, `FLAG_WALL_STEER`.

Constants: `kRunChunkRows` (100), `kRunMaxRows` (1700 = 170 s), `kRunHeapReserve` (80000 bytes),
`kRunNoteBytes` (2048), `kFormatVersion` (2), `kMaxFiles` (32), `kQueueRows` (32), `kFlushRows` (16),
`kFlushMs` (500 ms), `kFillCheckMs` (5000 ms), `kMaxLine` (400 bytes), `kMaxWriteErrors` (5),
`kLogDir` ("/log"), `kHeader` (CSV header).

---

## Autonomous run — `src/nav/`

`Navigator::Snapshot` (what the last control step saw):

| Field | Unit | Meaning |
|---|---|---|
| `yawRateDps` | deg/s | gyro turn rate |
| `headingRefDeg` | deg | heading the controller steered to |
| `crossTrackCm` | cm | offset from the virtual line, + = left |
| `corridorCm` | cm | LiDAR lane check, 0 = clear |
| `frontCm` | cm | value the obstacle check used, 0 = clear |
| `pole` | `PoleSeen` | pole detected in this scan |
| `wall` | `WallSeen` | left wall detected in this scan |

`PoleSeen`: `isFound`, `xCm` (forward, − = behind), `yCm` (right), `diameterCm`, `fitErrorCm`.
`WallSeen`: `isFound`, `angleDeg` (+ = we point left of the wall), `distanceCm`, `fitErrorCm`.

`Navigator` members:

| Name | Unit | Meaning |
|---|---|---|
| `_runState` | enum | `IDLE`, `RUNNING`, `STOPPED_MANUAL`, `STOPPED_OVERRIDE`, `STOPPED_LINK`, `STOPPED_CLIFF`, `STOPPED_TIMEOUT` |
| `_phase` | enum | `NAV_LEG1`, `NAV_WAIT`, `NAV_TURN`, `NAV_SETTLE`, `NAV_LEG2` |
| `_startedAtMs` | ms | real start of the run |
| `_runClockMs` | ms | start, moved forward by obstacle pauses (time limit, elapsed) |
| `_phaseStartMs` | ms | start of the current phase |
| `_turnTargetYawDeg` | deg | heading the turn aims at |
| `_turnPulseCount` | count | turn pulses done |
| `_headingErrorIntegral` | deg·s | integral of the heading error |
| `_lineTracker`, `_safetyMonitor` | objects | parts of the run |
| `_snapshot` | `Snapshot` | last step |
| `_eventText` | text | events for the log `note` column |

`LineTracker` members:

| Name | Unit | Meaning |
|---|---|---|
| `_targetYawDeg` | deg | heading of the line |
| `_headingRefDeg` | deg | last heading reference |
| `_crossTrackCm` | cm | offset from the line, + = left |
| `_odometryCm` | cm | distance driven in this run |
| `_selfDriftLeftCm` | cm | sideways motion caused by our own heading |
| `_lastDriveLeft`, `_lastDriveRight` | cmd | last command (odometry) |
| `_poleCount` | count | poles counted |
| `_isPoleTracking` | bool | a pole candidate is followed |
| `_trackStartXCm`, `_trackStartOdometryCm` | cm | candidate position and odometry when first seen |
| `_lastTrackXCm` | cm | candidate position in the last scan |
| `_poleAbeamCount` | scans | scans with the pole beside us |
| `_referencePoleYCm` | cm | side distance that means "on the line" |
| `_lastPoleYCm`, `_lastPoleOdometryCm` | cm | last pole used for drift |
| `_lastCountedPoleYCm` | cm | side distance of the last counted pole |
| `_selfDriftAtLastPoleCm` | cm | `_selfDriftLeftCm` at the last pole |
| `_wallFilteredCm` | cm | smoothed wall distance |
| `_slopeStartOdometryCm`, `_slopeStartWallCm`, `_slopeStartSelfDriftCm` | cm | start values of the wall drift window |
| `_isWallLocked` | bool | a wall reference is set |
| `_isWallTracking` | bool | the wall steers right now |
| `_wallReferenceCm` | cm | wall distance that means "on the line" |
| `_wallSeenCount`, `_wallMissedCount` | scans | wall debounce |
| `_eventText`, `_eventOutput` | text | events / last read events |

`SafetyMonitor` members:

| Name | Unit | Meaning |
|---|---|---|
| `_baselineSamplesCm[kBaselineSamples]` | cm | idle 45° ultrasonic readings (15 = 1.5 s) |
| `_baselineSampleCount`, `_baselineSampleIndex` | – | fill state of that buffer |
| `_baselineCm` | cm | ground baseline of the cliff check |
| `_cliffHistoryCm[3]` | cm | last three 45° readings (median filter) |
| `_cliffFilteredCm` | cm | median of them |
| `_obstacleCount`, `_clearCount` | steps | obstacle pause debounce |
| `_stepDropCount`, `_noEchoCount` | steps | cliff debounce |
| `_isPaused` | bool | obstacle pause active |
| `_isEnabled` | bool | safety stops on |
| `_eventText`, `_eventOutput` | text | events |

`Perception` and `NavMath` keep no state; their locals are named after what they hold
(`pointsX`, `pointsY`, `pointCount`, `centerX`, `centerY`, `radius`, `rmse`, `slope`, `intercept`,
`nearestCm`, `bestPole`, `groupWidthCm`, …). In `fitCircle` the sums `sumUU`, `sumVV`, `sumUV`,
`sumUZ`, `sumVZ`, `sumZ` are the least-squares sums over the centred points (`offsetU`,
`offsetV`, `squaredDistance`).

---

## Sensors — `src/sensors/`

`Sensors` members:

| Name | Unit | Meaning |
|---|---|---|
| `_ultrasonic`, `_lidar`, `_gps`, `_imu` | objects | the sensor drivers |
| `_lidarOffsetDeg` | deg | LiDAR rotation to the vehicle frame |
| `_useCompass` | bool | IMU with magnetometer |
| `_lastSnapshotMs`, `_lastImuAttemptMs`, `_lastPacketRateMs` | ms | last snapshot / IMU start attempt / packet rate |
| `_lastPacketCount` | count | LiDAR packets at the last rate calculation |
| `_ultrasonicCm[5]` | cm | page snapshot of the ultrasonic sensors |
| `_isLidarOk` | bool | LiDAR sends data |
| `_lidarPacketsPerS`, `_lidarCrcErrorPct` | 1/s, % | LiDAR packet rate, checksum errors |
| `_lidarCm[90]` | cm | page snapshot, nearest return per 4° |
| `_isImuReady` | bool | IMU started |
| `_yawSnapshotDeg`, `_pitchSnapshotDeg`, `_rollSnapshotDeg` | deg | page snapshot of the IMU |
| `_imuCalibration[4]` | 0…3 | calibration levels system, gyro, accel, mag |
| `_hasGpsFix`, `_satelliteCount`, `_hdop` | –, count, – | GPS state |
| `_latitudeDeg`, `_longitudeDeg` | deg | last GPS position |
| `_gpsCharCount` | count | NMEA characters received |

Constants: `kLidarPoints` (90), `kUsMaxCm` (300 = no echo), `kImuI2cAddress` (0x28),
`kSnapshotMs` (250 ms), `kImuRetryMs` (5000 ms).

`UsManager`: `_pins` (trigger/echo per sensor), `_maxDistanceCm`, `_timeoutUs`, `_taskHandle`,
`_frontCm`, `_leftCm`, `_rightCm`, `_backCm`, `_frontAngledCm` (last measurement, cm),
`kPauseBetweenSensorsMs` (5 ms).

`ImuManager`: `_isReady`, `_isCalibrationLoaded`, `_lastUpdateMs` (ms), `_fusionYawDeg`,
`_magYawDeg` (deg, sensor frame), `_pitchDeg`, `_rollDeg`, `_yawRateDps` (vehicle frame),
`_isNorthAnchored`, `_northOffsetDeg` (deg), `_calibration[4]` (0…3); constants `kBnoI2cAddress`,
`kUpdatePeriodMs` (20 ms), `kNorthAlignMs` (3000 ms), `kPrefsNamespace`, `kPrefsKey`;
`bnoSensor` (the BNO055 driver).

`GpsManager`: `_parser` (TinyGPSPlus); constants `kRxPin`, `kTxPin`, `kBaudRate`, `kMaxFixAgeMs`.

`LidarScan`: `_config` (`Config`: `rxPin`, `txPin`, `baud`, `angleShiftDeg`, `minMm`, `maxMm`,
`expireMs`), `_port`, `_crcTable[256]`, `_frameBuffer`, `_frameIndex`, `_packetCount`,
`_crcErrorCount`, `_bins[360]`. `Bin`: `hitMm[3]`, `hitMs[3]` (last three hits), `nextSlot`.
`Frame` (LD20 protocol): `header`, `verLen`, `speed`, `startAngleCentiDeg`, `points[12]`
(`Pt`: `distanceMm`, `intensity`), `endAngleCentiDeg`, `timestamp`, `crc`.

---

## Communication and display — `src/comms/`, `src/display/`

`RoverLink` members:

| Name | Unit | Meaning |
|---|---|---|
| `_periodMs` | ms | send period |
| `_setPointMux` | lock | guards the set point |
| `_setLeft`, `_setRight` | cmd | set point from `loop()` |
| `_setTimeMs` | ms | time of the set point (deadman) |
| `_roverIp` | IPv4 | rover address, 0 = searching |
| `_voltage` | V | rover battery |
| `_rttMs` | ms | smoothed round-trip time |
| `_okCount`, `_failCount` | count | commands since start |
| `_failsInRow` | count | consecutive failures |

Constants: `kHttpTimeoutMs` (400), `kStopRepeats` (3), `kLostAfterFails` (10), `kDeadmanMs` (500),
`kSearchEveryMs` (2000), `kIdleEveryMs` (500). Task locals: `lastSearchMs`, `lastIdleMs`,
`cycleStartMs`, `roverIp`, `driveLeft`, `driveRight`, `setTimeMs`, `usedMs`, `stopsLeft`,
`idleStatus`.

`WebHMI` members:

| Name | Unit | Meaning |
|---|---|---|
| `_server`, `_logger` | objects | web server, optional logger |
| `_title` | text | page title |
| `_isDrivePadEnabled`, `_driveMaxSpeed` | bool, cmd | drive pad shown, slider maximum |
| `_dataMutex` | mutex | guards state text, JSON, message, values, loop times |
| `_state`, `_sensorJson`, `_runJson`, `_message` | text | content for `/values` |
| `_commandQueue` | queue | run-card buttons, server → `loop()` |
| `_lastPollMs` | ms | last `/values` request (link watchdog) |
| `_firmwareVersion` | text | shown on the page |
| `_otaEnabled`, `_otaAllowed`, `_otaActive` | bool | firmware update available / allowed / running |
| `_otaRequest` | pointer | the accepted upload |
| `_otaLastDataMs` | ms | last upload chunk (stall detection) |
| `_restartAtMs` | ms | restart time after an upload, 0 = none |
| `_values[kMaxValues]` | – | "Dev" values (`name`, `text`, `inUse`) |
| `_cycleMinMs`, `_cycleMaxMs`, `_cycleAvgMs` | ms | loop time for the page |
| `_driveMux` | lock | guards the drive-pad set point |
| `_driveLeft`, `_driveRight` | cmd | last drive-pad command |
| `_driveReceivedMs` | ms | time it arrived, 0 = never |
| `_drivePageId`, `_driveSequence` | – | page instance and number of the newest accepted command |
| `_cycleLastUs`, `_cycleWindowStartMs`, `_cycleMinUs`, `_cycleMaxUs`, `_cycleCount`, `_cycleSumUs` | µs, ms | loop time statistics |

`DisplayManager`: `_oled` (SSD1306 driver), `_isReady`; constants `kWidth`, `kHeight`,
`kI2cAddress`, `kMaxChars`, `kLineStep`.

The JavaScript inside the page (`kPage` in `WebHMI.cpp`) keeps its short names on purpose: it is
sent to the phone as text from flash, so every character there costs memory.

---

## Renamed on 26.10.2026 (old → new)

For reading the CampusLogger sketch, the starter program or older notes.

| Where | Old → new |
|---|---|
| `box.ino` globals | `hmi` → `webHmi`, `rover` → `roverLink`, `disp` → `oledDisplay`, `nav` → `navigator`, `oledReady` → `isOledReady` |
| `box.ino` `loop()` | `t0` → `loopStartUs`, `now` → `nowMs`, `age` → `padCommandAgeMs`, `l`/`r` → `driveLeft`/`driveRight`, `navL`/`navR` → `autoDriveLeft`/`autoDriveRight`, `dt` → `stepS`, `stepNow` → `isControlStep`, `lastStep` → `lastStepGridMs`, `lastStepMs` → `lastStepTimeMs`, `lastMoveMs` → `lastMotionMs`, `lastRowMs` → `lastLogRowMs`, `active` → `isLogActive`, `prevLoopUs` → `previousLoopUs`, `updating` → `isFirmwareUpdating`, `updateSeen` → `isUpdateHandled`, `newLogFile` → `startNewLogFile`, `lastSnap`/`lastUi`/`lastOled`/`lastRep` → `lastSnapshotMs`/`lastUiUpdateMs`/`lastOledUpdateMs`/`lastReportMs`, `n` → `loopCount`, `sumUs`/`maxUs` → `sumLoopUs`/`maxLoopUs`, `loopUs` → `loopDurationUs` |
| `Logger::Row` | `tMs` → `timestampMs`, `l`/`r` → `driveLeft`/`driveRight`, `us` → `ultrasonicCm`, `yaw`/`pitch`/`roll` → `yawDeg`/`pitchDeg`/`rollDeg`, `lid` → `lidarSectorCm`, `lat`/`lon` → `latitudeDeg`/`longitudeDeg`, `volt` → `batteryV`, `lidCorr` → `lidarCorridorCm`, `lidFr`/`lidFl` → `lidarFrontRightCm`/`lidarFrontLeftCm`, `imuCal` → `imuCalibration`, `poles` → `poleCount`, `poleD`/`poleFit` → `poleDiameterCm`/`poleFitErrorCm`, booleans with `is`/`has` |
| `Logger` | `_name`/`_path` → `_fileName`/`_filePath`, `_queue` → `_rowQueue`, `_mtx` → `_fsMutex`, `_buf`/`_bufLen` → `_textBuffer`/`_textLength`, `_bytes` → `_fileBytes`, `_dropped` → `_droppedRowCount`, `_used`/`_total` → `_usedBytes`/`_totalBytes`, `kDir` → `kLogDir` |
| `Navigator` | `_run` → `_runState`, `_integral` → `_headingErrorIntegral`, `_turnPulses` → `_turnPulseCount`, `_line`/`_safety`/`_snap` → `_lineTracker`/`_safetyMonitor`/`_snapshot`; methods `running()`/`paused()`/`poles()`/`last()` → `isRunning()`/`isPaused()`/`poleCount()`/`lastSnapshot()` |
| `LineTracker` | `_odoCm` → `_odometryCm`, `_drLeftCm` → `_selfDriftLeftCm`, `_prevL`/`_prevR` → `_lastDriveLeft`/`_lastDriveRight`, `_refPoleY` → `_referencePoleYCm`, `_wallFilt` → `_wallFilteredCm`, `_wallGood`/`_wallBad` → `_wallSeenCount`/`_wallMissedCount`, `headingRef()` → `updateHeadingRefDeg()` |
| `SafetyMonitor` | `_baseBuf` → `_baselineSamplesCm`, `_cliffHist` → `_cliffHistoryCm`, `_hazardCount` → `_obstacleCount`, `_voidCount` → `_noEchoCount`; `enabled()`/`paused()` → `isEnabled()`/`isPaused()` |
| `PoleSeen` / `WallSeen` | `found`/`ok` → `isFound`, `x`/`y` → `xCm`/`yCm`, `d` → `diameterCm`, `fit` → `fitErrorCm`, `distCm` → `distanceCm` |
| `Sensors` | `_us` → `_ultrasonic`, `_usCm` → `_ultrasonicCm`, `_yaw`/`_pitch`/`_roll` → `_yawSnapshotDeg`/…, `_cal` → `_imuCalibration`, `_sats` → `_satelliteCount`, `_lidarPps` → `_lidarPacketsPerS`; `imuOk()`/`lidarOk()`/`gpsFix()` → `isImuOk()`/`isLidarOk()`/`hasGpsFix()` |
| `RoverLink` | `_setL`/`_setR`/`_setMs` → `_setLeft`/`_setRight`/`_setTimeMs`, `_ipRaw` → `_roverIp`, `_ok`/`_fail` → `_okCount`/`_failCount`, `_failRow` → `_failsInRow`, `_cmd()` → `_sendCommand()` |
| `WebHMI` | `_mtx` → `_dataMutex`, `_cmdQueue` → `_commandQueue`, `_driveL`/`_driveR`/`_driveMs`/`_driveSeq` → `_driveLeft`/`_driveRight`/`_driveReceivedMs`/`_driveSequence`, `_drivePad`/`_driveMax` → `_isDrivePadEnabled`/`_driveMaxSpeed`, `_otaReq` → `_otaRequest` |
| `ImuManager` | `bno` → `bnoSensor`, `_cal` → `_calibration`, `_ready` → `_isReady`, `_northAnchored` → `_isNorthAnchored` |
| `LidarScan` | `_cfg` → `_config`, `_buf`/`_idx` → `_frameBuffer`/`_frameIndex`, `Bin.mm`/`ms`/`head` → `hitMm`/`hitMs`/`nextSlot`, `Frame.p` → `points` |
| others | `UsManager::_task` → `_taskHandle`, `GpsManager::_gps` → `_parser`, `DisplayManager::_disp` → `_oled` |

The CSV column names (`t_ms`, `l`, `r`, `us_f`, …) did not change: they are the log format,
read by `tools/plot_run.py`.
