# Architecture

## Devices

```
┌─────────────┐  Wi-Fi AP CAS-Box-NN   ┌───────────────────────────────┐  HTTP /js?json  ┌───────────────┐
│   phone     │◄──────────────────────►│  box ESP32  (firmware/box)    │────────────────►│  WAVE ROVER   │
│  web page   │   /values  /drive      │  sensors · HMI · log · brain  │                 │  (frozen)     │
└─────────────┘                        └───────────────┬───────────────┘                 └───────────────┘
                                                       │  (link TBD: UART / Wi-Fi)
                                        ┌──────────────┴────────────────┐
                                        │  ESP32-CAM  (firmware/cam)    │
                                        │  front camera                 │
                                        └───────────────────────────────┘
```

- **Box ESP32**: the only place where decisions are made. Reads all sensors, serves the phone
  page, writes the timeline log, sends `L/R` to the rover.
- **ESP32-CAM**: separate processor and port. Role to be decided (ADR pending): MJPEG stream to the
  phone for remote driving, and/or detections (marker, line, colour) sent to the box.
- **Rover**: motors + battery. Black box, see [rover_interface.md](rover_interface.md).
- **Phone**: manual driving in the early phase, live sensor view, log download.

## Box firmware modules (`firmware/box/`)

| File | Role | Runs on |
|---|---|---|
| `box.ino` | `setup()` / `loop()`: wiring of the modules, drive decision | core 1, loop |
| `src/core/Config.h` | kit number and tunables, the only file a team member edits before flashing | – |
| `src/sensors/Sensors.*` | one object for all sensors, 4 Hz snapshot, `toJson()` for the page | core 1 |
| `src/sensors/UsManager.*` | five HC-SR04 measured round-robin in a background task | core 0 task |
| `src/sensors/LidarScan.h` | LD20 frame parser with median filtering, 360 bins | core 1 (UART drain) |
| `src/sensors/ImuManager.*` | BNO055 → ISO 8855 roll / pitch / yaw | core 1 |
| `src/sensors/GpsManager.*` | NEO-6M NMEA via TinyGPSPlus | core 1 |
| `src/display/DisplayManager.*` | 128×32 OLED status | core 1 |
| `src/comms/WebHMI.*` | Wi-Fi AP + async web server + phone page | server task |
| `src/comms/RoverLink.*` | HTTP commands to the rover, deadman, voltage poll | core 0 task |
| `src/core/Logger.*` | timeline CSV on LittleFS, `/log` download | core 1 + flush task |
| *(planned)* `Brain.*` | state machine: manual → assisted → autonomous per checkpoint | core 1 |

Rule: `loop()` must stay fast (the starter reports avg/max loop time every 10 s). Anything that
blocks longer than a few ms goes into its own FreeRTOS task on core 0, as `UsManager` and
`RoverLink` already do.

## Phases

| Phase | Driving | Code focus | Evidence |
|---|---|---|---|
| 0 | manual, phone pad | starter program as is, logging added | logs in `04_SAFETY_AND_TESTING/Test_Results/`, loop time |
| 1 | manual | sensor characterisation on the real track: what does each sensor show at each checkpoint? | plots from `tools/plot_run.py` |
| 2 | assisted | emergency stop from front ultrasonic / LiDAR while the human drives | log shows stop before contact |
| 3 | autonomous per checkpoint | one behaviour per checkpoint (straight, turn by IMU yaw, follow wall by side ultrasonic, …) | checkpoint reached, logged |
| 4 | autonomous whole track | state machine chaining the behaviours; CAM input if ready | demo run, 21 Jan 2027 |

Each phase ends with a `drive-YYYY-MM-DD` tag and a row in the test log (`04_SAFETY_AND_TESTING/Test_Results/test_log.md`).

## Data flow for a decision

```
sensors.update()  →  snapshot (us[5], lidar[90], yaw/pitch/roll, gps)
                  →  Brain::decide(snapshot, state)  →  (l, r, state')
                  →  rover.setDrive(l, r)
                  →  Logger::row(t, snapshot, l, r, state)   // same tick, same numbers
```

The log row is written from the same snapshot the decision used, so the post-drive plot shows
exactly what the brain saw, not what the phone page happened to poll.
