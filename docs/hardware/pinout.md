# Box ESP32 pinout (fixed hardware)

Extracted from the CAS starter program v1.0 and the sensor test sketch. The box PCB cannot be
rewired, so this table is a fact sheet. The column *defined in* says where the number lives in
`firmware/box/src/` (sensor pins in `src/sensors/`); change it there and here together if a measurement ever proves a pin wrong.

## Sensors

| Sensor | Signal | GPIO | Defined in | Notes |
|---|---|---|---|---|
| Ultrasonic **front** (HC-SR04) | TRIG / ECHO | 32 / 25 | `UsManager.h` `Pins` | echo via 1k/2k divider (5 V → 3.3 V) |
| Ultrasonic **left** | TRIG / ECHO | 4 / 18 | `UsManager.h` | |
| Ultrasonic **right** | TRIG / ECHO | 13 / 19 | `UsManager.h` | |
| Ultrasonic **back** | TRIG / ECHO | 5 / 14 | `UsManager.h` | |
| Ultrasonic **front, 45° down** | TRIG / ECHO | 23 / 26 | `UsManager.h` | drop / ramp detection |
| LiDAR **LD20** (top) | UART RX ← LiDAR TX / TX | 27 / 33 | `LidarScan.h` `Config` | `Serial1`, 230400 baud, ~10 Hz, 12 pts/frame, CRC8 |
| GPS **NEO-6M** (GY-GPS6MV2) | UART RX / TX | 16 / 17 | `GpsManager.cpp` | `Serial2`, 9600 baud, NMEA |
| IMU **BNO055** | I2C SDA / SCL | 21 / 22 | `Sensors.cpp` (`Wire.begin`) | address `0x28`, NDOF fusion, mounted turned 90° |
| OLED **SSD1306** 128×32 | I2C SDA / SCL | 21 / 22 | `Sensors.cpp` / `DisplayManager` | address `0x3C` |
| USB / programming | UART0 | 1 / 3 | – | CH340, 115200 baud serial monitor |

Ultrasonic index used throughout the code (`Sensors::usCm(i)`): 0 front, 1 left, 2 right, 3 back, 4 front angled.
The test sketch in `Test_Programs/Test_Code` orders them differently (45°, front, back, left, right); the
starter program order above is the one this repo uses.

## Ranges and timing

| Sensor | Range / rate | "Nothing" value |
|---|---|---|
| Ultrasonic | up to 300 cm, 17.5 ms echo timeout, all five measured round-robin in a core-0 task, ~4 Hz per sensor | `300` (`Sensors::kUsMaxCm`) |
| LiDAR | 0.2 m – 12 m, 360 bins of 1°, snapshot reduced to 90 points × 4° for the page | `0` |
| IMU | roll / pitch / yaw in ISO 8855 vehicle frame (x fwd, y left, z up), one read per 100 ms, 3.5 s blocking north alignment at start | `imu.ok=false` |
| GPS | lat / lon / sats / HDOP; fix valid for 3 s | `fix=false` |

## Free GPIO (candidates for the CAM link)

GPIOs not in the table above and usable on an ESP32 Dev Module: 2 (boot LED), 12 (strapping, avoid), 15, 34, 35, 36, 39 (input only).
Verify on the box before using any of them. Record the decision in `docs/decisions/`.

## ESP32-CAM

Separate module, pins fixed by the AI-Thinker board: see `firmware/cam/CamConfig.h`.
