# firmware/

Two independent Arduino sketches, each flashed through its **own port**:

| Sketch | Board (Tools → Board) | Port | Partition scheme |
|---|---|---|---|
| [`box/box.ino`](box/box.ino) | **ESP32 Dev Module** | CH340 USB on the box | default |
| [`cam/cam.ino`](cam/cam.ino) | **AI Thinker ESP32-CAM** | USB-TTL adapter / CAM programmer, second port | Huge APP (3MB No OTA/1MB SPIFFS) |

Arduino IDE rule: the folder name must equal the `.ino` name (`box/box.ino`, `cam/cam.ino`).
Every `.h`/`.cpp` next to the `.ino` **and everything under its `src/` folder (recursively)** is
compiled with it. Only root-level files open as IDE tabs; edit `src/` files in VS Code or via
*File → Open*. `box/` keeps its modules in `src/`:

```
box/
├── box.ino          setup(), loop()
├── sketch.yaml      arduino-cli build profile
├── VARIABLES.md     naming rules and every setting / variable with unit and meaning
└── src/
    ├── core/        Config.h, Logger.*
    ├── sensors/     Sensors.*, UsManager.*, ImuManager.*, GpsManager.*, LidarScan.h
    ├── comms/       RoverLink.*, WebHMI.*
    ├── nav/         autonomous run: NavConfig.h, Navigator.*, LineTracker.*, SafetyMonitor.*,
    │                Perception.*, NavMath.*  (docs/autonomous_run.md)
    └── display/     DisplayManager.*
```

Include rules: `box.ino` uses `#include "src/<folder>/<File>.h"`; a file in `src/` includes a
file in the same folder by name (`"UsManager.h"`) and one in another folder relatively
(`"../core/Config.h"`). Never put a `.cpp` in a folder called anything other than `src/...`
next to the `.ino`: the Arduino builder would not compile it.
The two sketches share nothing at compile time; what they share (the message format between
CAM and box) is written down in [`../docs/architecture.md`](../docs/architecture.md).

## Toolchain (both sketches)

- Arduino IDE **2.x**
- Boards Manager: **esp32** by Espressif Systems, **3.3.1** (any 3.3.x; 2.x does not work)
- Serial monitor **115200 baud**

## Libraries for `box/`

Library Manager names and the versions the starter program was tested with:

| Library | Version | Used by |
|---|---|---|
| ESP Async WebServer (ESP32Async) | 3.9.2 | WebHMI |
| Async TCP (ESP32Async) | 3.4.9 | WebHMI |
| Adafruit BNO055 | 1.6.4 | ImuManager |
| Adafruit Unified Sensor | 1.1.15 | ImuManager |
| Adafruit SSD1306 | 2.5.16 | DisplayManager |
| Adafruit GFX Library | 1.12.4 | DisplayManager |
| Adafruit BusIO | 1.17.4 | I2C helpers |
| TinyGPSPlus-ESP32 (Mikal Hart) | 0.0.2 | GpsManager |

WiFi, HTTPClient, Wire, SPI, LittleFS (timeline log), Preferences (saved IMU calibration) and
Update (firmware update over Wi-Fi) come with the esp32 core: no extra library for them.
Each sketch carries a `sketch.yaml` build profile, so once `arduino-cli` is installed the
build is reproducible without clicking through the IDE:

```bash
arduino-cli compile --profile box firmware/box
```

## Libraries for `cam/`

Only the esp32 core (`esp_camera.h`) so far. Add rows here when the sketch grows.

## Flash procedure (box)

1. `box/src/core/Config.h` → `BOX_ID` = kit number on the sticker. Change nothing else of the naming scheme.
   Set `FW_VERSION` to something new: the page shows it, so you can see which version runs.
2. Board *ESP32 Dev Module*, partition *default*, port = COM port of the box.
3. Upload, open the serial monitor at 115200. A `[sys]` status line appears every 10 s.
4. On the field: power the **box first, then the rover**.

### Update over Wi-Fi (no USB)

1. Arduino IDE: *Sketch → Export Compiled Binary*. It writes `box/build/esp32.esp32.esp32/box.ino.bin`
   (`build/` and `*.bin` are git-ignored).
2. Phone or laptop on `CAS-Box-NN`, page → "update over Wi-Fi" (`/update`), choose
   **`box.ino.bin`** (not `.merged.bin` / `.bootloader.bin`), upload. Not possible during a run.
3. The box stops the rover, restarts with the new firmware after ~10 s (after a failed upload
   it restarts with the old one). The log files and the saved IMU calibration stay.

This needs the *default* partition scheme (two 1.25 MB app slots, 1.4 MB LittleFS). The
box sketch is 1.23 MB = 93 % of one slot (core 3.3.12, 26.10.2026), so room is short: if a build
ever reports "Sketch too big", do not switch to a scheme without OTA or with a smaller LittleFS
without updating docs/logging.md.

## Flash procedure (cam)

The ESP32-CAM has no USB. Wire a USB-TTL adapter (5V, GND, U0R↔TX, U0T↔RX), pull **IO0 to GND**
while powering up to enter the bootloader, upload, remove the IO0 jumper, press reset.
Details and the camera pin map: [`cam/README.md`](cam/README.md).
