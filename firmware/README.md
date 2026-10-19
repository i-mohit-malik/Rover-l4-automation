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
└── src/
    ├── core/        Config.h, Logger.*
    ├── sensors/     Sensors.*, UsManager.*, ImuManager.*, GpsManager.*, LidarScan.h
    ├── comms/       RoverLink.*, WebHMI.*
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

WiFi, HTTPClient, Wire, SPI and LittleFS come with the esp32 core.
Each sketch carries a `sketch.yaml` build profile, so once `arduino-cli` is installed the
build is reproducible without clicking through the IDE:

```bash
arduino-cli compile --profile box firmware/box
```

## Libraries for `cam/`

Only the esp32 core (`esp_camera.h`) so far. Add rows here when the sketch grows.

## Flash procedure (box)

1. `box/src/core/Config.h` → `BOX_ID` = kit number on the sticker. Change nothing else of the naming scheme.
2. Board *ESP32 Dev Module*, partition *default*, port = COM port of the box.
3. Upload, open the serial monitor at 115200. A `[sys]` status line appears every 10 s.
4. On the field: power the **box first, then the rover**.

## Flash procedure (cam)

The ESP32-CAM has no USB. Wire a USB-TTL adapter (5V, GND, U0R↔TX, U0T↔RX), pull **IO0 to GND**
while powering up to enter the bootloader, upload, remove the IO0 jumper, press reset.
Details and the camera pin map: [`cam/README.md`](cam/README.md).
