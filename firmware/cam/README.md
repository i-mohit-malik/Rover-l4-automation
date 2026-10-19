# firmware/cam – ESP32-CAM

Separate device, separate port, separate upload. Nothing here is compiled into the box sketch.

## Status

Placeholder: heartbeat on Serial, LED blink. The camera driver is not started yet.
Open questions before the first real version (write the answers into
[docs/architecture.md](../../docs/architecture.md)):

1. How is the CAM connected to the box slot: power only, or also a UART/GPIO line to the box?
2. Does the CAM join the box Wi-Fi (`CAS-Box-NN`) as a station, or open its own AP?
3. Who consumes the image: the phone (MJPEG stream for driving) or the box (detections)?

## Flashing

The AI-Thinker ESP32-CAM has no USB port.

1. USB-TTL adapter: `5V→5V`, `GND→GND`, `TX→U0R`, `RX→U0T`.
2. Jumper **IO0 → GND**, then power up (bootloader mode).
3. Arduino IDE: board *AI Thinker ESP32-CAM*, partition *Huge APP (3MB No OTA/1MB SPIFFS)*,
   port of the adapter, upload.
4. Remove the IO0 jumper, press the reset button, open the serial monitor at 115200.

If the box slot already contains a programmer for the CAM, document its port here.

## Pins

See [`CamConfig.h`](CamConfig.h). GPIO 4 drives the flash LED and is shared with the SD card.
GPIO 1/3 are the serial port used for flashing and debugging; GPIO 12/13/14/15 are free only
if no SD card is used.
