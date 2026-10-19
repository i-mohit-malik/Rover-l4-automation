// =============================================================================================
//  CAS Box - starter program for the sensor box on the WAVE ROVER
//  Case Study Cooperative and Autonomous Systems, version 1.0 (2026-10-15)
// =============================================================================================
//
//  WHAT IT DOES
//    The box ESP32 is the "brain" of your robot. It opens its own Wi-Fi, serves a web page for
//    your phone, reads all box sensors and drives the WAVE ROVER underneath it over Wi-Fi.
//
//      phone ──Wi-Fi──► box (this program): web page, sensors ──Wi-Fi──► rover (motors)
//
//    Web page: drive pad (hold a key = drive, release = stop), speed slider, LiDAR top view,
//    ultrasonic distances, IMU (roll/pitch/yaw), GPS. "Dev" (top right) shows raw values.
//
//  BEFORE YOU FLASH: set BOX_ID in src/core/Config.h to the number of your kit (sticker on box and rover).
//    Everything else follows from it - do not change the naming scheme:
//      Wi-Fi name  CAS-Box-<NN>      e.g. kit 7 -> CAS-Box-07
//      password    CASbox<NN>        e.g. kit 7 -> CASbox07
//      web page    http://192.168.<10+N>.1   e.g. kit 7 -> http://192.168.17.1
//      channel     odd kits 1, even kits 11
//    Your rover is already set up to join exactly this Wi-Fi. A wrong BOX_ID means: your rover
//    does not find your box, or your box takes over another team's network.
//
//  START ORDER: switch on the box first, then the rover (the rover looks for its box only
//    while it boots). Phone: join CAS-Box-<NN>, open the web page.
//
//  ARDUINO IDE: board "ESP32 Dev Module", ESP32 core 3.3.x, default partition scheme.
//    Required libraries and versions: README.md. Serial monitor: 115200 baud.
//
//  FILES
//    box.ino                    this file: setup(), loop()
//    src/core/Config.h          kit number and tunables: the only file to touch before flashing
//    src/core/Logger.*          timeline log: one CSV row per drive tick on LittleFS, docs/logging.md
//    src/sensors/Sensors.*      all sensors behind one object (ultrasonic, LiDAR, IMU, GPS)
//    src/sensors/UsManager.*    5 ultrasonic sensors (HC-SR04), measured in a background task
//    src/sensors/LidarScan.h    LiDAR (LD20) reader with noise filtering
//    src/sensors/ImuManager.*   orientation sensor BNO055 (roll / pitch / yaw in the ISO 8855 vehicle frame)
//    src/sensors/GpsManager.*   GPS module NEO-6M
//    src/comms/RoverLink.*      box -> rover link (sends drive commands, reads battery voltage)
//    src/comms/WebHMI.*         web server and web page (incl. the /log routes)
//    src/display/DisplayManager.*  small OLED on the box
//    Everything under src/ is compiled recursively with the sketch (Arduino sketch rule).
//
//  SAFETY / BEHAVIOUR
//    The rover stops when you release the key, when the page loses focus, when the box gets no
//    update from the phone for 0.4 s, when RoverLink gets no new set point for 0.5 s, and - in
//    the rover itself - after 3 s without a command. Do not remove these stops.
//    The rover firmware must not be changed.
// =============================================================================================

#include "src/core/Config.h"
#include "src/comms/WebHMI.h"
#include "src/comms/RoverLink.h"
#include "src/sensors/Sensors.h"
#include "src/display/DisplayManager.h"
#include "src/core/Logger.h"

// ---- objects ---------------------------------------------------------------------------------
WebHMI hmi;           // web server + page
RoverLink rover;      // connection to the rover
Sensors sensors;      // all box sensors
DisplayManager disp;  // small OLED on the box
Logger logger;        // timeline log on the box flash (/log on the page)
bool oledReady = false;
char boxSsid[16], boxPassword[16];

// One log row per ROVER_PERIOD_MS tick, from the same snapshot the drive decision used.
// state: brain state ("MANUAL" while the phone drives); l, r: what went to rover.setDrive().
void logRow(uint32_t now, const char* state, float l, float r, uint32_t prevLoopUs) {
  Logger::Row row = {};
  row.tMs = now;
  strlcpy(row.state, state, sizeof(row.state));
  row.l = l;
  row.r = r;
  for (uint8_t i = 0; i < 5; i++) row.us[i] = sensors.usCm(i);
  row.yaw = sensors.yawDeg();
  row.pitch = sensors.pitchDeg();
  row.roll = sensors.rollDeg();
  row.imuOk = sensors.imuOk();
  row.lidOk = sensors.lidarOk();
  row.lid[0] = sensors.lidarMinCm(0, 10);    // front
  row.lid[1] = sensors.lidarMinCm(270, 10);  // left
  row.lid[2] = sensors.lidarMinCm(90, 10);   // right
  row.lid[3] = sensors.lidarMinCm(180, 10);  // back
  row.gpsFix = sensors.gpsFix();
  row.lat = sensors.latDeg();
  row.lon = sensors.lonDeg();
  row.volt = rover.voltage();
  row.loopUs = prevLoopUs;
  row.note[0] = 0;  // code may set a note here, e.g. "estop_us_f"; /log?mark= adds its text
  logger.row(row);  // copies the row into a queue; the flash write happens on core 0
}

// OLED: Wi-Fi name, web address, rover state. Started only when the display answers on I2C.
void updateOled() {
  static uint32_t lastTry = 0;
  if (!oledReady) {
    if (millis() - lastTry < 5000) return;
    lastTry = millis();
    if (!i2cPresent(0x3C)) return;
    oledReady = disp.begin();
  }
  String ip = WiFi.softAPIP().toString();
  String r = rover.connected() ? "Rover " + String(rover.voltage(), 1) + "V" : "Rover: search";
  disp.show(boxSsid, ip.c_str(), r.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // naming scheme, derived from BOX_ID only (see header)
  snprintf(boxSsid, sizeof(boxSsid), "CAS-Box-%02d", BOX_ID);
  snprintf(boxPassword, sizeof(boxPassword), "CASbox%02d", BOX_ID);
  uint8_t channel = (BOX_ID % 2) ? 1 : 11;

  hmi.setTitle(String("CAS Box ") + BOX_ID);
  hmi.enableDrivePad(MAX_SPEED);
  hmi.setStateText("Starting ...");
  logger.begin();            // mounts LittleFS, opens /log/run_<n>.csv (formats on first boot)
  hmi.attachLogger(&logger); // /log routes: list, download, clear, mark
  hmi.begin(boxSsid, boxPassword, IPAddress(192, 168, 10 + BOX_ID, 1), channel);

  sensors.begin(LIDAR_OFFSET_DEG);
  rover.begin(ROVER_PERIOD_MS);  // sends the current drive command every ROVER_PERIOD_MS
}

void loop() {
  uint32_t t0 = micros();
  hmi.tick();
  sensors.update();

  // ---- driving -------------------------------------------------------------------------------
  // Manual driving from the phone. For your own (autonomous) behaviour, compute l and r
  // yourself from the sensors, e.g. sensors.usCm(0) = front ultrasonic in cm, and call
  // rover.setDrive(l, r) every loop pass (range -0.5..0.5 per side).
  float l, r;
  uint32_t age;
  hmi.getDrive(l, r, age);
  if (age > PAD_TIMEOUT_MS) l = r = 0;  // key released, page closed or phone gone
  rover.setDrive(l, r);

  // ---- timeline log: one row per rover tick with exactly these l, r and sensor values --------
  uint32_t now = millis();
  static uint32_t lastLog = 0, prevLoopUs = 0;
  if (now - lastLog >= ROVER_PERIOD_MS) {
    lastLog = now;
    logRow(now, "MANUAL", l, r, prevLoopUs);
  }

  // ---- web page and OLED -----------------------------------------------------------------------
  static uint32_t lastSnap = 0, lastUi = 0, lastOled = 0;
  if (now - lastSnap >= SNAPSHOT_MS) {  // sensor snapshot for the page, 4 times per second
    lastSnap = now;
    hmi.setSensorJson(sensors.toJson());
  }
  if (now - lastUi >= UI_MS) {  // status line + values in the "Dev" view
    lastUi = now;
    hmi.setStateText(rover.statusText());
    hmi.showValue("rover.ip", rover.ip().toString());
    hmi.showValue("rover.voltage_V", rover.voltage());
    hmi.showValue("rover.rtt_ms", rover.rttMs());
    hmi.showValue("rover.ok", (double)rover.okCount());
    hmi.showValue("rover.fail", (double)rover.failCount());
    hmi.showValue("sys.heap_kB", ESP.getFreeHeap() / 1024.0);
    hmi.showValue("log", logger.status());
  }
  if (now - lastOled >= OLED_MS) {
    lastOled = now;
    updateOled();
  }

  // ---- load report on Serial every 10 s (loop time, free memory, sensor snapshot) -------------
  static uint32_t lastRep = 0, n = 0, maxUs = 0;
  static uint64_t sumUs = 0;
  uint32_t dt = micros() - t0;
  prevLoopUs = dt;  // loop_us column of the next log row
  n++; sumUs += dt; if (dt > maxUs) maxUs = dt;
  if (now - lastRep >= REPORT_MS) {
    Serial.printf("[sys] loop avg %.2f ms max %.1f ms, heap %u kB, rover %s, log %s\n",
                  sumUs / 1000.0 / n, maxUs / 1000.0, ESP.getFreeHeap() / 1024,
                  rover.connected() ? "ok" : "searching", logger.status().c_str());
    Serial.printf("[sys] %s\n", sensors.toJson().c_str());
    lastRep = now; n = 0; sumUs = 0; maxUs = 0;
  }
  delay(2);
}
