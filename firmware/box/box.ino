// =============================================================================================
//  CAS Box - sensor box on the WAVE ROVER, kit 6
//  Based on the CAS starter program 1.0 (2026-10-15); autonomous run ported from the
//  CampusLogger sketch of 26.10.2026 ("passed open sea") into src/nav/.
// =============================================================================================
//
//  WHAT IT DOES
//    The box ESP32 is the "brain" of your robot. It opens its own Wi-Fi, serves a web page for
//    your phone, reads all box sensors and drives the WAVE ROVER underneath it over Wi-Fi.
//
//      phone ──Wi-Fi──► box (this program): web page, sensors ──Wi-Fi──► rover (motors)
//
//    Web page: autonomous run card (start / stop / safety stops / save IMU calibration), drive
//    pad (hold a key = drive, release = stop), speed slider, LiDAR top view, ultrasonic distances,
//    IMU (roll/pitch/yaw), GPS. "Dev" (top right) shows raw values and the log link.
//    Two ways to drive:
//      - by hand with the drive pad (phase 0/1: characterisation, the log records everything)
//      - autonomous run (START RUN): straight along the pole row, count the poles on the right,
//        turn right 90 deg after the last one, drive straight on. docs/autonomous_run.md
//
//  BEFORE YOU FLASH: set BOX_ID in src/core/Config.h to the number of your kit (sticker on box and rover).
//    Everything else follows from it - do not change the naming scheme:
//      Wi-Fi name  CAS-Box-<NN>      e.g. kit 7 -> CAS-Box-07
//      password    CASbox<NN>        e.g. kit 7 -> CASbox07
//      web page    http://192.168.<10+N>.1   e.g. kit 7 -> http://192.168.17.1
//      channel     odd kits 1, even kits 11
//    Your rover is already set up to join exactly this Wi-Fi. A wrong BOX_ID means: your rover
//    does not find your box, or your box takes over another team's network.
//    Change FW_VERSION (Config.h) with every flash: the page shows it.
//
//  START ORDER: switch on the box first, then the rover (the rover looks for its box only
//    while it boots). Phone: join CAS-Box-<NN>, open the web page.
//
//  ARDUINO IDE: board "ESP32 Dev Module", ESP32 core 3.3.x, default partition scheme.
//    Required libraries and versions: README.md. Serial monitor: 115200 baud.
//    Later updates also over Wi-Fi: Sketch -> Export Compiled Binary, page -> "update over Wi-Fi".
//
//  FILES
//    box.ino                    this file: setup(), loop() - wiring only, no behaviour
//    src/core/Config.h          kit number and tunables: the only file to touch before flashing
//    src/core/Logger.*          timeline log: one CSV row per control step on LittleFS, docs/logging.md
//    src/nav/NavConfig.h        tuning of the autonomous run (speeds, gains, poles, wall, turn)
//    src/nav/Navigator.*        autonomous run: run state, phases, heading hold, turn
//    src/nav/LineTracker.*      virtual line: odometry, pole counting + drift, wall alignment
//    src/nav/SafetyMonitor.*    obstacle pause, cliff check
//    src/nav/Perception.*       pole / wall / lane detection from one LiDAR scan
//    src/nav/NavMath.*          angle difference, median, circle fit
//    src/sensors/Sensors.*      all sensors behind one object (ultrasonic, LiDAR, IMU, GPS)
//    src/sensors/UsManager.*    5 ultrasonic sensors (HC-SR04), measured in a background task
//    src/sensors/LidarScan.h    LiDAR (LD20) reader with noise filtering
//    src/sensors/ImuManager.*   orientation sensor BNO055 (ISO 8855 roll / pitch / yaw, turn rate)
//    src/sensors/GpsManager.*   GPS module NEO-6M
//    src/comms/RoverLink.*      box -> rover link (sends drive commands, reads battery voltage)
//    src/comms/WebHMI.*         web server and web page (run card, /log routes, firmware update)
//    src/display/DisplayManager.*  small OLED on the box
//    Everything under src/ is compiled recursively with the sketch (Arduino sketch rule).
//
//  SAFETY / BEHAVIOUR
//    By hand: the rover stops when you release the key, when the page loses focus, when the box
//    gets no update from the phone for 0.4 s, when RoverLink gets no new set point for 0.5 s,
//    and - in the rover itself - after 3 s without a command. Do not remove these stops.
//    Autonomous run: ends with STOP RUN or the pad's stop key, as soon as you press any drive
//    key (override), when the page has not polled for 3 s (link lost), on a cliff (if enabled)
//    and at the time limit (if set). An obstacle in the lane pauses it until the lane is clear.
//    RoverLink's 0.5 s deadman and the rover's 3 s stop stay active in every mode.
//    The rover firmware must not be changed.
// =============================================================================================

#include "src/core/Config.h"
#include "src/comms/WebHMI.h"
#include "src/comms/RoverLink.h"
#include "src/sensors/Sensors.h"
#include "src/display/DisplayManager.h"
#include "src/core/Logger.h"
#include "src/nav/Navigator.h"

// ---- objects ---------------------------------------------------------------------------------
WebHMI webHmi;           // web server + page
RoverLink roverLink;      // connection to the rover
Sensors sensors;      // all box sensors
DisplayManager oledDisplay;  // small OLED on the box
Logger logger;        // timeline log on the box flash (/log on the page)
Navigator navigator;        // autonomous run
bool isOledReady = false;
char boxSsid[16], boxPassword[16];

// One log row per control step with the values the drive decision of this step used.
// driveLeft, driveRight: what went to roverLink.setDrive(). startNewFile: this row starts a new
// run_<n>.csv (START RUN).
void logRow(uint32_t timestampMs, float driveLeft, float driveRight, uint32_t previousLoopUs, bool startNewFile) {
  Logger::Row logEntry = {};
  // ---- v1
  logEntry.timestampMs = timestampMs;
  strlcpy(logEntry.state, navigator.stateName(), sizeof(logEntry.state));  // MANUAL or the run phase
  logEntry.driveLeft = driveLeft;
  logEntry.driveRight = driveRight;
  for (uint8_t i = 0; i < 5; i++) logEntry.ultrasonicCm[i] = sensors.usNowCm(i);
  logEntry.yawDeg = sensors.yawDeg();
  logEntry.pitchDeg = sensors.pitchDeg();
  logEntry.rollDeg = sensors.rollDeg();
  logEntry.isImuOk = sensors.isImuOk();
  logEntry.isLidarOk = sensors.isLidarOk();
  logEntry.lidarSectorCm[0] = sensors.lidarMinCm(0, 10);    // front
  logEntry.lidarSectorCm[1] = sensors.lidarMinCm(270, 10);  // left
  logEntry.lidarSectorCm[2] = sensors.lidarMinCm(90, 10);   // right
  logEntry.lidarSectorCm[3] = sensors.lidarMinCm(180, 10);  // back
  logEntry.hasGpsFix = sensors.hasGpsFix();
  logEntry.latitudeDeg = sensors.latDeg();
  logEntry.longitudeDeg = sensors.lonDeg();
  logEntry.batteryV = roverLink.voltage();
  logEntry.loopUs = previousLoopUs;
  navigator.takeEvent(logEntry.note, sizeof(logEntry.note));  // run events; /log?mark= text is added by the Logger
  // ---- v2: what the autonomous run saw (also while driving by hand)
  const Navigator::Snapshot& navSnapshot = navigator.lastSnapshot();
  logEntry.yawRateDps = navSnapshot.yawRateDps;
  logEntry.headingRefDeg = navSnapshot.headingRefDeg;
  logEntry.crossTrackCm = navSnapshot.crossTrackCm;
  logEntry.imuCalibration = sensors.imuCalPacked();
  logEntry.frontCm = navSnapshot.frontCm;
  logEntry.lidarCorridorCm = navSnapshot.corridorCm;
  logEntry.lidarFrontRightCm = sensors.lidarMinCm(45, 10);
  logEntry.lidarFrontLeftCm = sensors.lidarMinCm(315, 10);
  logEntry.isPaused = navigator.isPaused();
  logEntry.isSafetyEnabled = navigator.isSafetyEnabled();
  logEntry.poleCount = navigator.poleCount();
  logEntry.isPoleSeen = navSnapshot.pole.isFound;
  logEntry.poleXCm = (int16_t)navSnapshot.pole.xCm;
  logEntry.poleYCm = (int16_t)navSnapshot.pole.yCm;
  logEntry.poleDiameterCm = navSnapshot.pole.diameterCm;
  logEntry.poleFitErrorCm = navSnapshot.pole.fitErrorCm;
  logEntry.lastPoleYCm = (int16_t)navigator.lastPoleYCm();
  logEntry.isWallSeen = navSnapshot.wall.isFound;
  logEntry.wallCm = (int16_t)navSnapshot.wall.distanceCm;
  logEntry.wallAngleDeg = navSnapshot.wall.angleDeg;
  logEntry.isWallSteering = navigator.isWallSteering();
  logEntry.cliffBaselineCm = (uint8_t)constrain(navigator.cliffBaselineCm(), 0, 255);
  logger.row(logEntry, startNewFile);  // copies the row into a queue; the flash write happens on core 0
}

// Buttons of the run card, queued by the web server and executed here (loop owns the run state,
// the IMU bus and the log). Returns true if a run was started (the log then opens a new file).
bool handleCommands(uint32_t nowMs, bool isFirmwareUpdating) {
  bool isRunStarted = false;
  WebHMI::Command command;
  while (webHmi.takeCommand(command)) {
    switch (command) {
      case WebHMI::CMD_RUN_START:
        if (isFirmwareUpdating) {
          webHmi.setMessage("Not during a firmware update.");
        } else if (navigator.isRunning()) {
          webHmi.setMessage("Run already active.");
        } else {
          const uint16_t logRows = logger.startRun();  // RAM log: no flash access while driving
          navigator.start(sensors, nowMs);
          isRunStarted = true;
          String message = sensors.isImuOk() ? "Run started." : "Run started WITHOUT IMU - heading hold is blind!";
          message += logRows ? String(" Log in RAM: ") + (logRows / 10) + " s." : String(" Log goes to flash.");
          webHmi.setMessage(message);
        }
        break;
      case WebHMI::CMD_RUN_STOP:
        if (navigator.isRunning()) {
          navigator.stop(Navigator::STOPPED_MANUAL);
          webHmi.setMessage("Run stopped.");
        }
        break;
      case WebHMI::CMD_SAFETY_TOGGLE:
        navigator.toggleSafety();
        webHmi.setMessage(navigator.isSafetyEnabled() ? "Safety stops enabled." : "Safety stops DISABLED.");
        break;
      case WebHMI::CMD_IMU_SAVE:
        if (navigator.isRunning()) webHmi.setMessage("Not while driving - stop the run first.");
        else if (!sensors.isImuOk()) webHmi.setMessage("IMU not found.");
        else if (sensors.saveImuCalibration()) webHmi.setMessage("IMU calibration saved, loaded at every power-on.");
        else webHmi.setMessage("Not saved: all four levels (system, gyro, accel, mag) must be 3.");
        break;
      default:
        break;
    }
  }
  return isRunStarted;
}

// OLED: Wi-Fi name, web address, rover / run state. Started only when the display answers on I2C.
void updateOled() {
  static uint32_t lastInitAttemptMs = 0;
  if (!isOledReady) {
    if (millis() - lastInitAttemptMs < 5000) return;
    lastInitAttemptMs = millis();
    if (!i2cPresent(0x3C)) return;
    isOledReady = oledDisplay.begin();
  }
  String apAddress = WiFi.softAPIP().toString();
  String statusLine;
  if (navigator.isRunning()) {
    statusLine = navigator.isPaused() ? String("PAUSED (obstacle)") : String(navigator.stateName()) + " poles " + navigator.poleCount();
  } else {
    statusLine = roverLink.connected() ? "Rover " + String(roverLink.voltage(), 1) + "V" : "Rover: search";
  }
  oledDisplay.show(boxSsid, apAddress.c_str(), statusLine.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n[box] firmware %s\n", FW_VERSION);

  // naming scheme, derived from BOX_ID only (see header)
  snprintf(boxSsid, sizeof(boxSsid), "CAS-Box-%02d", BOX_ID);
  snprintf(boxPassword, sizeof(boxPassword), "CASbox%02d", BOX_ID);
  uint8_t wifiChannel = (BOX_ID % 2) ? 1 : 11;

  webHmi.setTitle(String("CAS Box ") + BOX_ID);
  webHmi.enableDrivePad(MAX_SPEED);
  webHmi.setStateText("Starting ...");
  logger.begin();                         // mounts LittleFS, opens /log/run_<n>.csv (formats on first boot)
  webHmi.attachLogger(&logger);              // /log routes: list, download, clear, mark
  webHmi.enableFirmwareUpdate(FW_VERSION);   // /update: new firmware over Wi-Fi (refused during a run)
  webHmi.begin(boxSsid, boxPassword, IPAddress(192, 168, 10 + BOX_ID, 1), wifiChannel);

  sensors.begin(LIDAR_OFFSET_DEG, USE_COMPASS);
  roverLink.begin(ROVER_PERIOD_MS);  // sends the current drive command every ROVER_PERIOD_MS
}

void loop() {
  const uint32_t loopStartUs = micros();
  webHmi.tick();
  sensors.update();
  const uint32_t nowMs = millis();

  // ---- firmware update over Wi-Fi: the rover stops, the ultrasonic task gives the Wi-Fi room ----
  static bool isUpdateHandled = false;
  const bool isFirmwareUpdating = webHmi.updateActive();
  if (isFirmwareUpdating && !isUpdateHandled) {
    isUpdateHandled = true;
    navigator.stop(Navigator::STOPPED_MANUAL);
    sensors.suspendUltrasonic();
    webHmi.setStateText("Firmware update running ...");
    Serial.println("[ota] update started: rover stopped, ultrasonic suspended");
  }

  // ---- buttons of the run card -----------------------------------------------------------------
  static bool startNewLogFile = false;  // only when the run log could not get RAM
  if (handleCommands(nowMs, isFirmwareUpdating) && !logger.isRunActive()) startNewLogFile = true;
  webHmi.setUpdateAllowed(!navigator.isRunning());

  // ---- control step every ROVER_PERIOD_MS on a fixed grid (late steps do not add up) ------------
  static uint32_t lastStepGridMs = 0, lastStepTimeMs = 0;
  static float autoDriveLeft = 0, autoDriveRight = 0;
  bool isControlStep = false;
  if (nowMs - lastStepGridMs >= ROVER_PERIOD_MS) {
    lastStepGridMs = (nowMs - lastStepGridMs > 3 * ROVER_PERIOD_MS) ? nowMs : lastStepGridMs + ROVER_PERIOD_MS;  // stall: restart grid
    const float stepS = constrain((nowMs - lastStepTimeMs) / 1000.0f, 0.05f, 0.3f);  // real step time in s
    lastStepTimeMs = nowMs;
    isControlStep = true;
    navigator.step(sensors, nowMs, stepS, webHmi.pageAgeMs(), autoDriveLeft, autoDriveRight);  // l = r = 0 outside a run
  }

  // ---- driving -------------------------------------------------------------------------------
  // By hand from the phone pad, or the autonomous run. roverLink.setDrive() every loop pass
  // (range -0.5..0.5 per side); RoverLink sends it every ROVER_PERIOD_MS.
  float driveLeft, driveRight;
  uint32_t padCommandAgeMs;
  webHmi.getDrive(driveLeft, driveRight, padCommandAgeMs);
  if (padCommandAgeMs > PAD_TIMEOUT_MS) driveLeft = driveRight = 0;  // key released, page closed or phone gone
  // a drive key pressed after the run started takes over: the human always wins
  if (navigator.isRunning() && (driveLeft != 0 || driveRight != 0) && padCommandAgeMs < nowMs - navigator.startedAtMs()) {
    navigator.stop(Navigator::STOPPED_OVERRIDE);
    webHmi.setMessage("Run ended: drive pad override.");
  }
  if (navigator.isRunning()) {
    driveLeft = autoDriveLeft;
    driveRight = autoDriveRight;
  }
  if (isFirmwareUpdating) driveLeft = driveRight = 0;
  roverLink.setDrive(driveLeft, driveRight);

  // run ended (any reason): the rover stands, the RAM log may go to flash now
  static bool wasRunning = false;
  if (wasRunning && !navigator.isRunning()) logger.endRun();
  wasRunning = navigator.isRunning();

  // ---- timeline log: one row per control step with exactly these l, r and inputs ---------------
  // Standing without a run: one row per LOG_IDLE_PERIOD_MS (events and marks are written at once).
  static uint32_t previousLoopUs = 0, lastLogRowMs = 0, lastMotionMs = 0;
  if (driveLeft != 0 || driveRight != 0) lastMotionMs = nowMs;
  if (isControlStep) {
    const bool isLogActive = navigator.isRunning() || nowMs - lastMotionMs < 2000 || navigator.hasEvent() || startNewLogFile;
    if (isLogActive || nowMs - lastLogRowMs >= LOG_IDLE_PERIOD_MS) {
      lastLogRowMs = nowMs;
      logRow(nowMs, driveLeft, driveRight, previousLoopUs, startNewLogFile);
      startNewLogFile = false;
    }
  }

  // ---- web page and OLED -----------------------------------------------------------------------
  static uint32_t lastSnapshotMs = 0, lastUiUpdateMs = 0, lastOledUpdateMs = 0;
  if (nowMs - lastSnapshotMs >= SNAPSHOT_MS) {  // sensor snapshot + run card for the page, 4 times per second
    lastSnapshotMs = nowMs;
    webHmi.setSensorJson(sensors.toJson());
    webHmi.setRunJson(navigator.toJson());
  }
  if (nowMs - lastUiUpdateMs >= UI_MS) {  // status line + values in the "Dev" view
    lastUiUpdateMs = nowMs;
    if (!isFirmwareUpdating) webHmi.setStateText(roverLink.statusText());
    webHmi.showValue("rover.ip", roverLink.ip().toString());
    webHmi.showValue("rover.voltage_V", roverLink.voltage());
    webHmi.showValue("rover.rtt_ms", roverLink.rttMs());
    webHmi.showValue("rover.ok", (double)roverLink.okCount());
    webHmi.showValue("rover.fail", (double)roverLink.failCount());
    webHmi.showValue("sys.heap_kB", ESP.getFreeHeap() / 1024.0);
    webHmi.showValue("sys.heapMin_kB", ESP.getMinFreeHeap() / 1024.0);
    webHmi.showValue("log", logger.status());
    webHmi.showValue("nav", navigator.statusText());
  }
  if (nowMs - lastOledUpdateMs >= OLED_MS) {
    lastOledUpdateMs = nowMs;
    updateOled();
  }

  // ---- load report on Serial every 10 s (loop time, free memory, sensor snapshot) -------------
  static uint32_t lastReportMs = 0, loopCount = 0, maxLoopUs = 0;
  static uint64_t sumLoopUs = 0;
  const uint32_t loopDurationUs = micros() - loopStartUs;
  previousLoopUs = loopDurationUs;  // loop_us column of the next log row
  loopCount++; sumLoopUs += loopDurationUs; if (loopDurationUs > maxLoopUs) maxLoopUs = loopDurationUs;
  if (nowMs - lastReportMs >= REPORT_MS && !navigator.isRunning()) {  // no long prints while driving
    Serial.printf("[sys] loop avg %.2f ms max %.1f ms, heap %u kB, rover %s, log %s, nav %s\n",
                  sumLoopUs / 1000.0 / loopCount, maxLoopUs / 1000.0, ESP.getFreeHeap() / 1024,
                  roverLink.connected() ? "ok" : "searching", logger.status().c_str(), navigator.statusText().c_str());
    Serial.printf("[sys] %s\n", sensors.toJson().c_str());
    lastReportMs = nowMs; loopCount = 0; sumLoopUs = 0; maxLoopUs = 0;
  }
  delay(2);
}
