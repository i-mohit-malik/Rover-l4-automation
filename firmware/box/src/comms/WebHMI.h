#pragma once
// WebHMI - Wi-Fi access point, web server and web page of the box.
//
// Page (phone): state line, autonomous-run card (start / stop / safety / IMU calibration),
// drive pad (hold a key = drive, release = stop), speed slider, LiDAR top view + ultrasonic,
// IMU/GPS, and a "Dev" view with named values and the loop time.
// The page polls /values every 250 ms (one request at a time, 1 s timeout; a banner shows when
// the box stops answering); drive keys call /drive every 150 ms while held.
//
// Routes:  /                 the page
//          /values           JSON: state, dev values, loop time, sensor snapshot
//          /drive?l=..&r=..&id=..&n=..  wheel speeds from the drive pad; id = page instance,
//                            n = rising number (older commands that arrive late are ignored)
//          /log              log page: file list (run_<n>.csv, size), mark field, clear button
//          /log?get=<name>   downloads one CSV (text/csv)
//          /log?clear=1      deletes all logs, the current run restarts with its header
//          /log?mark=<text>  writes <text> into the note column of the next log row
//          (the /log routes exist only after attachLogger(), see docs/logging.md)
//          /run/start  /run/stop  /run/safety  /imu/save
//                            buttons of the run card; queued as Command for loop() (takeCommand)
//          /update           firmware update over Wi-Fi (GET page, POST the .bin); only after
//                            enableFirmwareUpdate() and only while setUpdateAllowed(true)
//
// Threads: the web server runs in its own task. Everything the page reads is guarded by a mutex,
// the drive set point by a spinlock, so loop() and the server never see half-written data.
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

class Logger;

class WebHMI {
public:
  static const uint8_t kMaxValues = 16;  // named values in the "Dev" view

  // Starts the access point (password min. 8 characters) and the web server.
  void begin(const char* ssid, const char* password, IPAddress ip, uint8_t channel);

  // Adds the /log routes for this logger. Call before begin().
  void attachLogger(Logger* logger) { _logger = logger; }

  // Adds /update (firmware upload over Wi-Fi). version is shown on the page. Call before begin().
  void enableFirmwareUpdate(const char* version) { _firmwareVersion = version; _otaEnabled = true; }
  void setUpdateAllowed(bool allowed) { _otaAllowed = allowed; }  // loop(): false while a run is active
  bool updateActive() const { return _otaActive; }  // an upload is running: stop driving

  // Buttons of the run card. The server task only queues them; loop() executes them, because
  // they touch the run state, the IMU (I2C) and the log, which belong to loop().
  enum Command : uint8_t { CMD_NONE, CMD_RUN_START, CMD_RUN_STOP, CMD_SAFETY_TOGGLE, CMD_IMU_SAVE };
  bool takeCommand(Command& command);               // next queued command, false if none

  void setRunJson(const String& json);          // run card, see Navigator::toJson()
  void setMessage(const String& text);          // result line under the run buttons

  // Time since the page last polled /values (UINT32_MAX = never): the link watchdog of the run.
  uint32_t pageAgeMs() const;

  void setTitle(const String& title);           // page title, set before the first page load
  void enableDrivePad(float maxSpeed);          // shows the drive pad, slider max = maxSpeed
  void setStateText(const String& text);        // state line at the top of the page
  void setSensorJson(const String& json);       // sensor snapshot, see Sensors::toJson()
  void showValue(const char* name, double value);         // named value in the "Dev" view
  void showValue(const char* name, const String& value);

  // Latest drive command from the page; ageMs = time since the page last sent it.
  void getDrive(float& left, float& right, uint32_t& commandAgeMs) const;

  // Call once per loop pass: measures the loop time for the "Dev" view and restarts the box after
  // a firmware upload (or after an upload that stalled).
  void tick();

private:
  struct Value {
    char name[24];
    char text[32];
    bool inUse;
  };

  void _lock();  // takes _dataMutex (release with xSemaphoreGive)
  void _setValue(const char* name, const String& text);
  String _buildPage() const;
  String _buildJson() const;
  void _handleLog(AsyncWebServerRequest* request);
  String _buildLogPage();
  void _queueCommand(AsyncWebServerRequest* request, Command command);
  void _handleUpload(AsyncWebServerRequest* request, const String& fileName, size_t offset, uint8_t* data,
                     size_t length, bool isFinal);

  AsyncWebServer* _server = nullptr;
  Logger* _logger = nullptr;          // optional, set by attachLogger()
  String _title = "CAS Box";
  bool _isDrivePadEnabled = false;
  float _driveMaxSpeed = 0.5f;

  SemaphoreHandle_t _dataMutex = nullptr;   // guards _state, _sensorJson, _runJson, _message, _values, _cycle*
  String _state = "Starting ...";
  String _sensorJson;
  String _runJson;
  String _message;

  QueueHandle_t _commandQueue = nullptr;  // Command, server task -> loop()
  volatile uint32_t _lastPollMs = 0;  // last /values request (0 = never)

  // firmware update (server task writes, loop reads; bools / 32-bit values are atomic)
  const char* _firmwareVersion = "";
  bool _otaEnabled = false;
  volatile bool _otaAllowed = false;
  volatile bool _otaActive = false;     // upload accepted and running
  AsyncWebServerRequest* volatile _otaRequest = nullptr;  // the accepted upload (one at a time)
  volatile uint32_t _otaLastDataMs = 0;
  volatile uint32_t _restartAtMs = 0;   // != 0: tick() restarts the box at this time
  Value _values[kMaxValues] = {};
  float _cycleMinMs = 0, _cycleMaxMs = 0, _cycleAvgMs = 0;

  mutable portMUX_TYPE _driveMux = portMUX_INITIALIZER_UNLOCKED;  // guards the drive set point
  float _driveLeft = 0, _driveRight = 0;
  uint32_t _driveReceivedMs = 0;            // 0 = never received
  uint32_t _drivePageId = 0, _driveSequence = 0;  // newest accepted command (page id, number)

  // loop time statistics, loop() only
  uint32_t _cycleLastUs = 0, _cycleWindowStartMs = 0, _cycleMinUs = 0, _cycleMaxUs = 0, _cycleCount = 0;
  uint64_t _cycleSumUs = 0;
};
