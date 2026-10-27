#pragma once
// Logger - timeline log on the box flash (LittleFS), row format v2 of docs/logging.md
// (v1 columns unchanged, the autonomous-run columns appended after "note").
//
// loop() hands one Row per RoverLink tick to row(); that only copies the row into a queue (a few
// microseconds). A low-priority task on core 0 formats the rows as CSV text and appends them to
// /log/run_<n>.csv every 500 ms or every 16 rows, so loop() never waits for the flash.
// One file per boot and one more per started run (row(entry, true)), n = highest existing number + 1.
// First line "# log v2", then the header row, column order exactly as in docs/logging.md. The logger stops at 90 % partition fill and says so
// in status(). The web routes (/log, see WebHMI) list, download, delete and mark.
//
// Autonomous run: like the CampusLogger sketch, nothing touches the flash while the rover drives on
// its own (every flash erase / write / read freezes both CPU cores for up to ~50 ms, which shifts
// the control step and the rover commands). startRun() keeps the run's rows in RAM (as many as the
// free heap allows above kRunHeapReserve, at most kRunMaxRows = 170 s); endRun() hands them to the
// flush task, which then opens the next run_<n>.csv and writes them. A full RAM buffer drops rows,
// the driving is not affected (as in CampusLogger).
//
// Threads: row() and mark() only touch a queue and a short spinlock. Every file access (flush
// task, web handlers) goes through one mutex, and no file is ever open twice at the same time
// (the flush opens, appends and closes on every flush), which LittleFS does not allow.
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

class Logger {
public:
  static const uint8_t kFormatVersion = 2;
  static const uint8_t kMaxFiles = 32;          // files the listing can show

  // One log row = the values the drive decision used in this tick (docs/logging.md, v2).
  struct Row {
    // ---- v1 ----
    uint32_t timestampMs;        // millis() of the tick
    char state[16];      // brain state, e.g. "MANUAL"
    float driveLeft, driveRight;          // what was sent to roverLink.setDrive() this tick, -0.5..0.5
    int16_t ultrasonicCm[5];       // ultrasonic cm: front, left, right, back, front angled (300 = nothing)
    float yawDeg, pitchDeg, rollDeg;  // deg, ISO 8855
    bool isImuOk, isLidarOk;
    uint16_t lidarSectorCm[4];     // LiDAR sector minima cm: front, left, right, back (0 = none)
    bool hasGpsFix;
    double latitudeDeg, longitudeDeg;     // deg
    float batteryV;          // rover battery V (0 = unknown)
    uint32_t loopUs;     // duration of the previous loop pass
    char note[24];       // free text, empty normally; "," and line breaks are replaced
    // ---- v2: autonomous run (src/nav/Navigator), also filled while driving by hand ----
    float yawRateDps;       // deg/s, + = turning left
    float headingRefDeg;    // heading the controller steered to (deg)
    float crossTrackCm;    // cm off the virtual line, + = left
    uint16_t imuCalibration;     // 4 digits sys/gyro/accel/mag, 3333 = fully calibrated
    int16_t frontCm;     // value the obstacle check used, 0 = lane clear
    uint16_t lidarCorridorCm;    // LiDAR lane (corridor) check, 0 = clear
    uint16_t lidarFrontRightCm, lidarFrontLeftCm;  // LiDAR sector minima front-right 45 / front-left 315 deg (0 = none)
    bool isPaused;         // obstacle pause active
    bool isSafetyEnabled;         // safety stops enabled
    uint8_t poleCount;       // poles counted in this run
    bool isPoleSeen;       // a pole was detected in this scan:
    int16_t poleXCm, poleYCm;   //   centre, x forward (- = behind), y right (cm)
    float poleDiameterCm, poleFitErrorCm;   //   fitted diameter, circle fit error (cm)
    int16_t lastPoleYCm;   // side distance of the last counted pole (drift ruler)
    bool isWallSeen;       // a left wall was detected in this scan:
    int16_t wallCm;      //   distance (cm)
    float wallAngleDeg;       //   our angle to it, + = pointing left of it
    bool isWallSteering;      // the wall steered in this step
    uint8_t cliffBaselineCm;   // ground baseline of the cliff check (cm)
  };

  struct FileInfo {
    char name[24];       // run_<n>.csv (without the /log/ directory)
    size_t size;
  };

  enum State : uint8_t { NO_FS, LOGGING, FULL, FAILED };  // FAILED = repeated write errors

  // Mounts LittleFS (formats it on the first boot), opens /log/run_<n>.csv with the header and
  // starts the flush task. Returns false without a usable filesystem (row() then drops rows).
  bool begin();

  bool ready() const { return _state != NO_FS; }
  State state() const { return _state; }

  // Queues one row. Never blocks; a full queue drops the row (counted in status()).
  // A pending mark() text is merged into the note column of this row.
  // startNewFile = true: this row starts a new run_<n>.csv (e.g. START RUN); the rows before it are
  // still written to the old file. Ignored while the logger is full or stopped.
  void row(const Row& entry, bool startNewFile = false);

  // Sets the note column of the next row (from any task, e.g. the web server).
  void mark(const char* text);

  // Run in RAM (loop() only). startRun() returns the capacity in rows (0 = no RAM: the run is then
  // written to flash directly into a new file, as manual driving). endRun() after the run ends.
  uint16_t startRun();
  void endRun();
  bool isRunActive() const { return _isRunActive; }           // rows go to RAM, no flash access
  bool isRunWritePending() const { return _isRunWritePending; }  // flush task writes the last run

  // One short line for the page, e.g. "run_3.csv 118kB 9%" or "FULL run_3.csv 1260kB".
  String status() const;
  const char* fileName() const { return _fileName; }   // current file, e.g. "run_3.csv"
  size_t usedBytes() const { return _usedBytes; }       // partition fill, refreshed every 5 s
  size_t totalBytes() const { return _totalBytes; }
  uint32_t droppedRows() const { return _droppedRowCount; }

  // ---- for the web routes (called from the server task) ----------------------------------
  uint8_t files(FileInfo* fileList, uint8_t maxFiles);       // lists /log, newest number last
  static bool validName(const char* candidate);         // "run_<digits>.csv" only (no paths)
  // Reads maxLen bytes of a file from offset index; 0 = end of file / no such file,
  // (size_t)-1 = flash busy, try again.
  size_t readChunk(const char* requestedName, uint8_t* buffer, size_t maxLength, size_t offset);
  bool clear();                                    // deletes all logs, restarts the current file

private:
  enum ItemKind : uint8_t { ITEM_ROW, ITEM_RUN_WRITE };
  struct Item {          // queue element
    Row row;
    bool startsNewFile;
    uint8_t kind;        // ITEM_RUN_WRITE: write the RAM run to a new file (row unused)
  };

  // One row of the RAM run buffer: the Row packed into small integers (x10 / x100 = the decimals
  // the CSV prints), 80 bytes instead of ~170. The note text lives in _runNotes.
  struct RunRow {
    uint32_t timestampMs;
    float latitudeDeg, longitudeDeg;
    uint16_t loopUs;
    uint16_t noteOffset;                 // into _runNotes, kNoNote = none
    int16_t ultrasonicCm[5];
    int16_t yawDeg10, pitchDeg10, rollDeg10;
    uint16_t lidarSectorCm[4];
    int16_t yawRateDps10, headingRefDeg10, crossTrackCm10;
    uint16_t imuCalibration;
    int16_t frontCm;
    uint16_t lidarCorridorCm, lidarFrontRightCm, lidarFrontLeftCm;
    int16_t poleXCm, poleYCm, poleDiameterCm10, poleFitErrorCm10, lastPoleYCm;
    int16_t wallCm, wallAngleDeg10;
    uint16_t batteryV100;
    int8_t driveLeft100, driveRight100;
    uint8_t stateIndex;                  // kStateNames
    uint8_t poleCount;
    uint8_t cliffBaselineCm;
    uint8_t flags;                       // RunFlag bits
  };
  enum RunFlag : uint8_t { FLAG_IMU = 1, FLAG_LIDAR = 2, FLAG_GPS = 4, FLAG_PAUSED = 8, FLAG_SAFETY = 16,
                           FLAG_POLE = 32, FLAG_WALL = 64, FLAG_WALL_STEER = 128 };

  void _storeRunRow(const Row& entry);             // loop(): pack into the RAM buffer
  void _unpackRunRow(const RunRow& packed, Row& entry) const;
  void _writeRun();                                // flush task: new file + all RAM rows
  void _freeRun();

  static void _taskEntry(void* self);
  void _task();
  void _rotate();                                  // flush task: start the next run_<n>.csv
  void _format(const Row& entry);                      // appends one CSV line to _textBuffer
  void _flush();                                   // appends _textBuffer to the file
  void _checkFill();
  bool _writeHeader();
  int _nextRunNumber();
  static void _copyField(char* destination, size_t capacity, const char* source);  // strips , " and line breaks

  static const size_t kQueueRows = 32;
  static const size_t kFlushRows = 16;
  static const uint32_t kFlushMs = 500;
  static const uint32_t kFillCheckMs = 5000;
  static const size_t kMaxLine = 400;
  static const uint8_t kMaxWriteErrors = 5;
  static const uint16_t kRunChunkRows = 100;       // RAM run buffer is allocated in blocks of 100 rows
  static const uint16_t kRunMaxRows = 1700;        // 170 s at 10 Hz (CampusLogger MAX_LOG_ENTRIES)
  static const uint32_t kRunHeapReserve = 80000;   // bytes kept free for Wi-Fi, web server, rover link
  static const size_t kRunNoteBytes = 2048;
  static const uint16_t kNoNote = 0xFFFF;

  volatile State _state = NO_FS;
  char _fileName[24] = "";                   // run_<n>.csv
  char _filePath[32] = "";                   // /log/run_<n>.csv
  QueueHandle_t _rowQueue = nullptr;
  SemaphoreHandle_t _fsMutex = nullptr;      // guards every LittleFS access
  TaskHandle_t _flushTaskHandle = nullptr;

  // RAM run buffer: filled by loop() while _isRunActive, read by the flush task after endRun()
  RunRow* _runChunks[kRunMaxRows / kRunChunkRows] = {};
  uint16_t _runCapacity = 0;
  volatile uint16_t _runRowCount = 0;
  uint32_t _runDroppedRows = 0;
  char _runNotes[kRunNoteBytes];
  size_t _runNotesLength = 0;
  volatile bool _isRunActive = false;
  volatile bool _isRunWritePending = false;

  // mark(): written by any task, consumed by row() (loop)
  mutable portMUX_TYPE _markMux = portMUX_INITIALIZER_UNLOCKED;
  char _markText[24] = "";
  bool _markPending = false;

  // flush task only
  char _textBuffer[4096];
  size_t _textLength = 0;
  uint16_t _pendingRows = 0;
  uint32_t _lastFlushMs = 0, _lastFillCheckMs = 0;
  uint8_t _writeErrorCount = 0;

  // written by the task / web handlers, read by loop(); 32-bit values are atomic on the ESP32
  volatile uint32_t _fileBytes = 0;          // bytes written to the current file this boot
  volatile uint32_t _droppedRowCount = 0;        // rows lost (queue full or logger stopped)
  volatile uint32_t _usedBytes = 0, _totalBytes = 0;
};
