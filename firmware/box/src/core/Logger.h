#pragma once
// Logger - timeline log on the box flash (LittleFS), row format v1 of docs/logging.md.
//
// loop() hands one Row per RoverLink tick to row(); that only copies the row into a queue (a few
// microseconds). A low-priority task on core 0 formats the rows as CSV text and appends them to
// /log/run_<n>.csv every 500 ms or every 16 rows, so loop() never waits for the flash.
// One file per boot, n = highest existing number + 1. First line "# log v1", then the header row,
// column order exactly as in docs/logging.md. The logger stops at 90 % partition fill and says so
// in status(). The web routes (/log, see WebHMI) list, download, delete and mark.
//
// Threads: row() and mark() only touch a queue and a short spinlock. Every file access (flush
// task, web handlers) goes through one mutex, and no file is ever open twice at the same time
// (the flush opens, appends and closes on every flush), which LittleFS does not allow.
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

class Logger {
public:
  static const uint8_t kFormatVersion = 1;
  static const uint8_t kMaxFiles = 32;          // files the listing can show

  // One log row = the snapshot the drive decision used in this tick (docs/logging.md, v1).
  struct Row {
    uint32_t tMs;        // millis() of the tick
    char state[16];      // brain state, e.g. "MANUAL"
    float l, r;          // what was sent to rover.setDrive() this tick, -0.5..0.5
    int16_t us[5];       // ultrasonic cm: front, left, right, back, front angled (300 = nothing)
    float yaw, pitch, roll;  // deg, ISO 8855
    bool imuOk, lidOk;
    uint16_t lid[4];     // LiDAR sector minima cm: front, left, right, back (0 = none)
    bool gpsFix;
    double lat, lon;     // deg
    float volt;          // rover battery V (0 = unknown)
    uint32_t loopUs;     // duration of the previous loop pass
    char note[24];       // free text, empty normally; "," and line breaks are replaced
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
  void row(const Row& r);

  // Sets the note column of the next row (from any task, e.g. the web server).
  void mark(const char* text);

  // One short line for the page, e.g. "run_3.csv 118kB 9%" or "FULL run_3.csv 1260kB".
  String status() const;
  const char* fileName() const { return _name; }   // current file, e.g. "run_3.csv"
  size_t usedBytes() const { return _used; }       // partition fill, refreshed every 5 s
  size_t totalBytes() const { return _total; }
  uint32_t droppedRows() const { return _dropped; }

  // ---- for the web routes (called from the server task) ----------------------------------
  uint8_t files(FileInfo* out, uint8_t max);       // lists /log, newest number last
  static bool validName(const char* name);         // "run_<digits>.csv" only (no paths)
  // Reads maxLen bytes of a file from offset index; 0 = end of file / no such file,
  // (size_t)-1 = flash busy, try again.
  size_t readChunk(const char* name, uint8_t* buf, size_t maxLen, size_t index);
  bool clear();                                    // deletes all logs, restarts the current file

private:
  static void _taskEntry(void* self);
  void _task();
  void _format(const Row& r);                      // appends one CSV line to _buf
  void _flush();                                   // appends _buf to the file
  void _checkFill();
  bool _writeHeader();
  int _nextRunNumber();
  static void _copyField(char* dst, size_t n, const char* src);  // strips , " and line breaks

  static const size_t kQueueRows = 64;
  static const size_t kFlushRows = 16;
  static const uint32_t kFlushMs = 500;
  static const uint32_t kFillCheckMs = 5000;
  static const size_t kMaxLine = 256;
  static const uint8_t kMaxWriteErrors = 5;

  volatile State _state = NO_FS;
  char _name[24] = "";                   // run_<n>.csv
  char _path[32] = "";                   // /log/run_<n>.csv
  QueueHandle_t _queue = nullptr;
  SemaphoreHandle_t _mtx = nullptr;      // guards every LittleFS access
  TaskHandle_t _taskHandle = nullptr;

  // mark(): written by any task, consumed by row() (loop)
  mutable portMUX_TYPE _markMux = portMUX_INITIALIZER_UNLOCKED;
  char _markText[24] = "";
  bool _markPending = false;

  // flush task only
  char _buf[4096];
  size_t _bufLen = 0;
  uint16_t _pendingRows = 0;
  uint32_t _lastFlushMs = 0, _lastFillCheckMs = 0;
  uint8_t _writeErrors = 0;

  // written by the task / web handlers, read by loop(); 32-bit values are atomic on the ESP32
  volatile uint32_t _bytes = 0;          // bytes written to the current file this boot
  volatile uint32_t _dropped = 0;        // rows lost (queue full or logger stopped)
  volatile uint32_t _used = 0, _total = 0;
};
