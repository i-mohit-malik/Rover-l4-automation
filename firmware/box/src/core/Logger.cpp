#include "Logger.h"
#include <stdarg.h>

static const char kLogDir[] = "/log";
// Row format v2, docs/logging.md: fixed column order, add new columns only at the end.
static const char kHeader[] =
  "t_ms,state,l,r,us_f,us_l,us_r,us_b,us_fa,yaw,pitch,roll,imu_ok,lid_ok,"
  "lid_f,lid_l,lid_r,lid_b,gps_fix,lat,lon,volt,loop_us,note,"            // v1 (24)
  "yaw_rate,heading_ref,cross_track,imu_cal,front_cm,lid_corr,lid_fr,lid_fl,paused,safety,"
  "poles,pole_x,pole_y,pole_d,pole_fit,last_pole_y,wall_cm,wall_deg,wall_steer,cliff_base";  // v2 (+20)

static const char* const kStateNames[] = {"MANUAL", "LEG1", "WAIT", "TURN", "SETTLE", "LEG2"};

static int16_t toX10(float value) { return (int16_t)lroundf(constrain(value * 10.0f, -32000.0f, 32000.0f)); }

// printf into buf at len, advancing len; false if it did not fit (len unchanged then).
static bool appendf(char* buffer, size_t capacity, size_t& length, const char* format, ...) {
  if (length >= capacity) return false;
  va_list args;
  va_start(args, format);
  const int written = vsnprintf(buffer + length, capacity - length, format, args);
  va_end(args);
  if (written < 0 || (size_t)written >= capacity - length) return false;
  length += written;
  return true;
}

// ---------------------------------------------------------------------------------------------

bool Logger::begin() {
  _fsMutex = xSemaphoreCreateMutex();
  _rowQueue = xQueueCreate(kQueueRows, sizeof(Item));
  if (!_fsMutex || !_rowQueue) {
    Serial.println("[log] out of memory, logging disabled");
    return false;
  }

  if (!LittleFS.begin(true)) {  // true: format the partition on the first boot
    Serial.println("[log] LittleFS mount failed, logging disabled");
    return false;
  }
  if (!LittleFS.exists(kLogDir)) LittleFS.mkdir(kLogDir);

  const int runNumber = _nextRunNumber();
  snprintf(_fileName, sizeof(_fileName), "run_%d.csv", runNumber);
  snprintf(_filePath, sizeof(_filePath), "%s/%s", kLogDir, _fileName);
  if (!_writeHeader()) {
    Serial.printf("[log] cannot create %s, logging disabled\n", _filePath);
    return false;
  }
  _checkFill();
  _state = _usedBytes >= (uint64_t)_totalBytes * 9 / 10 ? FULL : LOGGING;
  _lastFlushMs = millis();
  Serial.printf("[log] %s, flash %u/%u kB used%s\n", _filePath, _usedBytes / 1024, _totalBytes / 1024,
                _state == FULL ? " - FULL, logging stopped" : "");

  xTaskCreatePinnedToCore(_taskEntry, "logger", 6144, this, 1, &_flushTaskHandle, 0);
  return _state == LOGGING;
}

// Highest run_<n>.csv in /log + 1 (1 on an empty partition).
int Logger::_nextRunNumber() {
  int highestRunNumber = 0;
  File logDir = LittleFS.open(kLogDir);
  if (logDir && logDir.isDirectory()) {
    File entry;
    while ((entry = logDir.openNextFile())) {
      const char* entryName = entry.name();
      const char* baseName = strrchr(entryName, '/');
      baseName = baseName ? baseName + 1 : entryName;
      int runNumber = 0;
      if (sscanf(baseName, "run_%d.csv", &runNumber) == 1 && runNumber > highestRunNumber) highestRunNumber = runNumber;
      entry.close();
    }
    logDir.close();
  }
  return highestRunNumber + 1;
}

// Creates the current file with the version comment and the header row (caller holds _mtx or
// runs before the task exists).
bool Logger::_writeHeader() {
  File file = LittleFS.open(_filePath, FILE_WRITE);
  if (!file) return false;
  file.printf("# log v%d\n%s\n", kFormatVersion, kHeader);
  const size_t fileSize = file.size();
  file.close();
  _fileBytes = fileSize;
  return true;
}

// ---- loop() side -----------------------------------------------------------------------------

void Logger::row(const Row& entry, bool startNewFile) {
  if (!_rowQueue) return;
  Item item;
  item.row = entry;
  item.startsNewFile = startNewFile;
  item.kind = ITEM_ROW;
  Row& queuedRow = item.row;
  _copyField(queuedRow.state, sizeof(queuedRow.state), entry.state);
  _copyField(queuedRow.note, sizeof(queuedRow.note), entry.note);

  // merge a pending mark() into the note column: "code_note;mark" or just "mark"
  char pendingMark[sizeof(_markText)] = "";
  portENTER_CRITICAL(&_markMux);
  if (_markPending) {
    strlcpy(pendingMark, _markText, sizeof(pendingMark));
    _markPending = false;
  }
  portEXIT_CRITICAL(&_markMux);
  if (pendingMark[0]) {
    if (queuedRow.note[0]) strlcat(queuedRow.note, ";", sizeof(queuedRow.note));
    strlcat(queuedRow.note, pendingMark, sizeof(queuedRow.note));
  }

  if (_isRunActive) {  // autonomous run: RAM only, no flash access while driving
    _storeRunRow(queuedRow);
    return;
  }
  if (_state != LOGGING || xQueueSend(_rowQueue, &item, 0) != pdTRUE) _droppedRowCount = _droppedRowCount + 1;
}

// ---- autonomous run in RAM ---------------------------------------------------------------------

uint16_t Logger::startRun() {
  if (!_rowQueue || _state != LOGGING || _isRunActive) return 0;
  if (_isRunWritePending) {
    Serial.println("[log] last run is still being written: this run goes to flash directly");
    return 0;
  }
  // blocks of 100 rows from the heap while enough stays free (as CampusLogger's log buffer)
  _runCapacity = 0;
  while (_runCapacity < kRunMaxRows &&
         ESP.getFreeHeap() > kRunHeapReserve + kRunChunkRows * sizeof(RunRow)) {
    RunRow* chunk = (RunRow*)malloc(kRunChunkRows * sizeof(RunRow));
    if (!chunk) break;
    _runChunks[_runCapacity / kRunChunkRows] = chunk;
    _runCapacity += kRunChunkRows;
  }
  _runRowCount = 0;
  _runDroppedRows = 0;
  _runNotesLength = 0;
  _isRunActive = _runCapacity > 0;
  if (_isRunActive) {
    Serial.printf("[log] run in RAM: room for %u rows (%u s), no flash access until the run ends\n",
                  _runCapacity, _runCapacity / 10);
  } else {
    Serial.println("[log] no RAM for the run: written to flash while driving");
  }
  return _runCapacity;
}

void Logger::endRun() {
  if (!_isRunActive) return;
  _isRunActive = false;
  _isRunWritePending = true;
  Item item;
  item.kind = ITEM_RUN_WRITE;
  item.startsNewFile = false;
  // the queue is empty here (nothing was queued during the run), so this never waits in practice
  if (xQueueSend(_rowQueue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.println("[log] could not queue the run write, run log lost");
    _freeRun();
    _isRunWritePending = false;
  }
}

void Logger::_storeRunRow(const Row& entry) {
  if (_runRowCount >= _runCapacity) {
    _runDroppedRows++;
    return;
  }
  RunRow& packed = _runChunks[_runRowCount / kRunChunkRows][_runRowCount % kRunChunkRows];
  packed.timestampMs = entry.timestampMs;
  packed.latitudeDeg = (float)entry.latitudeDeg;
  packed.longitudeDeg = (float)entry.longitudeDeg;
  packed.loopUs = (uint16_t)(entry.loopUs > 65535 ? 65535 : entry.loopUs);
  packed.noteOffset = kNoNote;
  const size_t noteLength = strlen(entry.note);
  if (noteLength && _runNotesLength + noteLength + 1 <= kRunNoteBytes) {
    memcpy(_runNotes + _runNotesLength, entry.note, noteLength + 1);
    packed.noteOffset = (uint16_t)_runNotesLength;
    _runNotesLength += noteLength + 1;
  }
  for (int i = 0; i < 5; i++) packed.ultrasonicCm[i] = entry.ultrasonicCm[i];
  packed.yawDeg10 = toX10(entry.yawDeg);
  packed.pitchDeg10 = toX10(entry.pitchDeg);
  packed.rollDeg10 = toX10(entry.rollDeg);
  for (int i = 0; i < 4; i++) packed.lidarSectorCm[i] = entry.lidarSectorCm[i];
  packed.yawRateDps10 = toX10(entry.yawRateDps);
  packed.headingRefDeg10 = toX10(entry.headingRefDeg);
  packed.crossTrackCm10 = toX10(entry.crossTrackCm);
  packed.imuCalibration = entry.imuCalibration;
  packed.frontCm = entry.frontCm;
  packed.lidarCorridorCm = entry.lidarCorridorCm;
  packed.lidarFrontRightCm = entry.lidarFrontRightCm;
  packed.lidarFrontLeftCm = entry.lidarFrontLeftCm;
  packed.poleXCm = entry.poleXCm;
  packed.poleYCm = entry.poleYCm;
  packed.poleDiameterCm10 = toX10(entry.poleDiameterCm);
  packed.poleFitErrorCm10 = toX10(entry.poleFitErrorCm);
  packed.lastPoleYCm = entry.lastPoleYCm;
  packed.wallCm = entry.wallCm;
  packed.wallAngleDeg10 = toX10(entry.wallAngleDeg);
  packed.batteryV100 = (uint16_t)lroundf(constrain(entry.batteryV * 100.0f, 0.0f, 65000.0f));
  packed.driveLeft100 = (int8_t)lroundf(constrain(entry.driveLeft * 100.0f, -127.0f, 127.0f));
  packed.driveRight100 = (int8_t)lroundf(constrain(entry.driveRight * 100.0f, -127.0f, 127.0f));
  packed.stateIndex = 0;
  for (uint8_t i = 0; i < sizeof(kStateNames) / sizeof(kStateNames[0]); i++)
    if (strcmp(entry.state, kStateNames[i]) == 0) packed.stateIndex = i;
  packed.poleCount = entry.poleCount;
  packed.cliffBaselineCm = entry.cliffBaselineCm;
  packed.flags = (entry.isImuOk ? FLAG_IMU : 0) | (entry.isLidarOk ? FLAG_LIDAR : 0) |
                 (entry.hasGpsFix ? FLAG_GPS : 0) | (entry.isPaused ? FLAG_PAUSED : 0) |
                 (entry.isSafetyEnabled ? FLAG_SAFETY : 0) | (entry.isPoleSeen ? FLAG_POLE : 0) |
                 (entry.isWallSeen ? FLAG_WALL : 0) | (entry.isWallSteering ? FLAG_WALL_STEER : 0);
  _runRowCount = _runRowCount + 1;  // after the row is complete: the flush task reads only up to here
}

void Logger::_unpackRunRow(const RunRow& packed, Row& entry) const {
  memset(&entry, 0, sizeof(entry));
  entry.timestampMs = packed.timestampMs;
  strlcpy(entry.state, kStateNames[packed.stateIndex], sizeof(entry.state));
  entry.driveLeft = packed.driveLeft100 / 100.0f;
  entry.driveRight = packed.driveRight100 / 100.0f;
  for (int i = 0; i < 5; i++) entry.ultrasonicCm[i] = packed.ultrasonicCm[i];
  entry.yawDeg = packed.yawDeg10 / 10.0f;
  entry.pitchDeg = packed.pitchDeg10 / 10.0f;
  entry.rollDeg = packed.rollDeg10 / 10.0f;
  entry.isImuOk = packed.flags & FLAG_IMU;
  entry.isLidarOk = packed.flags & FLAG_LIDAR;
  for (int i = 0; i < 4; i++) entry.lidarSectorCm[i] = packed.lidarSectorCm[i];
  entry.hasGpsFix = packed.flags & FLAG_GPS;
  entry.latitudeDeg = packed.latitudeDeg;
  entry.longitudeDeg = packed.longitudeDeg;
  entry.batteryV = packed.batteryV100 / 100.0f;
  entry.loopUs = packed.loopUs;
  if (packed.noteOffset != kNoNote) strlcpy(entry.note, _runNotes + packed.noteOffset, sizeof(entry.note));
  entry.yawRateDps = packed.yawRateDps10 / 10.0f;
  entry.headingRefDeg = packed.headingRefDeg10 / 10.0f;
  entry.crossTrackCm = packed.crossTrackCm10 / 10.0f;
  entry.imuCalibration = packed.imuCalibration;
  entry.frontCm = packed.frontCm;
  entry.lidarCorridorCm = packed.lidarCorridorCm;
  entry.lidarFrontRightCm = packed.lidarFrontRightCm;
  entry.lidarFrontLeftCm = packed.lidarFrontLeftCm;
  entry.isPaused = packed.flags & FLAG_PAUSED;
  entry.isSafetyEnabled = packed.flags & FLAG_SAFETY;
  entry.poleCount = packed.poleCount;
  entry.isPoleSeen = packed.flags & FLAG_POLE;
  entry.poleXCm = packed.poleXCm;
  entry.poleYCm = packed.poleYCm;
  entry.poleDiameterCm = packed.poleDiameterCm10 / 10.0f;
  entry.poleFitErrorCm = packed.poleFitErrorCm10 / 10.0f;
  entry.lastPoleYCm = packed.lastPoleYCm;
  entry.isWallSeen = packed.flags & FLAG_WALL;
  entry.wallCm = packed.wallCm;
  entry.wallAngleDeg = packed.wallAngleDeg10 / 10.0f;
  entry.isWallSteering = packed.flags & FLAG_WALL_STEER;
  entry.cliffBaselineCm = packed.cliffBaselineCm;
}

// Flush task, after the run: the rows from before the run go to the old file, then a new
// run_<n>.csv gets all RAM rows (the rows after the run follow in the same file).
void Logger::_writeRun() {
  if (_textLength) _flush();
  _rotate();
  Row entry;
  const uint16_t rowCount = _runRowCount;
  for (uint16_t i = 0; i < rowCount; i++) {
    _unpackRunRow(_runChunks[i / kRunChunkRows][i % kRunChunkRows], entry);
    if (_textLength + kMaxLine > sizeof(_textBuffer)) _flush();
    _format(entry);
  }
  if (_textLength) _flush();
  Serial.printf("[log] run written to %s: %u rows", _fileName, rowCount);
  if (_runDroppedRows) Serial.printf(", RAM full: last %lu rows not logged", (unsigned long)_runDroppedRows);
  Serial.println();
  _freeRun();
  _isRunWritePending = false;
}

void Logger::_freeRun() {
  for (auto& chunk : _runChunks) {
    free(chunk);
    chunk = nullptr;
  }
  _runCapacity = 0;
  _runRowCount = 0;
}

void Logger::mark(const char* text) {
  char cleanText[sizeof(_markText)];
  _copyField(cleanText, sizeof(cleanText), text ? text : "");
  portENTER_CRITICAL(&_markMux);
  strlcpy(_markText, cleanText, sizeof(_markText));
  _markPending = cleanText[0] != 0;
  portEXIT_CRITICAL(&_markMux);
}

// Copies src into dst (n bytes incl. terminator); characters that would break a CSV row become '_'.
void Logger::_copyField(char* destination, size_t capacity, const char* source) {
  size_t i = 0;
  for (; source[i] && i + 1 < capacity; i++) {
    const char character = source[i];
    destination[i] = (character == ',' || character == '"' || character == '\n' || character == '\r' || (uint8_t)character < 0x20) ? '_' : character;
  }
  destination[i] = 0;
}

String Logger::status() const {
  if (_isRunActive) {
    String runLine = String("run in RAM ") + _runRowCount + "/" + _runCapacity;
    if (_runDroppedRows) runLine += " FULL";
    return runLine;
  }
  if (_isRunWritePending) return String("writing run ...");
  switch (_state) {
    case NO_FS: return "no filesystem";
    case FAILED: return String("STOPPED write errors ") + _fileName;
    case FULL:  return String("FULL ") + _fileName + " " + (_fileBytes / 1024) + "kB";
    default: break;
  }
  String statusLine = String(_fileName) + " " + (_fileBytes / 1024) + "kB";
  if (_totalBytes) statusLine += " " + String((uint32_t)(_usedBytes * 100ULL / _totalBytes)) + "%";
  if (_droppedRowCount) statusLine += " drop " + String(_droppedRowCount);
  return statusLine;
}

// ---- flush task (core 0) ---------------------------------------------------------------------

void Logger::_taskEntry(void* self) { static_cast<Logger*>(self)->_task(); }

void Logger::_task() {
  Item item;
  for (;;) {
    if (xQueueReceive(_rowQueue, &item, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (item.kind == ITEM_RUN_WRITE) {
        _writeRun();
        continue;
      }
      if (item.startsNewFile) {
        if (_textLength) _flush();  // the rows before belong to the old file
        _rotate();
      }
      if (_textLength + kMaxLine > sizeof(_textBuffer)) _flush();  // never overflow the text buffer
      _format(item.row);
    }
    // no flash access while the rover drives on its own: rows from before the run wait in RAM
    if (_isRunActive) continue;
    if (_textLength && (_pendingRows >= kFlushRows || millis() - _lastFlushMs >= kFlushMs)) _flush();
  }
}

// One CSV line in the column order of kHeader (44 columns). Pole and wall fields stay empty
// when nothing was detected in that scan.
void Logger::_format(const Row& entry) {
  const size_t capacity = sizeof(_textBuffer);
  size_t length = _textLength;
  bool fits = appendf(_textBuffer, capacity, length,
    "%lu,%s,%.2f,%.2f,%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.2f,%lu,%s,",
    (unsigned long)entry.timestampMs, entry.state, entry.driveLeft, entry.driveRight,
    entry.ultrasonicCm[0], entry.ultrasonicCm[1], entry.ultrasonicCm[2], entry.ultrasonicCm[3], entry.ultrasonicCm[4],
    entry.yawDeg, entry.pitchDeg, entry.rollDeg, entry.isImuOk ? 1 : 0, entry.isLidarOk ? 1 : 0,
    entry.lidarSectorCm[0], entry.lidarSectorCm[1], entry.lidarSectorCm[2], entry.lidarSectorCm[3],
    entry.hasGpsFix ? 1 : 0, entry.latitudeDeg, entry.longitudeDeg, entry.batteryV, (unsigned long)entry.loopUs, entry.note);
  fits = fits && appendf(_textBuffer, capacity, length, "%.1f,%.1f,%.1f,%u,%d,%u,%u,%u,%d,%d,%u,",
    entry.yawRateDps, entry.headingRefDeg, entry.crossTrackCm, (unsigned)entry.imuCalibration, entry.frontCm, (unsigned)entry.lidarCorridorCm,
    (unsigned)entry.lidarFrontRightCm, (unsigned)entry.lidarFrontLeftCm, entry.isPaused ? 1 : 0, entry.isSafetyEnabled ? 1 : 0, (unsigned)entry.poleCount);
  fits = fits && (entry.isPoleSeen ? appendf(_textBuffer, capacity, length, "%d,%d,%.1f,%.1f,", entry.poleXCm, entry.poleYCm, entry.poleDiameterCm, entry.poleFitErrorCm)
                         : appendf(_textBuffer, capacity, length, ",,,,"));
  fits = fits && appendf(_textBuffer, capacity, length, "%d,", entry.lastPoleYCm);
  fits = fits && (entry.isWallSeen ? appendf(_textBuffer, capacity, length, "%d,%.1f,", entry.wallCm, entry.wallAngleDeg)
                         : appendf(_textBuffer, capacity, length, ",,"));
  fits = fits && appendf(_textBuffer, capacity, length, "%d,%u\n", entry.isWallSteering ? 1 : 0, (unsigned)entry.cliffBaselineCm);
  if (fits) {  // a line that did not fit is dropped as a whole (kMaxLine headroom prevents that)
    _textLength = length;
    _pendingRows++;
  }
}

// Starts the next run_<n>.csv with its header (flush task, after the old buffer was written).
void Logger::_rotate() {
  if (_state != LOGGING) return;  // full or stopped: no new file
  if (xSemaphoreTake(_fsMutex, pdMS_TO_TICKS(2000)) != pdTRUE) return;  // keep writing the old file
  const int runNumber = _nextRunNumber();
  char newFileName[sizeof(_fileName)], newFilePath[sizeof(_filePath)];
  snprintf(newFileName, sizeof(newFileName), "run_%d.csv", runNumber);
  snprintf(newFilePath, sizeof(newFilePath), "%s/%s", kLogDir, newFileName);
  strlcpy(_filePath, newFilePath, sizeof(_filePath));
  strlcpy(_fileName, newFileName, sizeof(_fileName));
  if (!_writeHeader()) {
    _state = FAILED;
    Serial.printf("[log] cannot create %s, logging stopped\n", _filePath);
  } else {
    Serial.printf("[log] new file %s\n", _filePath);
  }
  _checkFill();
  xSemaphoreGive(_fsMutex);
}

// Opens, appends and closes the current file, then checks the partition fill every 5 s.
void Logger::_flush() {
  if (xSemaphoreTake(_fsMutex, pdMS_TO_TICKS(2000)) != pdTRUE) return;  // keep the text, retry later
  if (_state == LOGGING) {
    File file = LittleFS.open(_filePath, FILE_APPEND);
    const size_t writtenBytes = file ? file.write((const uint8_t*)_textBuffer, _textLength) : 0;
    if (file) file.close();
    if (writtenBytes == _textLength) {
      _fileBytes = _fileBytes + writtenBytes;
      _writeErrorCount = 0;
    } else {
      _droppedRowCount = _droppedRowCount + _pendingRows;
      if (++_writeErrorCount >= kMaxWriteErrors) {
        _state = FAILED;
        Serial.printf("[log] %u write errors in a row, logging stopped\n", _writeErrorCount);
      }
    }
    if (millis() - _lastFillCheckMs >= kFillCheckMs) {
      _checkFill();
      if (_state == LOGGING && _usedBytes >= (uint64_t)_totalBytes * 9 / 10) {
        _state = FULL;
        Serial.printf("[log] flash %u/%u kB used (90 %%), logging stopped\n", _usedBytes / 1024, _totalBytes / 1024);
      }
    }
  } else {
    _droppedRowCount = _droppedRowCount + _pendingRows;  // stopped: text is thrown away
  }
  _textLength = 0;
  _pendingRows = 0;
  _lastFlushMs = millis();
  xSemaphoreGive(_fsMutex);
}

void Logger::_checkFill() {
  _usedBytes = LittleFS.usedBytes();
  _totalBytes = LittleFS.totalBytes();
  _lastFillCheckMs = millis();
}

// ---- web side (server task) ------------------------------------------------------------------

bool Logger::validName(const char* candidate) {
  if (!candidate || strncmp(candidate, "run_", 4) != 0) return false;
  const char* cursor = candidate + 4;
  if (!isdigit((uint8_t)*cursor)) return false;
  while (isdigit((uint8_t)*cursor)) cursor++;
  return strcmp(cursor, ".csv") == 0 && strlen(candidate) < sizeof(_fileName);
}

uint8_t Logger::files(FileInfo* fileList, uint8_t maxFiles) {
  if (!ready() || xSemaphoreTake(_fsMutex, pdMS_TO_TICKS(3000)) != pdTRUE) return 0;
  uint8_t fileCount = 0;
  File logDir = LittleFS.open(kLogDir);
  if (logDir && logDir.isDirectory()) {
    File entry;
    while (fileCount < maxFiles && (entry = logDir.openNextFile())) {
      const char* entryName = entry.name();
      const char* baseName = strrchr(entryName, '/');
      baseName = baseName ? baseName + 1 : entryName;
      if (!entry.isDirectory() && validName(baseName)) {
        strlcpy(fileList[fileCount].name, baseName, sizeof(fileList[fileCount].name));
        fileList[fileCount].size = entry.size();
        fileCount++;
      }
      entry.close();
    }
    logDir.close();
  }
  xSemaphoreGive(_fsMutex);
  // sort by run number (insertion sort, few files)
  for (uint8_t i = 1; i < fileCount; i++) {
    FileInfo current = fileList[i];
    int runNumber = 0, otherRunNumber;
    sscanf(current.name, "run_%d", &runNumber);
    int j = i - 1;
    while (j >= 0 && (sscanf(fileList[j].name, "run_%d", &otherRunNumber) == 1 && otherRunNumber > runNumber)) { fileList[j + 1] = fileList[j]; j--; }
    fileList[j + 1] = current;
  }
  return fileCount;
}

size_t Logger::readChunk(const char* requestedName, uint8_t* buffer, size_t maxLength, size_t offset) {
  if (!ready() || !validName(requestedName)) return 0;
  if (xSemaphoreTake(_fsMutex, pdMS_TO_TICKS(500)) != pdTRUE) return (size_t)-1;
  char filePath[40];
  snprintf(filePath, sizeof(filePath), "%s/%s", kLogDir, requestedName);
  size_t bytesRead = 0;
  File file = LittleFS.open(filePath, FILE_READ);
  if (file) {
    if (offset < file.size() && file.seek(offset)) bytesRead = file.read(buffer, maxLength);
    file.close();
  }
  xSemaphoreGive(_fsMutex);
  return bytesRead;
}

// Deletes every run_*.csv and recreates the current file with its header. The flush task keeps
// running; rows queued meanwhile land in the new file.
bool Logger::clear() {
  if (!ready() || xSemaphoreTake(_fsMutex, pdMS_TO_TICKS(3000)) != pdTRUE) return false;
  FileInfo fileList[kMaxFiles];
  // collect first: deleting while iterating the directory is not safe on every FS
  uint8_t fileCount = 0;
  File logDir = LittleFS.open(kLogDir);
  if (logDir && logDir.isDirectory()) {
    File entry;
    while (fileCount < kMaxFiles && (entry = logDir.openNextFile())) {
      const char* entryName = entry.name();
      const char* baseName = strrchr(entryName, '/');
      baseName = baseName ? baseName + 1 : entryName;
      if (!entry.isDirectory() && validName(baseName)) strlcpy(fileList[fileCount++].name, baseName, sizeof(fileList[0].name));
      entry.close();
    }
    logDir.close();
  }
  for (uint8_t i = 0; i < fileCount; i++) {
    char filePath[40];
    snprintf(filePath, sizeof(filePath), "%s/%s", kLogDir, fileList[i].name);
    LittleFS.remove(filePath);
  }
  const bool isHeaderWritten = _writeHeader();
  _checkFill();
  _writeErrorCount = 0;
  _droppedRowCount = 0;
  if (isHeaderWritten && _state != NO_FS) _state = LOGGING;
  xSemaphoreGive(_fsMutex);
  Serial.printf("[log] cleared %u file(s), new %s\n", fileCount, _filePath);
  return isHeaderWritten;
}
