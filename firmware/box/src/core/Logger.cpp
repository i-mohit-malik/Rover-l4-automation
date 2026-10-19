#include "Logger.h"

static const char kDir[] = "/log";
// Row format v1, docs/logging.md: fixed column order, add new columns only at the end.
static const char kHeader[] =
  "t_ms,state,l,r,us_f,us_l,us_r,us_b,us_fa,yaw,pitch,roll,imu_ok,lid_ok,"
  "lid_f,lid_l,lid_r,lid_b,gps_fix,lat,lon,volt,loop_us,note";

// ---------------------------------------------------------------------------------------------

bool Logger::begin() {
  _mtx = xSemaphoreCreateMutex();
  _queue = xQueueCreate(kQueueRows, sizeof(Row));
  if (!_mtx || !_queue) {
    Serial.println("[log] out of memory, logging disabled");
    return false;
  }

  if (!LittleFS.begin(true)) {  // true: format the partition on the first boot
    Serial.println("[log] LittleFS mount failed, logging disabled");
    return false;
  }
  if (!LittleFS.exists(kDir)) LittleFS.mkdir(kDir);

  const int n = _nextRunNumber();
  snprintf(_name, sizeof(_name), "run_%d.csv", n);
  snprintf(_path, sizeof(_path), "%s/%s", kDir, _name);
  if (!_writeHeader()) {
    Serial.printf("[log] cannot create %s, logging disabled\n", _path);
    return false;
  }
  _checkFill();
  _state = _used >= (uint64_t)_total * 9 / 10 ? FULL : LOGGING;
  _lastFlushMs = millis();
  Serial.printf("[log] %s, flash %u/%u kB used%s\n", _path, _used / 1024, _total / 1024,
                _state == FULL ? " - FULL, logging stopped" : "");

  xTaskCreatePinnedToCore(_taskEntry, "logger", 6144, this, 1, &_taskHandle, 0);
  return _state == LOGGING;
}

// Highest run_<n>.csv in /log + 1 (1 on an empty partition).
int Logger::_nextRunNumber() {
  int maxN = 0;
  File dir = LittleFS.open(kDir);
  if (dir && dir.isDirectory()) {
    File f;
    while ((f = dir.openNextFile())) {
      const char* name = f.name();
      const char* base = strrchr(name, '/');
      base = base ? base + 1 : name;
      int n = 0;
      if (sscanf(base, "run_%d.csv", &n) == 1 && n > maxN) maxN = n;
      f.close();
    }
    dir.close();
  }
  return maxN + 1;
}

// Creates the current file with the version comment and the header row (caller holds _mtx or
// runs before the task exists).
bool Logger::_writeHeader() {
  File f = LittleFS.open(_path, FILE_WRITE);
  if (!f) return false;
  f.printf("# log v%d\n%s\n", kFormatVersion, kHeader);
  const size_t size = f.size();
  f.close();
  _bytes = size;
  return true;
}

// ---- loop() side -----------------------------------------------------------------------------

void Logger::row(const Row& r) {
  if (!_queue) return;
  Row q = r;
  _copyField(q.state, sizeof(q.state), r.state);
  _copyField(q.note, sizeof(q.note), r.note);

  // merge a pending mark() into the note column: "code_note;mark" or just "mark"
  char mark[sizeof(_markText)] = "";
  portENTER_CRITICAL(&_markMux);
  if (_markPending) {
    strlcpy(mark, _markText, sizeof(mark));
    _markPending = false;
  }
  portEXIT_CRITICAL(&_markMux);
  if (mark[0]) {
    if (q.note[0]) strlcat(q.note, ";", sizeof(q.note));
    strlcat(q.note, mark, sizeof(q.note));
  }

  if (_state != LOGGING || xQueueSend(_queue, &q, 0) != pdTRUE) _dropped = _dropped + 1;
}

void Logger::mark(const char* text) {
  char clean[sizeof(_markText)];
  _copyField(clean, sizeof(clean), text ? text : "");
  portENTER_CRITICAL(&_markMux);
  strlcpy(_markText, clean, sizeof(_markText));
  _markPending = clean[0] != 0;
  portEXIT_CRITICAL(&_markMux);
}

// Copies src into dst (n bytes incl. terminator); characters that would break a CSV row become '_'.
void Logger::_copyField(char* dst, size_t n, const char* src) {
  size_t i = 0;
  for (; src[i] && i + 1 < n; i++) {
    const char c = src[i];
    dst[i] = (c == ',' || c == '"' || c == '\n' || c == '\r' || (uint8_t)c < 0x20) ? '_' : c;
  }
  dst[i] = 0;
}

String Logger::status() const {
  switch (_state) {
    case NO_FS: return "no filesystem";
    case FAILED: return String("STOPPED write errors ") + _name;
    case FULL:  return String("FULL ") + _name + " " + (_bytes / 1024) + "kB";
    default: break;
  }
  String s = String(_name) + " " + (_bytes / 1024) + "kB";
  if (_total) s += " " + String((uint32_t)(_used * 100ULL / _total)) + "%";
  if (_dropped) s += " drop " + String(_dropped);
  return s;
}

// ---- flush task (core 0) ---------------------------------------------------------------------

void Logger::_taskEntry(void* self) { static_cast<Logger*>(self)->_task(); }

void Logger::_task() {
  Row r;
  for (;;) {
    if (xQueueReceive(_queue, &r, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (_bufLen + kMaxLine > sizeof(_buf)) _flush();  // never overflow the text buffer
      _format(r);
    }
    if (_bufLen && (_pendingRows >= kFlushRows || millis() - _lastFlushMs >= kFlushMs)) _flush();
  }
}

// One CSV line in the column order of kHeader (24 columns).
void Logger::_format(const Row& r) {
  const int n = snprintf(_buf + _bufLen, sizeof(_buf) - _bufLen,
    "%lu,%s,%.2f,%.2f,%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.2f,%lu,%s\n",
    (unsigned long)r.tMs, r.state, r.l, r.r,
    r.us[0], r.us[1], r.us[2], r.us[3], r.us[4],
    r.yaw, r.pitch, r.roll, r.imuOk ? 1 : 0, r.lidOk ? 1 : 0,
    r.lid[0], r.lid[1], r.lid[2], r.lid[3],
    r.gpsFix ? 1 : 0, r.lat, r.lon, r.volt, (unsigned long)r.loopUs, r.note);
  if (n > 0 && (size_t)n < sizeof(_buf) - _bufLen) {
    _bufLen += n;
    _pendingRows++;
  }
}

// Opens, appends and closes the current file, then checks the partition fill every 5 s.
void Logger::_flush() {
  if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(2000)) != pdTRUE) return;  // keep the text, retry later
  if (_state == LOGGING) {
    File f = LittleFS.open(_path, FILE_APPEND);
    const size_t w = f ? f.write((const uint8_t*)_buf, _bufLen) : 0;
    if (f) f.close();
    if (w == _bufLen) {
      _bytes = _bytes + w;
      _writeErrors = 0;
    } else {
      _dropped = _dropped + _pendingRows;
      if (++_writeErrors >= kMaxWriteErrors) {
        _state = FAILED;
        Serial.printf("[log] %u write errors in a row, logging stopped\n", _writeErrors);
      }
    }
    if (millis() - _lastFillCheckMs >= kFillCheckMs) {
      _checkFill();
      if (_state == LOGGING && _used >= (uint64_t)_total * 9 / 10) {
        _state = FULL;
        Serial.printf("[log] flash %u/%u kB used (90 %%), logging stopped\n", _used / 1024, _total / 1024);
      }
    }
  } else {
    _dropped = _dropped + _pendingRows;  // stopped: text is thrown away
  }
  _bufLen = 0;
  _pendingRows = 0;
  _lastFlushMs = millis();
  xSemaphoreGive(_mtx);
}

void Logger::_checkFill() {
  _used = LittleFS.usedBytes();
  _total = LittleFS.totalBytes();
  _lastFillCheckMs = millis();
}

// ---- web side (server task) ------------------------------------------------------------------

bool Logger::validName(const char* name) {
  if (!name || strncmp(name, "run_", 4) != 0) return false;
  const char* p = name + 4;
  if (!isdigit((uint8_t)*p)) return false;
  while (isdigit((uint8_t)*p)) p++;
  return strcmp(p, ".csv") == 0 && strlen(name) < sizeof(_name);
}

uint8_t Logger::files(FileInfo* out, uint8_t max) {
  if (!ready() || xSemaphoreTake(_mtx, pdMS_TO_TICKS(3000)) != pdTRUE) return 0;
  uint8_t n = 0;
  File dir = LittleFS.open(kDir);
  if (dir && dir.isDirectory()) {
    File f;
    while (n < max && (f = dir.openNextFile())) {
      const char* name = f.name();
      const char* base = strrchr(name, '/');
      base = base ? base + 1 : name;
      if (!f.isDirectory() && validName(base)) {
        strlcpy(out[n].name, base, sizeof(out[n].name));
        out[n].size = f.size();
        n++;
      }
      f.close();
    }
    dir.close();
  }
  xSemaphoreGive(_mtx);
  // sort by run number (insertion sort, few files)
  for (uint8_t i = 1; i < n; i++) {
    FileInfo key = out[i];
    int k = 0, kj;
    sscanf(key.name, "run_%d", &k);
    int j = i - 1;
    while (j >= 0 && (sscanf(out[j].name, "run_%d", &kj) == 1 && kj > k)) { out[j + 1] = out[j]; j--; }
    out[j + 1] = key;
  }
  return n;
}

size_t Logger::readChunk(const char* name, uint8_t* buf, size_t maxLen, size_t index) {
  if (!ready() || !validName(name)) return 0;
  if (xSemaphoreTake(_mtx, pdMS_TO_TICKS(500)) != pdTRUE) return (size_t)-1;
  char path[40];
  snprintf(path, sizeof(path), "%s/%s", kDir, name);
  size_t n = 0;
  File f = LittleFS.open(path, FILE_READ);
  if (f) {
    if (index < f.size() && f.seek(index)) n = f.read(buf, maxLen);
    f.close();
  }
  xSemaphoreGive(_mtx);
  return n;
}

// Deletes every run_*.csv and recreates the current file with its header. The flush task keeps
// running; rows queued meanwhile land in the new file.
bool Logger::clear() {
  if (!ready() || xSemaphoreTake(_mtx, pdMS_TO_TICKS(3000)) != pdTRUE) return false;
  FileInfo list[kMaxFiles];
  // collect first: deleting while iterating the directory is not safe on every FS
  uint8_t n = 0;
  File dir = LittleFS.open(kDir);
  if (dir && dir.isDirectory()) {
    File f;
    while (n < kMaxFiles && (f = dir.openNextFile())) {
      const char* name = f.name();
      const char* base = strrchr(name, '/');
      base = base ? base + 1 : name;
      if (!f.isDirectory() && validName(base)) strlcpy(list[n++].name, base, sizeof(list[0].name));
      f.close();
    }
    dir.close();
  }
  for (uint8_t i = 0; i < n; i++) {
    char path[40];
    snprintf(path, sizeof(path), "%s/%s", kDir, list[i].name);
    LittleFS.remove(path);
  }
  const bool ok = _writeHeader();
  _checkFill();
  _writeErrors = 0;
  _dropped = 0;
  if (ok && _state != NO_FS) _state = LOGGING;
  xSemaphoreGive(_mtx);
  Serial.printf("[log] cleared %u file(s), new %s\n", n, _path);
  return ok;
}
