#include "DisplayManager.h"

static const uint8_t kWidth = 128, kHeight = 32;
static const uint8_t kI2cAddress = 0x3C;
static const uint8_t kMaxChars = 20;   // 6 px per character at text size 1
static const uint8_t kLineStep = 12;   // 8 px text + 4 px gap -> 3 lines on 32 px

DisplayManager::DisplayManager() : _oled(kWidth, kHeight, &Wire, -1) {}

bool DisplayManager::begin() {
  _isReady = _oled.begin(SSD1306_SWITCHCAPVCC, kI2cAddress);
  if (!_isReady) return false;
  _oled.setTextColor(SSD1306_WHITE);
  _oled.setTextSize(1);
  _oled.clearDisplay();
  _oled.display();
  return true;
}

void DisplayManager::show(const char* line1, const char* line2, const char* line3) {
  if (!_isReady) return;
  _oled.clearDisplay();
  uint8_t lineIndex = 0;
  for (const char* line : {line1, line2, line3}) {
    if (line && line[0]) _printLine(lineIndex++, line);  // empty lines do not leave a gap
  }
  _oled.display();
}

void DisplayManager::_printLine(uint8_t lineIndex, const char* text) {
  char lineText[kMaxChars + 1];
  strncpy(lineText, text, kMaxChars);
  lineText[kMaxChars] = '\0';
  _oled.setCursor(0, lineIndex * kLineStep);
  _oled.print(lineText);
}
