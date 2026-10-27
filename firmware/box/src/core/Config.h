#pragma once
// Config - the one place for everything a team member may want to change before flashing.
//
// Pins are NOT here: the box PCB is fixed and every module keeps its own pin constants
// (UsManager.h, LidarScan.h, GpsManager.cpp, Sensors.cpp for I2C). The full table is in
// docs/hardware/pinout.md. Do not add pin numbers here unless the modules read them from here.
// Tuning of the autonomous run (speeds, gains, poles, wall, turn) is in ../nav/NavConfig.h.

#include <Arduino.h>

// ---- YOUR KIT ------------------------------------------------------------------------------
// 1..10 = kit number on the box sticker. Wi-Fi name, password, IP and channel derive from this
// (see box.ino header). A wrong value either loses the rover or hijacks another team's network.
const int BOX_ID = 6;

// ---- firmware --------------------------------------------------------------------------------
// Shown on the web page and the update page: change it with every flash, so you can see on the
// phone which version runs.
const char FW_VERSION[] = "27.10-campus timing";

// ---- driving -------------------------------------------------------------------------------
const float    MAX_SPEED       = 0.5f;  // drive pad maximum; 0.5 = full rover speed (100 % PWM)
const uint32_t PAD_TIMEOUT_MS  = 400;   // no update from the phone for this long -> stop
const uint32_t ROVER_PERIOD_MS = 100;   // RoverLink resends the current set point this often;
                                        // also the control step and the log tick
const uint32_t LINK_TIMEOUT_MS = 3000;  // autonomous run: stop if the page has not polled for 3 s
                                        // (crowded Wi-Fi tolerant, 26.10 runs)

// ---- sensors -------------------------------------------------------------------------------
const int LIDAR_OFFSET_DEG = 90;        // rotates the LiDAR view so that "front" is up
const bool USE_COMPASS = false;         // false = IMUPLUS: heading from the gyro only, relative to
                                        // power-on, no magnetometer calibration needed (with the
                                        // uncalibrated compass the rover curved while the IMU saw ~0 deg)

// ---- web page / OLED refresh ----------------------------------------------------------------
const uint32_t SNAPSHOT_MS = 250;       // sensor snapshot for the page
const uint32_t UI_MS       = 500;       // status line + "Dev" values
const uint32_t OLED_MS     = 1000;

// ---- timeline log ----------------------------------------------------------------------------
const uint32_t LOG_IDLE_PERIOD_MS = 1000;  // no run and the rover stands: one row per second instead of
                                           // one per ROVER_PERIOD_MS (saves flash, keeps the timeline)

// ---- serial load report ---------------------------------------------------------------------
const uint32_t REPORT_MS = 10000;       // "[sys] loop avg ..." line on Serial
