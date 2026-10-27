#pragma once
// NavConfig - tuning of the autonomous run (src/nav/). Values from the CampusLogger runs of
// 26.10.2026 ("passed open sea"); the comments say where a value came from. Change them here only.
// Kit, link and safety timeouts stay in ../core/Config.h; the behaviour is in docs/autonomous_run.md.
#include <Arduino.h>

// ---- straight driving: heading hold (Navigator) ----------------------------------------------
const float CRUISE_SPEED       = 0.22f;   // steady, safe speed (-0.5 .. 0.5)
const float HEADING_KP         = 0.022f;  // stiff P gain for quick heading correction
const float HEADING_KI         = 0.005f;  // I gain against steady veer / drift
const float HEADING_KD         = 0.004f;  // damping from the gyro turn rate (per deg/s)
const float HEADING_I_LIMIT    = 15.0f;   // anti-windup: integral clamped to +-15 deg*s
const float STEER_TRIM         = -0.05f;  // built-in pull to the right (26.10: L 0.17 / R 0.27); re-check per battery
const float DRIVE_LIMIT        = 0.4f;    // per side while driving straight

// ---- virtual line (LineTracker) ----------------------------------------------------------------
const float CM_PER_S_PER_CMD     = 96.0f; // speed per drive unit: 0.22 -> ~21 cm/s (26.10 run 7)
const float LINE_GAIN_DEG_PER_CM = 0.3f;  // aim 0.3 deg back toward the start line per cm off
const float LINE_MAX_DEG         = 6.0f;  // ...but never more than 6 deg
const float DRIFT_MAX_DEG        = 2.0f;  // one pole / wall drift correction turns the line by at most this
const float CROSS_TRACK_MAX_CM   = 20.0f; // a pole sets the cross-track offset to at most this

// ---- run limits ----------------------------------------------------------------------------------
const uint32_t MAX_RUN_TIME_MS = 0;       // driving time limit; 0 = none (STOP / link loss / safety)

// ---- obstacle pause (SafetyMonitor) ----------------------------------------------------------
const int FRONT_STOP_CM      = 45;        // pause if the LiDAR lane check sees something within 45 cm...
const int FRONT_RESUME_CM    = 60;        // ...resume once it is beyond 60 cm
const int FRONT_DEBOUNCE     = 3;         // control steps in a row for pause and for resume
const int US_CLOSE_CM        = 25;        // front ultrasonic counts only below this (its cone hits the ground at ~35-45 cm)
const int CORRIDOR_HALF_WIDTH_CM = 13;    // half rover width (8 cm) + 5 cm: only obstacles in our own lane
const int CORRIDOR_LOOKAHEAD_CM  = 150;   // how far ahead the corridor check looks

// ---- cliff check, 45 deg ultrasonic (SafetyMonitor) ------------------------------------------
const bool CLIFF_CHECK_ENABLED = false;   // off: on campus pavement the 45 deg sensor gets no echo ~73 % of
                                          // the time, so the check fired on random echoes (26.10). Still logged.
const int CLIFF_DELTA_CM       = 12;      // step down: baseline + 12 cm (e.g. 17 + 12 = 29 cm)
const int CLIFF_DEBOUNCE       = 3;       // control steps in a row
const int BASELINE_DEFAULT_CM  = 17;      // typical 45 deg reading on flat ground (26.10 log)
const int BASELINE_MIN_CM      = 8;       // a measured baseline outside 8..30 cm is a bad reading -> default
const int BASELINE_MAX_CM      = 30;
const int CLIFF_MAX_STEP_CM    = 150;     // upper limit for a step-drop echo
const int VOID_TIMEOUT_CYCLES  = 14;      // 1.4 s of "no echo" before it counts (tactile strips reflect away)

// ---- pole counting (Perception + LineTracker) --------------------------------------------------
const float POLE_DIAMETER_CM   = 35.0f;   // tape: 111 cm around the pole -> 35 cm
const int   POLE_MIN_RIGHT_CM  = 30;      // poles count only on the right, 30..250 cm from the rover centre
const int   POLE_MAX_RIGHT_CM  = 250;
const int   POLES_BEFORE_TURN  = 4;       // turn right after passing this many poles
const uint32_t TURN_DELAY_MS   = 6000;    // keep driving straight this long after the last pole, then turn
const bool  POLE_SHAPE_CHECK   = true;    // round with the measured diameter (rejects standing people)
const float POLE_DIAMETER_MIN  = 0.6f;    // fitted diameter 0.6 .. 1.5 x 35 cm = 21 .. 53 cm
const float POLE_DIAMETER_MAX  = 1.5f;
const float POLE_FIT_MAX_CM    = 2.5f;    // max. average distance of the points from the fitted circle
const bool  POLE_LINE_CORRECTION = true;  // each pole's side distance corrects our drift (26.10: ~9 cm per gap)
const float POLE_JUMP_CM       = 25.0f;   // side distance differs more than this -> not used for steering

// ---- wall alignment, left side (Perception + LineTracker) --------------------------------------
// While a straight wall is visible on the left, keep parallel to it at the distance it had when it
// was first seen. Runs 9/10 drifted 8-16 cm along the wall before the first pole could correct it.
const bool  WALL_ALIGNMENT     = true;
const int   WALL_MAX_CM        = 250;     // only walls closer than this
const int   WALL_MIN_POINTS    = 15;      // LiDAR points on the line (1 per degree)
const float WALL_MIN_SPAN_CM   = 60.0f;   // the line must be at least this long (rules out poles, people)
const float WALL_FIT_MAX_CM    = 3.0f;    // max. average distance of the points from the line
const float WALL_MAX_ANGLE_DEG = 20.0f;   // sanity check only: the wall ANGLE is never steered on (the LiDAR
                                          // mounting can be a few deg off, 26.10 run 13); only the DISTANCE steers
const float WALL_SLOPE_CM      = 150.0f;  // heading correction from the wall distance change over this much driving
const float WALL_DIST_GAIN     = 0.2f;    // per step: how fast the offset follows the measured wall distance
const float WALL_JUMP_CM       = 20.0f;   // distance jump (recess, door) -> new reference, no steering on it

// ---- 90 deg right turn in place (Navigator) ----------------------------------------------------
const float TURN_SPEED          = 0.5f;   // in-place rotation only works at full speed
const float TURN_LEAD_S         = 0.25f;  // stop early by (turn rate x this): the stop command arrives late
const float TURN_TOLERANCE_DEG  = 8.0f;   // error left after the turn that the straight controller fixes
const uint32_t TURN_SETTLE_MS   = 400;    // stand still after a turn pulse before measuring the error
const uint32_t TURN_PULSE_MAX_MS = 5000;  // one turn pulse never lasts longer than this
const int   TURN_MAX_PULSES     = 3;      // full-speed attempts before giving up and driving on
