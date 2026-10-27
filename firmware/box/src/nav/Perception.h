#pragma once
// Perception - features from one LiDAR scan for the autonomous run. Pure functions: they read the
// live 1-degree LiDAR view (Sensors::lidarCm, vehicle frame: 0 = front, 90 = right, clockwise) and
// keep no state. Call from loop() only (the LiDAR map is filled there).
// Coordinates of the results: x = forward (- = behind), y = right, in cm from the rover centre.
#include <Arduino.h>
#include "../sensors/Sensors.h"

struct PoleSeen {
  bool isFound;
  float xCm, yCm;   // pole centre
  float diameterCm;      // fitted diameter (cm)
  float fitErrorCm;    // circle fit error (cm)
};

struct WallSeen {
  bool isFound;
  float angleDeg;  // + = we point to the left of the wall direction (it comes closer ahead)
  float distanceCm;    // perpendicular distance from the rover centre
  float fitErrorCm;       // line fit error (cm)
};

namespace Perception {

// Closest point ahead inside a lane as wide as the rover (+ margin, CORRIDOR_*). People or rovers
// beside us are ignored. Forward distance in cm, 0 = lane clear.
uint16_t corridorCm(const Sensors& sensors);

// Next pole on the right: a short, free-standing arc of points whose fitted circle matches the
// pole diameter (POLE_*). Keeps the nearest one ahead or beside us; one behind only if nothing else.
PoleSeen detectPoleRight(const Sensors& sensors);

// Straight wall on the left (235..305 deg): line fit, outliers (people, poles) dropped, refit.
WallSeen detectWallLeft(const Sensors& sensors);

}  // namespace Perception
