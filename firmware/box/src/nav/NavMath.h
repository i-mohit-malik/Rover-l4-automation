#pragma once
// NavMath - small geometry helpers for the autonomous run. No state, no sensors.
#include <Arduino.h>

// a - b wrapped to -180..180 deg.
float angleDiff(float angleDeg, float referenceDeg);

// Median of n ints (n <= 15); used to reject single bad ultrasonic readings.
int medianInt(const int* values, int count);

// Least-squares circle through n points (Kasa fit, centred for accuracy).
// Returns false if the points are (nearly) on a straight line.
bool fitCircle(const float* pointsX, const float* pointsY, int count, float& centerX, float& centerY,
               float& radius, float& rmse);
