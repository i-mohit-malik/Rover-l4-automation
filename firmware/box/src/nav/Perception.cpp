#include "Perception.h"
#include <math.h>
#include "NavConfig.h"
#include "NavMath.h"

namespace Perception {

uint16_t corridorCm(const Sensors& sensors) {
  uint16_t nearestCm = 0;
  for (int angleDeg = -45; angleDeg <= 45; angleDeg++) {  // +-45 deg covers the corridor down to the 20 cm LiDAR minimum
    const uint16_t distanceCm = sensors.lidarCm(angleDeg);
    if (distanceCm == 0) continue;
    const float forwardCm = distanceCm * cosf(angleDeg * DEG_TO_RAD);  // forward
    const float sideCm = distanceCm * sinf(angleDeg * DEG_TO_RAD);  // sideways
    if (forwardCm > CORRIDOR_LOOKAHEAD_CM || fabsf(sideCm) > CORRIDOR_HALF_WIDTH_CM) continue;
    if (nearestCm == 0 || forwardCm < nearestCm) nearestCm = (uint16_t)forwardCm;
  }
  return nearestCm;
}

// A pole shows up as a short group of neighbouring LiDAR points (an arc) whose width matches the
// pole diameter and that stands free in front of the background. A wall gives a much wider group.
PoleSeen detectPoleRight(const Sensors& sensors) {
  PoleSeen bestPole = {false, 0, 0, 0, 0};
  const int kMaxPoints = 64;
  float pointsX[kMaxPoints], pointsY[kMaxPoints];
  int pointCount = 0, firstDeg = 0, lastDeg = 0;
  float lastDistanceCm = 0, minDistanceCm = 0, firstX = 0, firstY = 0, lastX = 0, lastY = 0;

  // Checks the finished group and keeps it if it looks like a pole: right width, free-standing,
  // and the points lie on a circle of the pole's diameter.
  auto finishGroup = [&]() {
    if (pointCount < 4) return;
    const float groupWidthCm = hypotf(lastX - firstX, lastY - firstY);
    if (groupWidthCm < POLE_DIAMETER_CM * 0.4f || groupWidthCm > POLE_DIAMETER_CM * 1.5f) return;
    // free-standing: just outside the group there is nothing, or something clearly farther away
    const uint16_t beforeCm = sensors.lidarCm(firstDeg - 2), afterCm = sensors.lidarCm(lastDeg + 2);
    if (beforeCm && beforeCm < minDistanceCm + 30) return;
    if (afterCm && afterCm < minDistanceCm + 30) return;
    // round like the pole: fitted diameter close to the real one, small fit error, and the circle
    // centre behind the points (a round surface facing us, not a flat or hollow shape)
    float centerX, centerY, radius, rmse;
    if (!fitCircle(pointsX, pointsY, pointCount, centerX, centerY, radius, rmse)) return;
    if (POLE_SHAPE_CHECK) {
      if (2 * radius < POLE_DIAMETER_CM * POLE_DIAMETER_MIN || 2 * radius > POLE_DIAMETER_CM * POLE_DIAMETER_MAX) return;
      if (rmse > POLE_FIT_MAX_CM) return;
      if (hypotf(centerX, centerY) < minDistanceCm + 0.5f * radius) return;
    }
    if (centerY < POLE_MIN_RIGHT_CM || centerY > POLE_MAX_RIGHT_CM) return;
    // keep the next pole: the nearest one ahead or beside us (x > -30); one behind only if nothing else
    const bool isAhead = centerX > -30, isBestAhead = bestPole.isFound && bestPole.xCm > -30;
    if (!bestPole.isFound || (isAhead && (!isBestAhead || centerX < bestPole.xCm)) || (!isAhead && !isBestAhead && centerX > bestPole.xCm))
      bestPole = {true, centerX, centerY, 2 * radius, rmse};
  };

  for (int angleDeg = 20; angleDeg <= 164; angleDeg++) {  // right side: 90 = straight right
    const uint16_t distanceCm = (angleDeg <= 160) ? sensors.lidarCm(angleDeg) : 0;
    const bool isValid = distanceCm > 0 && distanceCm < 300;
    if (!isValid) {
      if (pointCount > 0 && angleDeg - lastDeg > 3) { finishGroup(); pointCount = 0; }  // allow small gaps (missing returns)
      continue;
    }
    if (pointCount > 0 && (angleDeg - lastDeg > 3 || fabsf(distanceCm - lastDistanceCm) >= 15)) { finishGroup(); pointCount = 0; }
    const float pointX = distanceCm * cosf(angleDeg * DEG_TO_RAD), pointY = distanceCm * sinf(angleDeg * DEG_TO_RAD);
    if (pointCount == 0) { firstDeg = angleDeg; firstX = pointX; firstY = pointY; minDistanceCm = distanceCm; }
    if (pointCount < kMaxPoints) { pointsX[pointCount] = pointX; pointsY[pointCount] = pointY; }
    pointCount = min(pointCount + 1, kMaxPoints);
    lastX = pointX; lastY = pointY;
    lastDeg = angleDeg; lastDistanceCm = distanceCm;
    if (distanceCm < minDistanceCm) minDistanceCm = distanceCm;
  }
  if (pointCount > 0) finishGroup();
  return bestPole;
}

// Fits y = m*x + b to the points on the left (x forward, y right, so the wall has y < 0).
WallSeen detectWallLeft(const Sensors& sensors) {
  WallSeen wall = {false, 0, 0, 0};
  const int kMaxPoints = 80;
  float pointsX[kMaxPoints], pointsY[kMaxPoints];
  int pointCount = 0;
  for (int angleDeg = 235; angleDeg <= 305 && pointCount < kMaxPoints; angleDeg++) {  // left: 270 = straight left
    const uint16_t distanceCm = sensors.lidarCm(angleDeg);
    if (distanceCm == 0 || distanceCm > WALL_MAX_CM) continue;
    pointsX[pointCount] = distanceCm * cosf(angleDeg * DEG_TO_RAD);
    pointsY[pointCount] = distanceCm * sinf(angleDeg * DEG_TO_RAD);
    pointCount++;
  }
  float slope = 0, intercept = 0, rmse = 0;
  for (int pass = 0; pass < 2; pass++) {
    if (pointCount < WALL_MIN_POINTS) return wall;
    double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
    for (int i = 0; i < pointCount; i++) { sumX += pointsX[i]; sumY += pointsY[i]; sumXX += pointsX[i] * pointsX[i]; sumXY += pointsX[i] * pointsY[i]; }
    const double denominator = pointCount * sumXX - sumX * sumX;
    if (denominator < 1e-3) return wall;
    slope = (pointCount * sumXY - sumX * sumY) / denominator;
    intercept = (sumY - slope * sumX) / pointCount;
    double squaredErrorSum = 0;
    for (int i = 0; i < pointCount; i++) { const double residual = pointsY[i] - (slope * pointsX[i] + intercept); squaredErrorSum += residual * residual; }
    rmse = sqrt(squaredErrorSum / pointCount);
    if (pass == 0) {  // drop points far off the line, then refit
      int keptCount = 0;
      const float outlierLimitCm = max(5.0f, 2.0f * rmse);
      for (int i = 0; i < pointCount; i++)
        if (fabsf(pointsY[i] - (slope * pointsX[i] + intercept)) <= outlierLimitCm) { pointsX[keptCount] = pointsX[i]; pointsY[keptCount] = pointsY[i]; keptCount++; }
      pointCount = keptCount;
    }
  }
  float minX = pointsX[0], maxX = pointsX[0];
  for (int i = 1; i < pointCount; i++) { minX = min(minX, pointsX[i]); maxX = max(maxX, pointsX[i]); }
  if (maxX - minX < WALL_MIN_SPAN_CM || rmse > WALL_FIT_MAX_CM || intercept >= 0) return wall;
  wall.isFound = true;
  wall.angleDeg = atanf(slope) * RAD_TO_DEG;
  wall.distanceCm = -intercept * cosf(atanf(slope));  // perpendicular distance to the line
  wall.fitErrorCm = rmse;
  return wall;
}

}  // namespace Perception
