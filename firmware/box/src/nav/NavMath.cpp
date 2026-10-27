#include "NavMath.h"
#include <math.h>

float angleDiff(float angleDeg, float referenceDeg) {
  float diffDeg = fmodf(angleDeg - referenceDeg + 180.0f, 360.0f);
  if (diffDeg < 0) diffDeg += 360.0f;
  return diffDeg - 180.0f;
}

int medianInt(const int* values, int count) {
  int sorted[15];
  if (count > 15) count = 15;
  for (int i = 0; i < count; i++) sorted[i] = values[i];
  for (int i = 1; i < count; i++)  // insertion sort, n is tiny
    for (int k = i; k > 0 && sorted[k - 1] > sorted[k]; k--) { int swap = sorted[k]; sorted[k] = sorted[k - 1]; sorted[k - 1] = swap; }
  return sorted[count / 2];
}

bool fitCircle(const float* pointsX, const float* pointsY, int count, float& centerX, float& centerY, float& radius, float& rmse) {
  double meanX = 0, meanY = 0;
  for (int i = 0; i < count; i++) { meanX += pointsX[i]; meanY += pointsY[i]; }
  meanX /= count; meanY /= count;
  double sumUU = 0, sumVV = 0, sumUV = 0, sumUZ = 0, sumVZ = 0, sumZ = 0;
  for (int i = 0; i < count; i++) {
    const double offsetU = pointsX[i] - meanX, offsetV = pointsY[i] - meanY, squaredDistance = offsetU * offsetU + offsetV * offsetV;
    sumUU += offsetU * offsetU; sumVV += offsetV * offsetV; sumUV += offsetU * offsetV; sumUZ += offsetU * squaredDistance; sumVZ += offsetV * squaredDistance; sumZ += squaredDistance;
  }
  const double determinant = sumUU * sumVV - sumUV * sumUV;  // centred: the 3x3 system reduces to 2x2
  if (fabs(determinant) < 1e-6) return false;
  const double twiceCenterU = (sumUZ * sumVV - sumVZ * sumUV) / determinant, twiceCenterV = (sumVZ * sumUU - sumUZ * sumUV) / determinant;  // = 2*centre (u,v)
  const double centerU = twiceCenterU / 2, centerV = twiceCenterV / 2;
  radius = sqrt(centerU * centerU + centerV * centerV + sumZ / count);
  centerX = centerU + meanX; centerY = centerV + meanY;
  double squaredErrorSum = 0;
  for (int i = 0; i < count; i++) { const double residual = hypot(pointsX[i] - centerX, pointsY[i] - centerY) - radius; squaredErrorSum += residual * residual; }
  rmse = sqrt(squaredErrorSum / count);
  return true;
}
