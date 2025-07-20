#include "./transform.h"

#include <math.h>

#include "../math_const.h"

void clark(float a, float b, float c, float* ret) {
  ret[0] = TWO_THIRD * (a - 0.5f * b - 0.5f * c);
  ret[1] = TWO_THIRD * (SQRT3_DIV2 * b - SQRT3_DIV2 * c);
}

void iclark(float alpha, float beta, float* ret) {
  ret[0] = alpha;
  ret[1] = -0.5f * alpha + SQRT3_DIV2 * beta;
  ret[2] = -0.5f * alpha - SQRT3_DIV2 * beta;
}

void park(float alpha, float beta, float theta, float* ret) {
  float cos_theta = cosf(theta);
  float sin_theta = sinf(theta);
  ret[0] = alpha * cos_theta + beta * sin_theta;
  ret[1] = -alpha * sin_theta + beta * cos_theta;
}

void ipark(float d, float q, float theta, float* ret) {
  float cos_theta = cosf(theta);
  float sin_theta = sinf(theta);
  ret[0] = d * cos_theta - q * sin_theta;
  ret[1] = d * sin_theta + q * cos_theta;
}