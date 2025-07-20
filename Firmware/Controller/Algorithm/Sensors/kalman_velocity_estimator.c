#include "kalman_velocity_estimator.h"

#include <math.h>
#include <stddef.h>

// Define PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Position rollover value, assuming radians (2 * PI)
#define POS_ROLLOVER (2.0f * M_PI)

void kalman_velocity_estimator_init(kalman_velocity_estimator_t* self,
                                    float q_angle, float q_velocity,
                                    float r_angle, float initial_pos) {
  if (self == NULL) return;

  self->q_angle = q_angle;
  self->q_velocity = q_velocity;
  self->r_angle = r_angle;

  // Initialize state
  self->x[0] = initial_pos;  // angle
  self->x[1] = 0.0f;         // velocity

  // Initialize state covariance matrix P with high uncertainty
  self->P[0][0] = 1.0f;
  self->P[0][1] = 0.0f;
  self->P[1][0] = 0.0f;
  self->P[1][1] = 1.0f;
}

float kalman_velocity_estimator_update(kalman_velocity_estimator_t* self,
                                       float current_pos, float dt) {
  if (self == NULL) return 0.0f;

  if (dt <= 1e-6f) {
    return self->x[1];  // Return previous velocity if dt is too small
  }

  // --- PREDICTION STEP ---

  // State transition model: x_k = F * x_{k-1}
  // x_angle_pred = x_angle + dt * x_velocity
  self->x[0] += dt * self->x[1];

  // Update state covariance matrix: P_pred = F * P * F^T + Q
  // F = [[1, dt], [0, 1]]
  float p00 = self->P[0][0];
  float p01 = self->P[0][1];
  float p10 = self->P[1][0];
  float p11 = self->P[1][1];

  self->P[0][0] = p00 + dt * (p10 + p01) + dt * dt * p11 + self->q_angle;
  self->P[0][1] = p01 + dt * p11;
  self->P[1][0] = p10 + dt * p11;
  self->P[1][1] = p11 + self->q_velocity;

  // --- UPDATE STEP ---

  // Measurement residual (innovation): y = z - H * x_pred
  // H = [1, 0], so y = current_pos - x_pred[0]
  float innovation = current_pos - self->x[0];

  // Handle angle wrap-around for the innovation
  if (innovation > M_PI) {
    innovation -= POS_ROLLOVER;
  } else if (innovation < -M_PI) {
    innovation += POS_ROLLOVER;
  }

  // Innovation covariance: S = H * P_pred * H^T + R
  // S = P_pred[0][0] + R
  float S = self->P[0][0] + self->r_angle;

  // Kalman gain: K = P_pred * H^T * S^-1
  // K = [P_pred[0][0] / S, P_pred[1][0] / S]^T
  float K[2];
  K[0] = self->P[0][0] / S;
  K[1] = self->P[1][0] / S;

  // Update state estimate: x_new = x_pred + K * y
  self->x[0] += K[0] * innovation;
  self->x[1] += K[1] * innovation;

  // Update state covariance: P_new = (I - K * H) * P_pred
  p00 = self->P[0][0];
  p01 = self->P[0][1];

  self->P[0][0] -= K[0] * p00;
  self->P[0][1] -= K[0] * p01;
  self->P[1][0] -= K[1] * p00;
  self->P[1][1] -= K[1] * p01;

  // Ensure estimated angle stays within [0, 2*PI] range
  self->x[0] = fmodf(self->x[0] + POS_ROLLOVER, POS_ROLLOVER);

  return self->x[1];
}

float kalman_velocity_estimator_get_velocity(
    kalman_velocity_estimator_t* self) {
  if (self == NULL) return 0.0f;
  return self->x[1];
}

float kalman_velocity_estimator_get_angle(kalman_velocity_estimator_t* self) {
  if (self == NULL) return 0.0f;
  return self->x[0];
}