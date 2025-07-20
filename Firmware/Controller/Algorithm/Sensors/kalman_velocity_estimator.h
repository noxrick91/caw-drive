#ifndef __KALMAN_VELOCITY_ESTIMATOR_H__
#define __KALMAN_VELOCITY_ESTIMATOR_H__

#include <stdint.h>

typedef struct {
  // State vector [angle, velocity]
  float x[2];

  // State covariance matrix
  float P[2][2];

  // Process noise covariance (tuning parameters)
  float q_angle;
  float q_velocity;

  // Measurement noise covariance (tuning parameter)
  float r_angle;

} kalman_velocity_estimator_t;

/**
 * @brief Initializes the Kalman filter based velocity estimator.
 *
 * @param self Pointer to the Kalman filter instance.
 * @param q_angle Process noise for angle. Represents the uncertainty of the
 * model for position.
 * @param q_velocity Process noise for velocity. Represents the uncertainty of
 * the model for velocity (e.g., random accelerations).
 * @param r_angle Measurement noise for angle. Represents the uncertainty of the
 * encoder measurement.
 * @param initial_pos The initial position measurement from the encoder.
 */
void kalman_velocity_estimator_init(kalman_velocity_estimator_t* self,
                                    float q_angle, float q_velocity,
                                    float r_angle, float initial_pos);

/**
 * @brief Updates the velocity estimation with a new position measurement.
 *
 * @param self Pointer to the Kalman filter instance.
 * @param current_pos The new position measurement from the encoder.
 * @param dt The time step since the last update, in seconds.
 * @return The latest estimated velocity in rad/s.
 */
float kalman_velocity_estimator_update(kalman_velocity_estimator_t* self,
                                       float current_pos, float dt);

/**
 * @brief Gets the current estimated velocity.
 *
 * @param self Pointer to the Kalman filter instance.
 * @return The estimated velocity in rad/s.
 */
float kalman_velocity_estimator_get_velocity(kalman_velocity_estimator_t* self);

/**
 * @brief Gets the current estimated angle.
 *
 * @param self Pointer to the Kalman filter instance.
 * @return The estimated angle in radians.
 */
float kalman_velocity_estimator_get_angle(kalman_velocity_estimator_t* self);

#endif  // __KALMAN_VELOCITY_ESTIMATOR_H__