#ifndef VELOCITY_ESTIMATOR_H
#define VELOCITY_ESTIMATOR_H

// This file provides an interface for estimating velocity from a position
// sensor. It uses a position difference method combined with a low-pass filter.

/**
 * @brief Initializes the velocity estimator.
 *        This must be called before any other functions in this module.
 * @param alpha The filter coefficient for the internal low-pass filter.
 *              Calculated as T_s / (T_s + tau), where T_s is the sampling time
 *              and tau is the filter time constant.
 * @param current_pos The initial position from the sensor (in radians).
 * @param sample_time The fixed time interval (dt) between updates, in seconds.
 *                    This should be the period of the control loop.
 */
void velocity_estimator_init(float alpha, float current_pos, float sample_time);

/**
 * @brief Updates the velocity estimation based on a new position reading.
 *        This function should be called at a fixed frequency, corresponding to
 *        the sample_time provided in the init function.
 * @param current_pos The current position from the sensor (in radians).
 * @return The new filtered velocity in rad/s.
 */
float velocity_estimator_update(float current_pos);

/**
 * @brief Gets the latest estimated velocity without performing a new
 * calculation.
 * @return The last calculated filtered velocity in rad/s.
 */
float velocity_estimator_get_velocity(void);

/**
 * @brief Gets the total accumulated position, tracking rollovers (laps).
 * @return The total continuous position in radians.
 */
float velocity_estimator_get_total_position(void);

/**
 * @brief Updates the alpha value for the low-pass filter on-the-fly.
 * @param alpha The new filter coefficient.
 */
void velocity_estimator_set_alpha(float alpha);

#endif  // VELOCITY_ESTIMATOR_H
