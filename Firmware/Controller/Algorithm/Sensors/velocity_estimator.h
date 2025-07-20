#ifndef __VELOCITY_ESTIMATOR_H__
#define __VELOCITY_ESTIMATOR_H__

#include <stdint.h>

#include "../Controllers/lowpass_filter.h"
#include "../Controllers/median_filter.h"

typedef struct {
  lowpass_filter_t lpf;
  float pos_prev;
  int32_t lap_count;
  uint32_t last_tick;
} velocity_estimator_t;

/**
 * @brief Initializes the velocity estimator.
 * @param alpha The filter coefficient for the low-pass filter.
 * @param current_pos The initial position.
 * @param dt The fixed time step in seconds.
 */
void velocity_estimator_init(float alpha, float current_pos);

/**
 * @brief Updates the velocity estimation with a new position measurement.
 * @param current_pos The new position measurement.
 * @return The latest estimated velocity.
 */
float velocity_estimator_update(float current_pos);

/**
 * @brief Gets the current estimated velocity.
 * @return The estimated velocity.
 */
float velocity_estimator_get_velocity(void);

/**
 * @brief Gets the total accumulated position, including laps.
 * @return The total accumulated position.
 */
float velocity_estimator_get_total_position(void);

void velocity_estimator_set_alpha(float alpha);

#endif