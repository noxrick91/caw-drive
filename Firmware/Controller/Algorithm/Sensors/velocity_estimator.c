#include "velocity_estimator.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"

// Define PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Position rollover value, assuming radians (2 * PI)
#define POS_ROLLOVER (2.0f * M_PI)

static velocity_estimator_t g_velocity_estimator;

void velocity_estimator_init(float alpha, float current_pos) {
  lowpass_filter_init(&g_velocity_estimator.lpf, alpha);
  g_velocity_estimator.pos_prev = current_pos;
  g_velocity_estimator.lap_count = 0;
  g_velocity_estimator.last_tick = HAL_GetTick();
  lowpass_filter_reset(&g_velocity_estimator.lpf, 0.0f);
}

float velocity_estimator_update(float current_pos) {
  uint32_t current_tick = HAL_GetTick();
  float dt = (current_tick - g_velocity_estimator.last_tick) / 1000.0f;
  g_velocity_estimator.last_tick = current_tick;

  if (dt <= 1e-6f) {  // Avoid division by zero or invalid dt
    return g_velocity_estimator.lpf.y_prev;  // Return previous velocity
  }

  // Handle position rollover
  float delta_pos = current_pos - g_velocity_estimator.pos_prev;
  if (delta_pos > (POS_ROLLOVER / 2.0f)) {
    // Rollover backward (e.g., from 0.1 to 6.2)
    delta_pos -= POS_ROLLOVER;
    g_velocity_estimator.lap_count--;
  } else if (delta_pos < -(POS_ROLLOVER / 2.0f)) {
    // Rollover forward (e.g., from 6.2 to 0.1)
    delta_pos += POS_ROLLOVER;
    g_velocity_estimator.lap_count++;
  }

  // Calculate instantaneous velocity (rad/s)
  float velocity_raw = delta_pos / dt;

  // Update state for next iteration
  g_velocity_estimator.pos_prev = current_pos;

  // Apply low-pass filter
  return lowpass_filter_step(&g_velocity_estimator.lpf, velocity_raw);
}

float velocity_estimator_get_velocity(void) {
  return g_velocity_estimator.lpf.y_prev;
}

float velocity_estimator_get_total_position(void) {
  return g_velocity_estimator.pos_prev +
         (float)g_velocity_estimator.lap_count * POS_ROLLOVER;
}

void velocity_estimator_set_alpha(float alpha) {
  lowpass_filter_init(&g_velocity_estimator.lpf, alpha);
}