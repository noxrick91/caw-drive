#include "velocity_estimator.h"

#include <math.h>
#include <stddef.h>

#include "../../Utils/log.h"  // For debug()
#include "Algorithm/Controllers/lowpass_filter.h"

// Define PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Position rollover value, assuming radians (2 * PI)
#define POS_ROLLOVER (2.0f * M_PI)

// Internal state for the velocity estimator
typedef struct {
  lowpass_filter_t lpf;
  float pos_prev;
  int32_t lap_count;
  float dt;  // Fixed sample time
} velocity_estimator_state_t;

static velocity_estimator_state_t g_estimator;

void velocity_estimator_init(float alpha, float current_pos,
                             float sample_time) {
  lowpass_filter_init(&g_estimator.lpf, alpha);
  g_estimator.pos_prev = current_pos;
  g_estimator.lap_count = 0;
  g_estimator.dt = sample_time;
  lowpass_filter_reset(&g_estimator.lpf, 0.0f);
}

float velocity_estimator_update(float current_pos) {
  // Use the fixed sample time for higher accuracy
  float dt = g_estimator.dt;

  // If dt is invalid, return the last known velocity to prevent division by
  // zero
  if (dt <= 1e-6f) {
    return g_estimator.lpf.y_prev;
  }

  // Handle position rollover (wraparound)
  float delta_pos = current_pos - g_estimator.pos_prev;

  if (delta_pos > (POS_ROLLOVER / 2.0f)) {
    // Rollover backward (e.g., from 0.1 to 6.2 should be detected)
    delta_pos -= POS_ROLLOVER;
    g_estimator.lap_count--;
  } else if (delta_pos < -(POS_ROLLOVER / 2.0f)) {
    // Rollover forward (e.g., from 6.2 to 0.1 should be detected)
    delta_pos += POS_ROLLOVER;
    g_estimator.lap_count++;
  }

  // Calculate instantaneous velocity (rad/s)
  float velocity_raw = delta_pos / dt;

  // Update state for the next iteration
  g_estimator.pos_prev = current_pos;

  // Apply low-pass filter to smooth the raw velocity
  return lowpass_filter_step(&g_estimator.lpf, velocity_raw);
}

float velocity_estimator_get_velocity(void) { return g_estimator.lpf.y_prev; }

float velocity_estimator_get_total_position(void) {
  return g_estimator.pos_prev + (float)g_estimator.lap_count * POS_ROLLOVER;
}

void velocity_estimator_set_alpha(float alpha) {
  // Update the filter coefficient directly
  g_estimator.lpf.alpha = alpha;
}
