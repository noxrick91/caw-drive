#include "lowpass_filter.h"

void lowpass_filter_init(lowpass_filter_t *filter, float alpha) {
  if (filter) {
    filter->alpha = alpha;
    filter->y_prev = 0.0f;
  }
}

void lowpass_filter_reset(lowpass_filter_t *filter, float value) {
  if (filter) {
    filter->y_prev = value;
  }
}

float lowpass_filter_step(lowpass_filter_t *filter, float x) {
  if (!filter) {
    return 0.0f;
  }

  // The core filter equation: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
  float y = filter->alpha * x + (1.0f - filter->alpha) * filter->y_prev;

  // Store the current output for the next iteration
  filter->y_prev = y;

  return y;
}