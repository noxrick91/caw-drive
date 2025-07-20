#ifndef __LOWPASS_FILTER_H__
#define __LOWPASS_FILTER_H__

#include <stdint.h>

/**
 * @brief Structure for a first-order low-pass filter.
 */
typedef struct {
  float alpha;  // Filter coefficient (related to time constant and sample time)
  float y_prev;  // Previous filtered output
} lowpass_filter_t;

/**
 * @brief Initializes the low-pass filter.
 * @param filter Pointer to the lowpass_filter_t struct.
 * @param alpha The filter coefficient, calculated as T_s / (T_s + tau),
 *              where T_s is the sampling time and tau is the time constant.
 *              A smaller alpha gives stronger filtering.
 */
void lowpass_filter_init(lowpass_filter_t *filter, float alpha);

/**
 * @brief Resets the filter's state to a specific value.
 * @param filter Pointer to the lowpass_filter_t struct.
 * @param value The value to reset the filter's output to.
 */
void lowpass_filter_reset(lowpass_filter_t *filter, float value);

/**
 * @brief Applies the low-pass filter to an input value.
 * @param filter Pointer to the lowpass_filter_t struct.
 * @param x The raw input value.
 * @return The filtered output value.
 */
float lowpass_filter_step(lowpass_filter_t *filter, float x);

#endif  // __LOWPASS_FILTER_H__