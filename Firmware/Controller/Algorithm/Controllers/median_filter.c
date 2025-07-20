#include "median_filter.h"

#include <stdlib.h>
#include <string.h>

// Helper function for sorting, used by qsort
static int compare_floats(const void *a, const void *b) {
  float fa = *(const float *)a;
  float fb = *(const float *)b;
  return (fa > fb) - (fa < fb);
}

void median_filter_init(median_filter_t *filter) {
  memset(filter->buffer, 0, sizeof(filter->buffer));
  filter->index = 0;
  filter->count = 0;
}

float median_filter_step(median_filter_t *filter, float input) {
  // Add new value to the circular buffer
  filter->buffer[filter->index] = input;
  filter->index = (filter->index + 1) % MEDIAN_FILTER_SIZE;

  if (filter->count < MEDIAN_FILTER_SIZE) {
    filter->count++;
  }

  // To find the median, we need a copy of the buffer to sort
  float sorted_buffer[MEDIAN_FILTER_SIZE];
  memcpy(sorted_buffer, filter->buffer, filter->count * sizeof(float));

  // Sort the copy
  qsort(sorted_buffer, filter->count, sizeof(float), compare_floats);

  // Return the median value
  return sorted_buffer[filter->count / 2];
}
