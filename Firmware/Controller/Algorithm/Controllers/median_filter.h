#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include <stdint.h>

#define MEDIAN_FILTER_SIZE 3

typedef struct {
  float buffer[MEDIAN_FILTER_SIZE];
  uint8_t index;
  uint8_t count;
} median_filter_t;

void median_filter_init(median_filter_t *filter);
float median_filter_step(median_filter_t *filter, float input);

#endif  // MEDIAN_FILTER_H
