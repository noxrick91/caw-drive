#ifndef __SVPWM_H__
#define __SVPWM_H__

#include <stdbool.h>

typedef struct _svpwm_t {
  float Tpwm;
} svpwm_t;

void svpwm_init(float Tpwm);
void svpwm_calculate(float Vdc, float alpha, float beta, float* ret,
                     bool zero_seq);

#endif