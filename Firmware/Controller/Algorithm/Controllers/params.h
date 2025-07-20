#ifndef __PARAMS_H__
#define __PARAMS_H__

typedef enum _calc_current_pi_method_e {
  CALC_CURRENT_PI_METHOD_CLASSIC,
  CALC_CURRENT_PI_METHOD_BANDWIDTH
} calc_current_pi_method_e;

void calc_current_pi(float Rs, float L, float Tpwm, float* Kp, float* Ki,
                     calc_current_pi_method_e method, float bandwidth_hz);

#endif