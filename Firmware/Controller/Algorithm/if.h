#ifndef __IF_H__
#define __IF_H__

#include <stdint.h>

typedef struct {
  float angle;     // rad, electrical angle
  float freq_sp;   // Hz, frequency setpoint
  float i_mag_sp;  // A, magnitude of current setpoint
  float v_mag_sp;  // V, magnitude of voltage output
  float dt;        // s, time step
} if_controller_t;

void if_init(if_controller_t *p_if, float dt);
void if_set_pi_params(float kp, float ki);
void if_step(if_controller_t *p_if, float i_mag_sp, float freq_sp,
             float i_mag_meas, float *v_alpha, float *v_beta);

#endif  // IF_H
