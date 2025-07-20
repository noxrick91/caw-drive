#ifndef __PWM_H__
#define __PWM_H__

#include <stdint.h>

#include "tim.h"

#define PWM_CH1 0x00000000U
#define PWM_CH2 0x00000004U
#define PWM_CH3 0x00000008U

void pwm_init(void);
void pwm_start(void);
void pwm_stop(void);
void pwm_set_duty_cycle(float duty_a, float duty_b, float duty_c);

#define PWM_SET_COMPARE(__CHANNEL__, __COMPARE__)                            \
  (((__CHANNEL__) == PWM_CH1)   ? ((&htim1)->Instance->CCR1 = (__COMPARE__)) \
   : ((__CHANNEL__) == PWM_CH2) ? ((&htim1)->Instance->CCR2 = (__COMPARE__)) \
   : ((__CHANNEL__) == PWM_CH3) ? ((&htim1)->Instance->CCR3 = (__COMPARE__)) \
                                : ((&htim1)->Instance->CCR4 = (__COMPARE__)))

#endif