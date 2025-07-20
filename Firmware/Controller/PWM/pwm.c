#include "./pwm.h"

void pwm_init(void) {
  TIM1->ARR = 8000 - 1;
  TIM1->CCR4 = 8000 - 2;
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void pwm_start(void) {
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void pwm_stop(void) {
  uint32_t half_period = (TIM1->ARR + 1) / 2;
  TIM1->CCR1 = half_period;
  TIM1->CCR2 = half_period;
  TIM1->CCR3 = half_period;

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void pwm_set_duty_cycle(float duty_a, float duty_b, float duty_c) {
  TIM1->CCR1 = (uint32_t)(duty_a * (float)(TIM1->ARR + 1));
  TIM1->CCR2 = (uint32_t)(duty_b * (float)(TIM1->ARR + 1));
  TIM1->CCR3 = (uint32_t)(duty_c * (float)(TIM1->ARR + 1));
}