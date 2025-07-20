#include "./hall.h"

#include "dev_usart.h"
#include "tim.h"

#define PI 3.14159265358979

/// 极对数
#define PP 4

static float g_phase_shift_angle = 240.0 / 360.0 * 2.0 * PI;
static float g_theta_delta = 0.0;
static float g_theta = 0.0;
static float g_speed = 0.0;

static float g_hall_value = 0.0;

void hall_init(void) { HAL_TIMEx_HallSensor_Start_IT(&htim3); }

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (TIM3 == htim->Instance) {
    g_hall_value = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
    g_theta_delta = (PI / 3) / (g_hall_value / 10000000) / 10000;
    g_speed = (PI / 3) / (g_hall_value / 10000000) * 30 / (4 * PI);
    uint8_t state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    state |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) << 1;
    state |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) << 2;

    if (state == 0x05) {
      g_theta = 0.0 + g_phase_shift_angle;
    } else if (state == 0x04) {
      g_theta = (PI / 3.0) + g_phase_shift_angle;
    } else if (state == 0x06) {
      g_theta = (PI * 2.0 / 3.0f) + g_phase_shift_angle;
    } else if (state == 0x02) {
      g_theta = PI + g_phase_shift_angle;
    } else if (state == 0x03) {
      g_theta = (PI * 4.0f / 3.0f) + g_phase_shift_angle;
    } else if (state == 0x01) {
      g_theta = (PI * 5.0 / 3.0f) + g_phase_shift_angle;
    }
    if (g_theta < 0.0f) {
      g_theta += 2.0f * PI;
    } else if (g_theta > (2.0f * PI)) {
      g_theta -= 2.0f * PI;
    }
  }
}

void hall_step(void) {
  g_theta = g_theta + g_theta_delta;
  if (g_theta < 0.0f) {
    g_theta += 2.0f * PI;
  } else if (g_theta > (2.0f * PI)) {
    g_theta -= 2.0 * PI;
  }
}

float hall_get_theta(void) { return g_theta; }
float hall_get_speed(void) { return g_speed; }