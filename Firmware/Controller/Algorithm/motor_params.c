#include "motor_params.h"

#include <math.h>
#include <stm32f4xx_hal.h>

#include "../../config.h"
#include "../Drivers/drv8323/drv8323.h"
#include "../PWM/pwm.h"
#include "../Sensors/current.h"
#include "../Sensors/vbus.h"
#include "../Utils/log.h"
#include "../Utils/time.h"
#include "../config.h"
#include "../controller.h"
#include "../foc.h"
#include "../math_const.h"
#include "../transform.h"

// 辅助函数：对数组进行排序（冒泡排序）
static void bubble_sort(float arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

// 辅助函数：计算数组的中位数
static float calculate_median(float arr[], int n) {
  bubble_sort(arr, n);
  if (n % 2 == 0) {
    return (arr[n / 2 - 1] + arr[n / 2]) / 2.0f;
  } else {
    return arr[n / 2];
  }
}

void motor_params_init(motor_params_t* self, int32_t pp, float test_current,
                       float nominal_rpm) {
  self->phase_resistance = 0.0f;
  self->inductance_d = 0.0f;
  self->inductance_q = 0.0f;
  self->flux_linkage = 0.0f;
  self->pole_pairs = pp;
  self->test_current = test_current;
  self->nominal_mechanical_rpm = nominal_rpm;
  self->B = 1e-6f;
  self->J = 1e-5f;
}

int motor_params_ident_resistance(motor_params_t* self, float test_current) {
  const float voltage_step = 0.5f;
  const uint32_t settle_time_ms = 400;
  const uint32_t timeout_ms = 8000;

  float v_alpha = 0.0f;

  uint32_t start_time = HAL_GetTick();

  int ret = -1;

  while (HAL_GetTick() - start_time < timeout_ms) {
    float pwm_duties[3];
    float vbus = vbus_get();

    v_alpha += voltage_step;
    svpwm_calculate(vbus, v_alpha, 0.0f, pwm_duties, false);
    pwm_set_duty_cycle(pwm_duties[0], pwm_duties[1], pwm_duties[2]);

    HAL_Delay(settle_time_ms);

    // 获取相电流
    float i_ab_measured[2];
    float phase_currents[3];
    current_get(phase_currents);
    clark(phase_currents[0], phase_currents[1], phase_currents[2],
          i_ab_measured);

    float Is = sqrtf(i_ab_measured[0] * i_ab_measured[0] +
                     i_ab_measured[1] * i_ab_measured[1]);
    if (Is > test_current) {
      self->phase_resistance = v_alpha / Is;
      ret = 0;
      break;
    }
  }
  pwm_set_duty_cycle(0.0f, 0.0f, 0.0f);
  return ret;
}

int motor_params_ident_inductance(motor_params_t* self, float test_current) {
  if (self->phase_resistance < 1e-6f) {
    return -1;
  }

  const uint32_t align_time_ms = 400;
  const uint32_t dt_us = 400;
  const float dt_s = (float)dt_us * 1e-6f;
  const int num_measurements = 5;  // 进行5次测量以计算中位数

  // 存储多次测量的Ld值
  float Ld_measurements[num_measurements];
  int valid_measurements = 0;

  // 2. 多次测量Ld
  for (int i = 0; i < num_measurements; i++) {
    // 通过给定电流计算出所需的对齐电压
    float vbus = vbus_get();
    float v_align = test_current * self->phase_resistance;

    if (v_align > vbus * 0.8f) {
      v_align = vbus * 0.8f;
    }

    float v_step = v_align > 0.5f ? 0.5f : v_align;

    float pwm_duties[3];
    svpwm_calculate(vbus, v_align, 0.0f, pwm_duties, false);
    pwm_set_duty_cycle(pwm_duties[0], pwm_duties[1], pwm_duties[2]);
    HAL_Delay(align_time_ms);

    float i_ab_measured[2];
    float phase_currents[3];

    current_get(phase_currents);
    clark(phase_currents[0], phase_currents[1], phase_currents[2],
          i_ab_measured);
    float i_d_before = i_ab_measured[0];

    svpwm_calculate(vbus, v_align + v_step, 0.0f, pwm_duties, false);
    pwm_set_duty_cycle(pwm_duties[0], pwm_duties[1], pwm_duties[2]);
    _delay_us(dt_us);

    current_get(phase_currents);
    clark(phase_currents[0], phase_currents[1], phase_currents[2],
          i_ab_measured);
    float i_d_after = i_ab_measured[0];

    float di_d = i_d_after - i_d_before;
    if (di_d > 1e-3f) {
      float did_dt = di_d / dt_s;
      Ld_measurements[valid_measurements] = v_step / did_dt;
      valid_measurements++;
    }
    HAL_Delay(10);
  }

  // 检查是否有有效的测量结果
  if (valid_measurements > 0) {
    bubble_sort(Ld_measurements, num_measurements);
    float Ld_avg = 0.0f;
    for (int i = 1; i < num_measurements - 1; i++) {
      Ld_avg += Ld_measurements[i];
    }
    Ld_avg /= (num_measurements - 2);
    self->inductance_d = Ld_avg;
    self->inductance_q = self->inductance_d;
  } else {
    pwm_set_duty_cycle(0.0f, 0.0f, 0.0f);
    return -1;
  }

  pwm_set_duty_cycle(0.0f, 0.0f, 0.0f);
  return 0;
}
