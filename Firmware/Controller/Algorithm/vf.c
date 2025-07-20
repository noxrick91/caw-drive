#include "vf.h"

#include <math.h>

#include "../Utils/log.h"
#include "math_const.h"  // 确保包含了 PI 的定义

// 初始化 V/f 控制器
void vf_init(vf_controller_t *vf, float max_freq, float min_freq,
             float max_volt, float pwm_freq, float ts,
             float ramp_time_seconds) {
  vf->max_frequency = max_freq;
  vf->min_frequency = min_freq;  // 通常可以设为0或一个较小的值
  vf->max_voltage = max_volt;
  vf->voltage_per_hz = max_volt / max_freq;  // 基本 V/f 比率
  vf->pwm_frequency = pwm_freq;
  vf->Ts = ts;
  vf->current_frequency = 0.0f;
  vf->current_voltage = 0.0f;
  vf->target_frequency = 0.0f;
  vf->target_voltage = 0.0f;
  vf->angle = 0.0f;
  if (ramp_time_seconds <= 0.0f) {
    vf->ramp_time_s = 1.0f;
  } else {
    vf->ramp_time_s = ramp_time_seconds;
  }
}

// V/f 控制器步进函数
void vf_step(vf_controller_t *vf, float target_freq) {
  // 修正: 实现一个平滑的频率斜坡, 而不是错误的瞬时跳变
  const float FREQUENCY_RAMP_INCREMENT =
      0.05f;  // 每次中断调用时频率增加的量 (Hz)

  // 如果当前频率低于目标频率, 则缓慢增加
  if (vf->current_frequency < target_freq) {
    vf->current_frequency += FREQUENCY_RAMP_INCREMENT;
    if (vf->current_frequency > target_freq) {
      vf->current_frequency = target_freq;
    }
  }
  // 如果当前频率高于目标频率, 则缓慢降低
  else if (vf->current_frequency > target_freq) {
    vf->current_frequency -= FREQUENCY_RAMP_INCREMENT;
    if (vf->current_frequency < target_freq) {
      vf->current_frequency = target_freq;
    }
  }

  // 限制频率在安全范围内
  if (vf->current_frequency > vf->max_frequency) {
    vf->current_frequency = vf->max_frequency;
  }
  if (vf->current_frequency < vf->min_frequency) {
    vf->current_frequency = vf->min_frequency;
  }

  vf->target_frequency =
      vf->current_frequency;  // 更新目标频率为当前（可能平滑过的）频率

  // 根据 V/f 曲线计算目标电压
  // 简单线性 V/f
  vf->target_voltage = vf->target_frequency * vf->voltage_per_hz;

  // 电压钳位
  if (vf->target_voltage > vf->max_voltage) {
    vf->target_voltage = vf->max_voltage;
  }
  // 低频电压提升（可选，用于补偿定子电阻压降）
  // if (vf->target_frequency < SOME_LOW_FREQ_THRESHOLD &&
  // vf->target_frequency > 0) {
  //     vf->target_voltage += VOLTAGE_BOOST;
  //     if (vf->target_voltage > vf->max_voltage) {
  //         vf->target_voltage = vf->max_voltage;
  //     }
  // }

  // 更新角度
  // angle = integral(omega) = integral(2*pi*f)
  vf->angle += 2.0f * PI * vf->target_frequency * vf->Ts;
  if (vf->angle > (2.0f * PI)) {
    vf->angle -= (2.0f * PI);
  } else if (vf->angle < 0.0f) {  // 虽然通常是正向增加，但以防万一
    vf->angle += (2.0f * PI);
  }
  vf->current_voltage = vf->target_voltage;  // 更新当前电压
}

// 获取当前输出电压的三个相位值 (Ua, Ub, Uc)
// 或者返回 alpha, beta 分量给 SVPWM
voltage_vector_t vf_get_voltage_vector(vf_controller_t *vf) {
  voltage_vector_t v_vector;
  // 计算 alpha 和 beta 分量
  // Valpha = V * cos(angle)
  // Vbeta  = V * sin(angle)
  // 注意：这里的 V 是相电压峰值。如果 SVPWM
  // 需要线电压或不同的参考，需要调整。 假设 vf->current_voltage
  // 是目标相电压的幅值。
  v_vector.alpha = vf->current_voltage * cosf(vf->angle);
  v_vector.beta = vf->current_voltage * sinf(vf->angle);
  return v_vector;
}

float vf_get_angle(vf_controller_t *vf) { return vf->angle; }