#ifndef __VF_H__
#define __VF_H__

#include <stdint.h>

#include "./common_types.h"  // Include common types

// V/f 控制器结构体
typedef struct {
  float target_frequency;   // 目标频率 (Hz)
  float target_voltage;     // 目标电压 (V)
  float current_frequency;  // 当前频率 (Hz)
  float current_voltage;    // 当前电压 (V)
  float max_frequency;      // 最大频率 (Hz)
  float min_frequency;      // 最小频率 (Hz)
  float max_voltage;        // 最大电压 (V)
  float voltage_per_hz;     // V/f 比率
  uint32_t pwm_frequency;   // PWM 频率 (Hz)
  float angle;              // 当前输出角度 (rad)
  float Ts;                 // 采样时间 (s)
  float ramp_time_s;        // 从0到max_frequency的斜坡时间 (秒)
} vf_controller_t;

// 初始化 V/f 控制器
void vf_init(vf_controller_t *vf, float max_freq, float min_freq,
             float max_volt, float pwm_freq, float ts, float ramp_time_seconds);

// V/f 控制器步进函数
// 输入: vf 控制器指针, 目标频率 (Hz)
// 输出: 无 (直接修改 PWM 占空比或输出电压和角度)
void vf_step(vf_controller_t *vf, float target_freq);

// 获取当前输出电压的三个相位值 (Ua, Ub, Uc)
// 通常在 vf_step 内部计算，并可能通过其他方式输出到 PWM 模块
// 这里我们假设它会返回电压的 alpha 和 beta 分量，以便 SVPWM 使用

voltage_vector_t vf_get_voltage_vector(vf_controller_t *vf);
float vf_get_angle(vf_controller_t *vf);

#endif