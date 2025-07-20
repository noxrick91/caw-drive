#include "if.h"

#include <math.h>

#include "../Controllers/pid.h"
#include "../Utils/log.h"
#include "math_const.h"

// 为电流幅值环路提供静态PI控制器实例
static pid_t pid_i_mag;

/**
 * @brief 初始化I/f控制器。
 *
 * @param p_if 指向I/f控制器结构的指针。
 * @param dt 时间步长（秒）。
 */
void if_init(if_controller_t *p_if, float dt) {
  p_if->angle = 0.0f;
  p_if->freq_sp = 0.0f;
  p_if->i_mag_sp = 0.0f;
  p_if->v_mag_sp = 0.0f;
  p_if->dt = dt;

  // 初始化电流幅值环路的PI控制器。
  // Kp, Ki, Kd 最初设置为0，稍后进行配置。
  // out_max 应设置为最大允许电压幅值。
  // 假设一个合理的默认值，如12.0f（伏特），但这
  // 应根据电机和电源进行调整。
  pid_init(&pid_i_mag, 0.0f, 0.0f, 0.0f, 12.0f, 0.0f);
}

/**
 * @brief 设置电流控制器的PI参数。
 *
 * @param kp 比例增益。
 * @param ki 积分增益。
 */
void if_set_pi_params(float kp, float ki) {
  // 可以对理论计算出的PI增益应用一个降额因子
  // 以提高实际系统的稳定性，因为实际系统存在模型未捕获的非线性。
  // 0.5是一个常见的起始值。
  const float K_DETUNE_FACTOR = 0.5f;

  float kp_detuned = kp * K_DETUNE_FACTOR;
  float ki_detuned = ki * K_DETUNE_FACTOR;

  pid_set_param(&pid_i_mag, kp_detuned, ki_detuned, 0.0f, 50.0f, 0.0f);
  debug("IF PI params set (detuned): Kp=%.2f, Ki=%.2f", kp_detuned, ki_detuned);
}

/**
 * @brief 执行I/f控制环路的一个步骤。
 *
 * @param p_if 指向I/f控制器结构的指针。
 * @param i_mag_sp 电流幅值的设定点（安培）。
 * @param freq_sp 电频率的设定点（赫兹）。
 * @param i_mag_meas 测量的电流幅值（安培）。
 * @param v_alpha 指向用于存储计算出的alpha轴电压的指针。
 * @param v_beta 指向用于存储计算出的beta轴电压的指针。
 */
void if_step(if_controller_t *p_if, float i_mag_sp, float freq_sp,
             float i_mag_meas, float *v_alpha, float *v_beta) {
  // 更新设定点
  p_if->i_mag_sp = i_mag_sp;
  p_if->freq_sp = freq_sp;

  // --- 电流控制环路 ---
  // 这是一个闭环控制器，可调节电压幅值
  // 以达到所需的电流幅值。

  // 1. 计算所需电流与测量电流之间的误差。
  float i_mag_error = p_if->i_mag_sp - i_mag_meas;

  // 2. 使用PI控制器确定必要的电压幅值。
  p_if->v_mag_sp = pid_step(&pid_i_mag, i_mag_error, p_if->dt);

  // --- 角度生成 ---
  // 电压矢量的角度由所需频率确定，
  // 从而产生旋转磁场。

  // 1. 对频率进行积分以获得角度。
  p_if->angle += TWO_PI * p_if->freq_sp * p_if->dt;

  // 2. 将角度限制在[0, 2*PI]范围内以防止溢出。
  if (p_if->angle > TWO_PI) {
    p_if->angle -= TWO_PI;
  } else if (p_if->angle < 0.0f) {
    p_if->angle += TWO_PI;
  }

  // --- 反Park变换 ---
  // 将极坐标（电压幅值，角度）转换为
  // 用于SVPWM调制器的笛卡尔坐标（V_alpha, V_beta）。
  *v_alpha = p_if->v_mag_sp * cosf(p_if->angle);
  *v_beta = p_if->v_mag_sp * sinf(p_if->angle);
}
