#include "./params.h"

#include "../../Utils/log.h"
#include "../../math_const.h"

/**
 * @brief 根据电机参数计算电流环PI控制器的连续域增益。
 *        此函数返回的是连续域参数，后续的离散化应由PID执行器（如pid_step）通过乘以dt来完成。
 *
 * @param Rs 电机相电阻 (Ohms)
 * @param L 电机相电感 (Henry)
 * @param Tpwm PWM周期 (Seconds), 仅用于经典方法中的经验计算
 * @param Kp 指向Kp输出值的指针
 * @param Ki 指向Ki输出值的指针
 * @param method 计算方法选择 (经典经验法则或基于带宽)
 * @param bandwidth_hz 目标闭环带宽 (Hz), 仅在BANDWIDTH方法下使用
 */
void calc_current_pi(float Rs, float L, float Tpwm, float* Kp, float* Ki,
                     calc_current_pi_method_e method, float bandwidth_hz) {
  // 安全检查
  if (Tpwm <= 1e-6f || L <= 1e-9f || Rs <= 1e-6f) {
    error("calc_current_pi: Invalid input params. Rs=%.4f, L=%.4e, Tpwm=%.4e",
          Rs, L, Tpwm);
    *Kp = 0.0f;
    *Ki = 0.0f;
    return;
  }

  if (method == CALC_CURRENT_PI_METHOD_BANDWIDTH &&
      (bandwidth_hz <= 10.0f || bandwidth_hz > (1.0f / Tpwm) / 5.0f)) {
    error(
        "calc_current_pi: Invalid bandwidth_hz (%.1f). Must be in (10, "
        "%.1f)",
        bandwidth_hz, (1.0f / Tpwm) / 5.0f);
    *Kp = 0.0f;
    *Ki = 0.0f;
    return;
  }

  float Kp_continuous = 0.0f;
  float Ki_continuous = 0.0f;

  if (method == CALC_CURRENT_PI_METHOD_CLASSIC) {
    // 方法1：经典经验法则 (不推荐，仅作备用)
    // 这是一种简化的经验方法，通常将控制器极点设置在远低于采样频率的地方以保证稳定。
    // 这里的 "10.0f" 是一个经验系数。
    Kp_continuous = L / (10.0f * Tpwm);
    Ki_continuous = Rs / (10.0f * Tpwm);
  } else if (method == CALC_CURRENT_PI_METHOD_BANDWIDTH) {
    // 方法2：基于期望带宽的极点零点对消法 (推荐)
    // 这是更常用和更精确的方法。
    // 1. 将目标带宽从Hz转换为rad/s
    float wc = 2.0f * PI * bandwidth_hz;
    // 2. 计算Kp和Ki以对消电机的极点(s = -Rs/L)，并达到期望的闭环带宽。
    //    Kp = L * wc
    //    Ki = Rs * wc
    Kp_continuous = L * wc;
    Ki_continuous = Rs * wc;
  } else {
    // 未知方法或其他错误情况，返回0
    error("calc_current_pi: Unknown method: %d", method);
    *Kp = 0.0f;
    *Ki = 0.0f;
    return;
  }

  // 直接输出计算出的连续域参数。
  // 离散化（即Ki乘以dt）将在pid_step函数中通过 `integral += error * dt`
  // 的方式隐式完成。
  *Kp = Kp_continuous;
  *Ki = Ki_continuous;

  info("calc_current_pi: OK. Rs=%.3f, L=%.3e, BW=%.1fHz -> Kp=%.3f, Ki=%.3f",
       Rs, L, bandwidth_hz, *Kp, *Ki);
}