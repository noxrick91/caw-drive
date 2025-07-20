#ifndef __FOC_H__
#define __FOC_H__

#include "../Controller/Algorithm/Controllers/pid.h"
#include "./Controllers/trajectory_planner.h"
#include "motor_params.h"

// FOC控制模式枚举
typedef enum {
  FOC_CONTROL_MODE_TORQUE,    // 电流/力矩控制
  FOC_CONTROL_MODE_SPEED,     // 速度控制
  FOC_CONTROL_MODE_POSITION,  // 位置控制
  FOC_CONTROL_MODE_VOLTAGE    // 直接电压控制(用于参数辨识)
} foc_control_mode_t;

// FOC所需的所有PID参数
typedef struct {
  pid_t id_pid;        // d轴电流环
  pid_t iq_pid;        // q轴电流环
  pid_t velocity_pid;  // 速度环
  pid_t position_pid;  // 位置环
  float velocity_lpf_tau;
} foc_params_t;

// FOC控制器实例
typedef struct _foc_t {
  volatile foc_control_mode_t control_mode;  // 当前FOC控制模式
  float Tpwm;                                // PWM周期

  // 目标值设定点
  float id_ref;        // d轴电流目标 (通常为0)
  float iq_ref;        // q轴电流目标 (力矩)
  float speed_ref;     // 速度目标 (rpm)
  float position_ref;  // 位置目标 (rad)

  // PID控制器 (从config中获取参数)
  foc_params_t* params;

  // 速度斜坡功能 (用于速度模式)
  float speed_ref_actual;  // 斜坡过程中，当前实际的速度参考值 (rpm)
  float speed_ramp_accel;  // 速度斜坡的加速度 (rpm/s)

  // 轨迹规划器 (用于位置模式)
  trajectory_planner_t trajectory;
} foc_t;

extern foc_t g_foc;  // 全局FOC控制器实例

void foc_init(foc_params_t* params, float Tpwm, float vbus);

void foc_step(float id_meas, float iq_meas, float speed_meas,
              float multi_turn_angle, float* v_d, float* v_q);

// --- PID Parameter Update Functions ---
void foc_update_current_pi(float kp, float ki, float id_limit, float iq_limit);
void foc_update_velocity_pid(float kp, float ki, float kd, float limit,
                             float ramp);

void foc_update_position_pid(float kp, float ki, float kd, float limit);

void foc_update_iq_pi(float kp, float ki, float limit);

void foc_update_id_pi(float kp, float ki, float limit);

void foc_reset_pid();
#endif