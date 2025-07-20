#include "foc.h"

#include <math.h>
#include <stddef.h>

#include "../Utils/log.h"
#include "./Controllers/trajectory_planner.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RPM_TO_RADS (2.0f * M_PI / 60.0f)
#define RADS_TO_RPM (60.0f / (2.0f * M_PI))

#ifndef FLT_EPSILON
#define FLT_EPSILON 1.19209290E-07F
#endif

foc_t g_foc;

// FOC软启动状态
typedef enum {
  FOC_STARTUP_IDLE,
  FOC_STARTUP_RAMP,
  FOC_STARTUP_NORMAL
} foc_startup_state_t;

static foc_startup_state_t startup_state = FOC_STARTUP_IDLE;
static float startup_timer = 0.0f;
static float startup_ramp_duration = 0.5f;  // 500ms软启动时间

/**
 * @brief 初始化FOC控制器
 * @param self FOC控制器实例
 * @param params 指向包含所有PID参数的结构体
 * @param Tpwm PWM周期 (秒)
 */
void foc_init(foc_params_t* params, float Tpwm, float vbus) {
  if (params == NULL) return;

  g_foc.params = params;
  g_foc.Tpwm = Tpwm;

  // 初始化所有目标值为0
  g_foc.id_ref = 0.0f;
  g_foc.iq_ref = 0.0f;
  g_foc.speed_ref = 0.0f;
  g_foc.position_ref = 0.0f;

  float pid_output_limit = vbus / sqrtf(3.0f);
  g_foc.params->id_pid.output_limit = pid_output_limit;
  g_foc.params->iq_pid.output_limit = pid_output_limit;

  // 初始化所有PID控制器
  pid_init(&g_foc.params->id_pid, g_foc.params->id_pid.Kp,
           g_foc.params->id_pid.Ki, g_foc.params->id_pid.Kd,
           g_foc.params->id_pid.output_limit, 0.0f);
  pid_init(&g_foc.params->iq_pid, g_foc.params->iq_pid.Kp,
           g_foc.params->iq_pid.Ki, g_foc.params->iq_pid.Kd,
           g_foc.params->iq_pid.output_limit, 0.0f);
  pid_init(&g_foc.params->velocity_pid, g_foc.params->velocity_pid.Kp,
           g_foc.params->velocity_pid.Ki, g_foc.params->velocity_pid.Kd,
           g_foc.params->velocity_pid.output_limit, 4000.0f);
  pid_init(&g_foc.params->position_pid, g_foc.params->position_pid.Kp,
           g_foc.params->position_pid.Ki, g_foc.params->position_pid.Kd,
           g_foc.params->position_pid.output_limit, 0.0f);

  // 初始化速度斜坡/规划器参数
  g_foc.speed_ref_actual = 0.0f;
  g_foc.speed_ramp_accel = 1500.0f;  // 默认加速度 2000 rpm/s

  // 初始化轨迹规划器
  trajectory_planner_init(&g_foc.trajectory,
                          PROFILE_TYPE_S_CURVE);  // 默认S型曲线
  // 设置一个默认的限制，可以在别处修改
  trajectory_planner_set_limits(&g_foc.trajectory, 1000.0f * RPM_TO_RADS,
                                4000.0f * RPM_TO_RADS, 40000.0f * RPM_TO_RADS);
}

/**
 * @brief 执行FOC控制的核心步骤 (级联控制)
 * @param self FOC控制器实例
 * @param id_meas 测量的d轴电流
 * @param iq_meas 测量的q轴电流
 * @param speed_meas 测量的速度 (rpm)
 * @param multi_turn_angle 测量的多圈绝对角度 (rad)
 * @param v_d 指向输出d轴电压的指针
 * @param v_q 指向输出q轴电压的指针
 */
void foc_step(float id_meas, float iq_meas, float speed_meas,
              float multi_turn_angle, float* v_d, float* v_q) {
  // 检查是否为直接电压控制模式（用于参数辨识）
  if (g_foc.control_mode == FOC_CONTROL_MODE_VOLTAGE) {
    *v_d = g_foc.id_ref;  // 在此模式下, id_ref 被用作 v_d 电压
    *v_q = g_foc.iq_ref;  // 在此模式下, iq_ref 被用作 v_q 电压
    return;               // 直接返回，跳过PID控制器
  }

  float id_ref_internal = g_foc.id_ref;  // 内部d轴电流目标
  float iq_ref_internal = g_foc.iq_ref;  // 内部q轴电流目标

  // // 确保在力矩控制模式下d轴电流目标为0
  // if (g_foc.control_mode == FOC_CONTROL_MODE_TORQUE) {
  //   id_ref_internal = 0.0f;  // 力矩控制模式下d轴电流应为0
  // }

  id_ref_internal = 0.0f;
  float final_speed_ref_for_pid = 0.0f;  // PID速度环的最终输入
  static float last_pos_ref = 0.0f;

  // --- 根据不同控制模式，计算速度环的目标值 ---
  if (g_foc.control_mode == FOC_CONTROL_MODE_POSITION) {
    // 位置控制模式，使用轨迹规划器
    // 1. 检测目标位置是否变化，如果变化则重新规划
    if (fabsf(g_foc.position_ref - last_pos_ref) > FLT_EPSILON) {
      trajectory_planner_plan(&g_foc.trajectory, g_foc.trajectory.current_pos,
                              g_foc.position_ref);
      last_pos_ref = g_foc.position_ref;
    }

    // 2. 执行规划器步进
    trajectory_planner_step(&g_foc.trajectory, g_foc.Tpwm);

    // 3. 位置环 (P控制器)
    //    位置环的输出作为速度环的修正量
    // float position_error = g_foc.trajectory.current_pos - multi_turn_angle;
    // float pos_correction_vel =
    //     pid_step(&g_foc.params->position_pid, position_error, g_foc.Tpwm);
    float position_error = g_foc.trajectory.current_pos - multi_turn_angle;
    float pos_correction_vel = g_foc.params->position_pid.Kp * position_error;

    // 4. 将规划速度和位置环修正速度相加
    final_speed_ref_for_pid =
        (g_foc.trajectory.current_vel * RADS_TO_RPM) + pos_correction_vel;

  } else if (g_foc.control_mode == FOC_CONTROL_MODE_SPEED) {
    // 速度控制模式，使用速度斜坡
    float max_speed_change = g_foc.speed_ramp_accel * g_foc.Tpwm;
    float speed_error = g_foc.speed_ref - g_foc.speed_ref_actual;

    if (fabsf(speed_error) > max_speed_change) {
      g_foc.speed_ref_actual +=
          (speed_error > 0 ? 1.0f : -1.0f) * max_speed_change;
    } else {
      g_foc.speed_ref_actual = g_foc.speed_ref;
    }
    final_speed_ref_for_pid = g_foc.speed_ref_actual;
  }

  // 根据不同控制模式，执行级联控制
  // 力矩模式直接使用 iq_ref, 速度和位置模式则通过PID计算
  if (g_foc.control_mode == FOC_CONTROL_MODE_POSITION ||
      g_foc.control_mode == FOC_CONTROL_MODE_SPEED) {
    // 2. 速度环 (中间环)
    float speed_error = final_speed_ref_for_pid - speed_meas;
    // 速度环的输出是q轴电流环的目标值
    iq_ref_internal =
        pid_step(&g_foc.params->velocity_pid, speed_error, g_foc.Tpwm);
  }

  // 3. 电流环 (最内环)
  // d轴电流环
  float id_error = id_ref_internal - id_meas;
  *v_d = pid_step(&g_foc.params->id_pid, id_error, g_foc.Tpwm);

  // q轴电流环
  float iq_error = iq_ref_internal - iq_meas;
  *v_q = pid_step(&g_foc.params->iq_pid, iq_error, g_foc.Tpwm);
  static uint16_t tick = 0;
}

/**
 * @brief 在运行时更新FOC电流环的PI参数
 * @param self FOC控制器实例
 * @param kp 新的 Kp (用于d和q轴)
 * @param ki 新的 Ki (用于d和q轴)
 */
void foc_update_current_pi(float kp, float ki, float id_limit, float iq_limit) {
  if (g_foc.params == NULL) return;

  // 1. 更新配置结构体中的值，以便保存
  g_foc.params->id_pid.Kp = kp;
  g_foc.params->id_pid.Ki = ki;
  g_foc.params->id_pid.output_limit = id_limit;
  g_foc.params->iq_pid.Kp = kp;
  g_foc.params->iq_pid.Ki = ki;
  g_foc.params->iq_pid.output_limit = iq_limit;

  // 2. 调用pid库函数设置参数（虽然pid_step直接用成员，但这是良好实践）
  pid_set_param(&g_foc.params->id_pid, kp, ki, g_foc.params->id_pid.Kd,
                id_limit, 0.0f);
  pid_set_param(&g_foc.params->iq_pid, kp, ki, g_foc.params->iq_pid.Kd,
                iq_limit, 0.0f);

  // 3. 参数更新后，重置PID状态以避免积分突变
  pid_reset(&g_foc.params->id_pid);
  pid_reset(&g_foc.params->iq_pid);
  debug("FOC Current PI updated: Kp=%.2f, Ki=%.2f", kp, ki);
}

void foc_update_velocity_pid(float kp, float ki, float kd, float limit,
                             float ramp) {
  pid_set_param(&g_foc.params->velocity_pid, kp, ki, kd, limit, ramp);
  pid_reset(&g_foc.params->velocity_pid);
}

void foc_update_position_pid(float kp, float ki, float kd, float limit) {
  pid_set_param(&g_foc.params->position_pid, kp, ki, kd, limit, 0.0f);
  pid_reset(&g_foc.params->position_pid);
}

void foc_update_iq_pi(float kp, float ki, float limit) {
  pid_set_param(&g_foc.params->iq_pid, kp, ki, 0.0f, limit, 0.0f);
}

void foc_update_id_pi(float kp, float ki, float limit) {
  pid_set_param(&g_foc.params->id_pid, kp, ki, 0.0f, limit, 0.0f);
}

void foc_reset_pid() {
  pid_reset(&g_foc.params->id_pid);
  pid_reset(&g_foc.params->iq_pid);
  pid_reset(&g_foc.params->velocity_pid);
  pid_reset(&g_foc.params->position_pid);
}

/**
 * @brief FOC软启动管理
 * @param iq_target 目标q轴电流
 * @param dt 时间步长
 * @return 经过软启动处理的q轴电流目标
 */
float foc_soft_start(float iq_target, float dt) {
  switch (startup_state) {
    case FOC_STARTUP_IDLE:
      if (fabsf(iq_target) > 0.1f) {  // 检测到非零目标，开始启动
        startup_state = FOC_STARTUP_RAMP;
        startup_timer = 0.0f;
      }
      return 0.0f;

    case FOC_STARTUP_RAMP:
      startup_timer += dt;
      if (startup_timer >= startup_ramp_duration) {
        startup_state = FOC_STARTUP_NORMAL;
        return iq_target;
      } else {
        // 线性斜坡启动
        float ramp_factor = startup_timer / startup_ramp_duration;
        return iq_target * ramp_factor;
      }

    case FOC_STARTUP_NORMAL:
      if (fabsf(iq_target) < 0.05f) {  // 目标接近零，重置状态
        startup_state = FOC_STARTUP_IDLE;
        startup_timer = 0.0f;
      }
      return iq_target;

    default:
      startup_state = FOC_STARTUP_IDLE;
      return 0.0f;
  }
}