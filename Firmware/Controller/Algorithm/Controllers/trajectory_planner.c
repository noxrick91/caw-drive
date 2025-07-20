#include "trajectory_planner.h"

#include <math.h>
#include <string.h>  // For memset

#ifndef FLT_EPSILON
#define FLT_EPSILON 1.19209290E-07F
#endif

// 内部辅助函数
static inline float sign(float val) {
  return (val > 0.0f) ? 1.0f : ((val < 0.0f) ? -1.0f : 0.0f);
}

void trajectory_planner_init(trajectory_planner_t* planner,
                             profile_type_t profile_type) {
  memset(planner, 0, sizeof(trajectory_planner_t));
  planner->profile_type = profile_type;
  planner->state = PLANNER_STATE_IDLE;
  // 设置默认限制值，避免未初始化时使用
  planner->max_velocity = 10.0f;      // rad/s
  planner->max_acceleration = 50.0f;  // rad/s^2
  planner->max_jerk = 500.0f;         // rad/s^3
}

void trajectory_planner_set_limits(trajectory_planner_t* planner, float max_vel,
                                   float max_accel, float max_jerk) {
  planner->max_velocity = fabsf(max_vel);
  planner->max_acceleration = fabsf(max_accel);
  planner->max_jerk = fabsf(max_jerk);
}

bool trajectory_planner_plan(trajectory_planner_t* planner, float start_pos,
                             float target_pos) {
  if (planner->state != PLANNER_STATE_IDLE &&
      planner->state != PLANNER_STATE_DONE) {
    return false;  // 正在执行，无法规划新轨迹
  }

  // --- 1. 初始化和计算基本参数 ---
  planner->start_pos = start_pos;
  planner->target_pos = target_pos;
  planner->total_move = target_pos - start_pos;
  planner->direction = sign(planner->total_move);
  planner->move_dist = fabsf(planner->total_move);

  // 如果距离过小，则不移动
  if (planner->move_dist < FLT_EPSILON) {
    planner->state = PLANNER_STATE_DONE;
    planner->current_pos = start_pos;
    planner->current_vel = 0.0f;
    planner->current_accel = 0.0f;
    return true;
  }

  float Vmax = planner->max_velocity;
  float Amax = planner->max_acceleration;
  float Jmax = planner->max_jerk;

  // --- 2. 根据曲线类型计算时间分段 ---
  if (planner->profile_type == PROFILE_TYPE_S_CURVE) {
    // S-Curve planning logic
    // Reference: "Trajectory planning for a custom SCARA robot" by Politecnico
    // di Torino
    float v_a_limit = Amax * Amax / Jmax;

    if (Vmax * Jmax < Amax * Amax) {
      // Velocity limited case
      planner->T_j = sqrtf(Vmax / Jmax);
      planner->T_a = 2 * planner->T_j;
      planner->v_a = Jmax * planner->T_j * planner->T_j;
    } else {
      // Acceleration limited case
      planner->T_j = Amax / Jmax;
      planner->T_a = Vmax / Amax;
      planner->v_a = Vmax;
    }

    float dist_accel_phase = planner->v_a * planner->T_a;

    if (planner->move_dist >= dist_accel_phase) {
      // Trapezoidal S-Curve (reaches Vmax)
      planner->T_d = planner->T_a;
      planner->T_v = (planner->move_dist - dist_accel_phase) / Vmax;
    } else {
      // Triangular S-Curve (does not reach Vmax)
      planner->T_j = powf(planner->move_dist / (2 * Jmax), 1.0f / 3.0f);
      planner->T_a = 2 * planner->T_j;
      planner->v_a = Jmax * planner->T_j * planner->T_j;
      planner->T_d = planner->T_a;
      planner->T_v = 0.0f;
    }
  } else {
    // T型曲线规划 (梯形速度)
    planner->T_j = 0.0f;  // T型曲线无Jerk段

    // 能达到最大速度所需的距离
    float dist_to_vmax = Vmax * Vmax / (2 * Amax) * 2;

    if (planner->move_dist > dist_to_vmax) {
      // 梯形速度曲线
      planner->v_a = Vmax;
      planner->T_a = Vmax / Amax;
      planner->T_d = planner->T_a;
      planner->T_v = (planner->move_dist - dist_to_vmax / 2 * 2) / Vmax;
    } else {
      // 三角形速度曲线
      planner->T_a = sqrtf(planner->move_dist / Amax);
      planner->v_a = planner->T_a * Amax;
      planner->T_d = planner->T_a;
      planner->T_v = 0.0f;
    }
  }

  planner->T_total = planner->T_a + planner->T_v + planner->T_d;

  // --- 3. 重置状态并准备执行 ---
  planner->current_time = 0.0f;
  planner->current_pos = start_pos;
  planner->current_vel = 0.0f;
  planner->current_accel = 0.0f;
  planner->state = PLANNER_STATE_TRAJECTORY;

  return true;
}

planner_state_t trajectory_planner_step(trajectory_planner_t* planner,
                                        float dt) {
  if (planner->state != PLANNER_STATE_TRAJECTORY) {
    return planner->state;
  }

  planner->current_time += dt;

  if (planner->current_time >= planner->T_total) {
    // 轨迹结束
    planner->state = PLANNER_STATE_DONE;
    planner->current_pos = planner->target_pos;
    planner->current_vel = 0.0f;
    planner->current_accel = 0.0f;
    return planner->state;
  }

  float t = planner->current_time;
  float J = planner->max_jerk;
  float A = planner->max_acceleration;
  float Tj = planner->T_j;
  float Ta = planner->T_a;
  float Tv = planner->T_v;
  float Td = planner->T_d;
  float Va = planner->v_a;
  float move_dist = planner->move_dist;

  float p, v, a;

  if (planner->profile_type == PROFILE_TYPE_S_CURVE) {
    // S-Curve Profile, based on
    // https://www.mathworks.com/help/motioncontrol/ref/scurvetrajectory.html
    float T_total = Ta + Tv + Td;
    float t_accel_end = Ta;
    float t_cruise_end = Ta + Tv;
    float t_decel_end = T_total;

    // Pre-calculate positions at phase ends
    float p_j1_end = (1.0f / 6.0f) * J * powf(Tj, 3);
    float v_j1_end = 0.5f * J * Tj * Tj;
    float p_const_a_end = v_j1_end * (Ta - Tj) + p_j1_end;
    float v_const_a_end = A * (Ta - Tj) + v_j1_end;
    float p_j2_end = (1.0f / 6.0f) * J * powf(Tj, 3) + 0.5f * A * Tj * Tj +
                     v_const_a_end * Tj;

    // Acceleration phase
    if (t <= t_accel_end) {
      if (t < Tj) {  // Increasing acceleration
        a = J * t;
        v = 0.5f * J * t * t;
        p = (1.0f / 6.0f) * J * t * t * t;
      } else if (t <= Ta - Tj) {  // Constant acceleration
        a = A;
        v = J * Tj * Tj * 0.5f + A * (t - Tj);
        p = J / 6.0f * powf(Tj, 3) + J * Tj * Tj / 2.0f * (t - Tj) +
            A / 2.0f * powf(t - Tj, 2);
      } else {  // Decreasing acceleration
        float t_rem = Ta - t;
        a = J * t_rem;
        v = Va - 0.5f * J * t_rem * t_rem;
        p = move_dist / 2.0f -
            (Va * (Ta - t_rem - t) + (1.0f / 6.0f) * J * powf(t_rem, 3));
      }
    } else if (t <= t_cruise_end) {  // Constant velocity phase
      a = 0;
      v = Va;
      p = Va * Ta - planner->max_velocity * Tj -
          planner->max_velocity * planner->max_velocity / (2 * A) +
          Va * (t - Ta);
    } else {  // Deceleration phase
      float td = t - t_cruise_end;
      if (td < Tj) {  // Increasing deceleration
        a = -J * td;
        v = Va - 0.5f * J * td * td;
        p = Va * Ta - planner->max_velocity * Tj -
            planner->max_velocity * planner->max_velocity / (2 * A) + Va * Tv +
            Va * td - (1.0f / 6.0f) * J * powf(td, 3);
      } else if (td <= Td - Tj) {  // Constant deceleration
        a = -A;
        v = Va - (A * (td - Tj) + J * Tj * Tj / 2.0f);
        p = move_dist - (A * powf(Td - td, 2) / 2.0f + J * Tj * (Td - td) +
                         J * Tj * Tj / 3.0f);
      } else {  // Decreasing deceleration
        float t_rem = Td - td;
        a = -J * t_rem;
        v = 0.5f * J * t_rem * t_rem;
        p = move_dist - (1.0f / 6.0f) * J * powf(t_rem, 3);
      }
    }
  } else {
    // Trapezoidal Profile
    if (t < Ta) {  // Acceleration phase
      a = A;
      v = A * t;
      p = 0.5f * A * t * t;
    } else if (t < Ta + Tv) {  // Constant velocity phase
      a = 0;
      v = Va;
      p = 0.5f * A * Ta * Ta + Va * (t - Ta);
    } else {  // Deceleration phase
      float td = t - (Ta + Tv);
      a = -A;
      v = Va - A * td;
      p = 0.5f * A * Ta * Ta + Va * Tv + Va * td - 0.5f * A * td * td;
    }
  }

  planner->current_accel = planner->direction * a;
  planner->current_vel = planner->direction * v;
  planner->current_pos = planner->start_pos + planner->direction * p;

  return planner->state;
}

void trajectory_planner_stop(trajectory_planner_t* planner) {
  planner->state = PLANNER_STATE_IDLE;
  planner->current_time = 0.0f;
  // Don't reset current pos/vel/accel, let them be as they were
}
