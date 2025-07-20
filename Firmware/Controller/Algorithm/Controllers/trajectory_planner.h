#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

// 规划器类型
typedef enum {
  PROFILE_TYPE_TRAPEZOIDAL,  // T型速度曲线 (梯形加减速)
  PROFILE_TYPE_S_CURVE,      // S型速度曲线
} profile_type_t;

// 规划器状态
typedef enum {
  PLANNER_STATE_IDLE,        // 空闲
  PLANNER_STATE_TRAJECTORY,  // 轨迹执行中
  PLANNER_STATE_DONE,        // 完成
} planner_state_t;

// 轨迹规划器实例
typedef struct {
  // --- 配置参数 ---
  profile_type_t profile_type;  // 速度曲线类型 (默认S型)
  float max_velocity;           // 最大速度 (rad/s)
  float max_acceleration;       // 最大加速度 (rad/s^2)
  float max_jerk;               // 最大加加速度 (rad/s^3) - S曲线专用

  // --- 目标和起始状态 ---
  float start_pos;   // 起始位置 (rad)
  float target_pos;  // 目标位置 (rad)
  float total_move;  // 总移动距离 (rad)
  float move_dist;   // 总移动距离 (绝对值)

  // --- 内部状态变量 ---
  planner_state_t state;  // 当前规划器状态
  float current_pos;      // 当前位置 (rad)
  float current_vel;      // 当前速度 (rad/s)
  float current_accel;    // 当前加速度 (rad/s^2)
  float current_time;     // 从轨迹开始的当前时间 (s)
  int8_t direction;       // 运动方向 (1 或 -1)

  // --- 轨迹分段计算时间点 ---
  // 对于T型曲线(梯形速度):
  // |---- T_a ----|---- T_v ----|---- T_d ----|
  //
  // 对于S型曲线(7段):
  // | T_j1 | T_a - 2*T_j1 | T_j1 | T_v | T_j2 | T_d - 2*T_j2 | T_j2 |
  // T_j1: 加速段的加加速/减加速时间
  // T_j2: 减速段的加加速/减加速时间
  // (在我们的实现中，为了简化，假设加减速对称，T_j1=T_j2=Tj)
  float T_j;      // S曲线中，Jerk段的时间
  float T_a;      // 总加速时间
  float T_v;      // 匀速时间
  float T_d;      // 总减速时间
  float T_total;  // 轨迹总时间

  // 各阶段结束时的位置和速度
  float p_a, p_v, p_d;
  float v_a;
} trajectory_planner_t;

/**
 * @brief 初始化轨迹规划器
 *
 * @param planner 指向规划器实例的指针
 * @param profile_type 速度曲线类型 (S型或T型)
 */
void trajectory_planner_init(trajectory_planner_t* planner,
                             profile_type_t profile_type);

/**
 * @brief 设置运动限制参数
 * @param planner 指向规划器实例的指针
 * @param max_vel 最大速度 (rad/s)
 * @param max_accel 最大加速度 (rad/s^2)
 * @param max_jerk 最大加加速度 (rad/s^3) (仅S曲线)
 */
void trajectory_planner_set_limits(trajectory_planner_t* planner, float max_vel,
                                   float max_accel, float max_jerk);

/**
 * @brief 规划一个新的运动轨迹
 * @param planner 指向规划器实例的指针
 * @param start_pos 起始位置 (rad)
 * @param target_pos 目标位置 (rad)
 * @return true 如果规划成功, false 如果无法到达
 */
bool trajectory_planner_plan(trajectory_planner_t* planner, float start_pos,
                             float target_pos);

/**
 * @brief 执行轨迹规划的一步
 * @param planner 指向规划器实例的指针
 * @param dt 时间步长 (s)
 * @return planner_state_t 当前规划器状态
 */
planner_state_t trajectory_planner_step(trajectory_planner_t* planner,
                                        float dt);

/**
 * @brief 停止当前的规划并重置状态
 * @param planner 指向规划器实例的指针
 */
void trajectory_planner_stop(trajectory_planner_t* planner);

#endif  // TRAJECTORY_PLANNER_H
