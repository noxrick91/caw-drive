#ifndef __PID_H__
#define __PID_H__

typedef struct _pid_t {
  // --- Parameters ---
  float Kp, Ki, Kd;
  float output_limit;  // 输出绝对值限幅
  float output_ramp;   // 输出变化率限幅 (单位/秒)

  // --- State Variables ---
  float integral;
  float prev_error;
  float prev_output;
} pid_t;

/**
 * @brief 初始化PID控制器
 * @param self 指向PID结构体的指针
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 * @param output_limit 输出绝对值限幅
 * @param output_ramp 输出变化率限幅 (单位/秒), 设为0则禁用此功能
 */
void pid_init(pid_t* self, float Kp, float Ki, float Kd, float output_limit,
              float output_ramp);

void pid_set_param(pid_t* self, float Kp, float Ki, float Kd, float limit,
                   float ramp);
void pid_reset(pid_t* self);

float pid_step(pid_t* self, float error, float dt);

#endif