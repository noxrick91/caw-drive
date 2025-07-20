#include "./pid.h"

#include <math.h>  // For fabsf

// 内部辅助函数，用于限幅
static inline float _constrain(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

void pid_init(pid_t* self, float Kp, float Ki, float Kd, float output_limit,
              float output_ramp) {
  self->Kp = Kp;
  self->Ki = Ki;
  self->Kd = Kd;
  self->output_limit = fabsf(output_limit);
  self->output_ramp = fabsf(output_ramp);  // 变化率限幅
  self->integral = 0.0f;
  self->prev_error = 0.0f;
  self->prev_output = 0.0f;
}

void pid_set_param(pid_t* self, float Kp, float Ki, float Kd, float limit,
                   float ramp) {
  self->Kp = Kp;
  self->Ki = Ki;
  self->Kd = Kd;
  self->output_limit = fabsf(limit);
  self->output_ramp = fabsf(ramp);

  pid_reset(self);
}

void pid_reset(pid_t* self) {
  self->integral = 0.0f;
  self->prev_error = 0.0f;
  self->prev_output = 0.0f;
}

float pid_step(pid_t* self, float error, float dt) {
  // 安全检查，防止dt无效导致除零错误
  if (dt <= 0.0f) {
    return self->prev_output;
  }

  // --- P项 ---
  float proportional = self->Kp * error;

  // --- I项 (使用梯形积分法，更精确) ---
  // 注意：这里没有使用复杂的抗饱和逻辑，而是直接对积分项进行限幅，
  // 这是一种简单有效的抗饱和方法。
  self->integral += self->Ki * dt * 0.5f * (error + self->prev_error);
  self->integral =
      _constrain(self->integral, -self->output_limit, self->output_limit);

  // --- D项 ---
  float derivative = self->Kd * (error - self->prev_error) / dt;

  // --- 计算理论总输出 ---
  float output = proportional + self->integral + derivative;

  // --- 输出限幅 (绝对值限制) ---
  output = _constrain(output, -self->output_limit, self->output_limit);

  // --- 输出斜坡限制 (变化率限制) ---
  if (self->output_ramp > 0.0f) {
    float max_change = self->output_ramp * dt;
    float output_change = output - self->prev_output;
    output_change = _constrain(output_change, -max_change, max_change);
    output = self->prev_output + output_change;
  }

  // --- 更新状态变量以备下次调用 ---
  self->prev_error = error;
  self->prev_output = output;

  return output;
}