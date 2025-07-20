#ifndef __SERVICE_OFFLINE_IDENT_H__
#define __SERVICE_OFFLINE_IDENT_H__

#include <stdbool.h>
#include <stdint.h>

#include "../math_const.h"

// 辨识过程的状态机
typedef enum {
  IDENT_STATE_IDLE,         // 空闲
  IDENT_STATE_RAMP_UP,      // 电机加速稳定阶段
  IDENT_STATE_MEASURING,    // 测量数据阶段
  IDENT_STATE_CALCULATING,  // 计算结果阶段
  IDENT_STATE_DONE,         // 完成
  IDENT_STATE_ERROR         // 错误
} offline_ident_state_t;

typedef struct {
  offline_ident_state_t state;
  uint32_t start_time_ms;  // 用于计时的起始时间

  // --- 备份和目标值 ---
  float target_rpm;
  float target_frequency;
  float target_backup;
  int ctl_mode_backup;

  // --- 数据累加器 ---
  float vq_sum;
  float iq_sum;
  float id_sum;
  int32_t measurement_count;

  // --- 最终结果 ---
  float flux_linkage_result;
  int32_t error;
} offline_ident_t;

void service_offline_ident_start(void);
void service_offline_ident_periodic(float* phase_currents, float* V_ab,
                                    float theta);
void service_offline_ident_stop(void);
void service_offline_ident_wait(void);

#endif