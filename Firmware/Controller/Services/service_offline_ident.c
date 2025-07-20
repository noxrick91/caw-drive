#include "service_offline_ident.h"

#include <math.h>

#include "../Utils/log.h"
#include "../config.h"
#include "../controller.h"
#include "../transform.h"

// --- 辨识参数定义 ---
#define RAMP_UP_DURATION_MS 2000      // 2秒加速时间
#define MEASUREMENT_DURATION_MS 2000  // 2秒测量时间

static offline_ident_t g_ident;

void service_offline_ident_start(void) {
  config_t* cfg = config_get();

  // 1. 初始化辨识状态结构体
  g_ident.state = IDENT_STATE_RAMP_UP;
  g_ident.error = 0;
  g_ident.start_time_ms = HAL_GetTick();
  g_ident.measurement_count = 0;
  g_ident.vq_sum = 0.0f;
  g_ident.iq_sum = 0.0f;
  g_ident.id_sum = 0.0f;

  // 2. 备份当前控制模式和目标值
  g_ident.target_backup = cfg->target;
  g_ident.ctl_mode_backup = cfg->ctl_mode;

  // 3. 设置辨识所需的目标转速和频率
  g_ident.target_rpm = 100.0f;  // 磁链辨识设定的转速 (RPM)
  g_ident.target_frequency =
      g_ident.target_rpm / 60.0f *
      cfg->motor_params.pole_pairs;  // 转换为电气频率 (Hz)

  // 4. 切换到V/F控制模式并启动电机
  cfg->ctl_mode = CONTROL_MODE_VF;
  cfg->target = g_ident.target_frequency;

  info("Starting flux linkage identification...");
  info("Ramping up for %d ms...", RAMP_UP_DURATION_MS);
}

void service_offline_ident_stop(void) {
  config_t* cfg = config_get();
  // 恢复之前的控制模式和目标值
  cfg->target = g_ident.target_backup;
  cfg->ctl_mode = g_ident.ctl_mode_backup;
  g_ident.state = IDENT_STATE_IDLE;
  info("Offline identification stopped.");
}

void service_offline_ident_periodic(float* phase_currents, float* V_ab,
                                    float theta) {
  if (g_ident.state == IDENT_STATE_IDLE || g_ident.state == IDENT_STATE_DONE) {
    return;
  }

  config_t* cfg = config_get();
  motor_params_t* motor_params = &cfg->motor_params;

  // --- 公共计算：无论在哪个状态，都需要这些值 ---
  // 1. 计算电气角速度 (rad/s)
  float omega_e = g_ident.target_rpm * motor_params->pole_pairs * PI / 30.0f;

  // 3. 坐标变换：三相 -> αβ -> dq
  float i_ab_measured[2];
  clark(phase_currents[0], phase_currents[1], phase_currents[2], i_ab_measured);

  float idq_measured[2];
  park(i_ab_measured[0], i_ab_measured[1], theta, idq_measured);
  float id_measured = idq_measured[0];
  float iq_measured = idq_measured[1];

  float vdq_command[2];
  park(V_ab[0], V_ab[1], theta, vdq_command);
  float vq_command = vdq_command[1];

  // --- 状态机逻辑 ---
  switch (g_ident.state) {
    case IDENT_STATE_RAMP_UP: {
      if (HAL_GetTick() - g_ident.start_time_ms > RAMP_UP_DURATION_MS) {
        info("Ramp up finished. Starting measurement for %d ms...",
             MEASUREMENT_DURATION_MS);
        g_ident.state = IDENT_STATE_MEASURING;
        g_ident.start_time_ms = HAL_GetTick();  // 重置计时器
      }
      break;
    }

    case IDENT_STATE_MEASURING: {
      if (HAL_GetTick() - g_ident.start_time_ms < MEASUREMENT_DURATION_MS) {
        // 累加测量值
        g_ident.vq_sum += vq_command;
        g_ident.iq_sum += iq_measured;
        g_ident.id_sum += id_measured;
        g_ident.measurement_count++;
      } else {
        g_ident.state = IDENT_STATE_CALCULATING;
      }
      break;
    }

    case IDENT_STATE_CALCULATING: {
      if (g_ident.measurement_count > 100) {  // 确保采集了足够的数据点
        // 计算平均值
        float vq_avg = g_ident.vq_sum / g_ident.measurement_count;
        float iq_avg = g_ident.iq_sum / g_ident.measurement_count;
        float id_avg = g_ident.id_sum / g_ident.measurement_count;

        // 使用稳态电压方程计算磁链
        // Ψf = (Vq - Rs*Iq - ωe*Ld*Id) / ωe
        // 为简化，我们先忽略 Ld*Id 项，因为它通常较小
        if (fabsf(omega_e) > 1.0f) {  // 避免除以零
          g_ident.flux_linkage_result =
              (vq_avg - motor_params->phase_resistance * iq_avg -
               omega_e * motor_params->inductance_d * id_avg) /
              omega_e;

          // 将结果更新到配置中
          cfg->motor_params.flux_linkage = g_ident.flux_linkage_result;

          info("Flux Linkage Identification successful.");
          info("  - Vq_avg: %.4f V", vq_avg);
          info("  - Iq_avg: %.4f A", iq_avg);
          info("  - omega_e: %.2f rad/s", omega_e);
          info("  => Flux Linkage: %.6f Wb", g_ident.flux_linkage_result);

        } else {
          g_ident.error = -1;
          error("Identification failed: omega_e is too small.");
        }
      } else {
        g_ident.error = -2;
        error("Identification failed: Not enough data points collected.");
      }
      g_ident.state = IDENT_STATE_DONE;
      service_offline_ident_stop();  // 辨识完成，自动停止
      break;
    }

    default:
      break;
  }
}

void service_offline_ident_wait(void) {
  while (g_ident.state != IDENT_STATE_DONE) {
    HAL_Delay(10);
  }
}