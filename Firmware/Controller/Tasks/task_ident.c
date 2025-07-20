#include "task_ident.h"

#include <stm32f4xx_hal.h>

#include "../Algorithm/Controllers/params.h"
#include "../Algorithm/if.h"
#include "../Services/service_offline_ident.h"
#include "../Utils/log.h"
#include "../config.h"

int task_ident(const protocol_header_t* header, const uint8_t* buf) {
  config_t* cfg = config_get();

  config_set_state(0);
  // 电阻辨识
  if (0 != motor_params_ident_resistance(&cfg->motor_params,
                                         cfg->motor_params.test_current)) {
    error("Motor parameters ident resistance failed");
    return -1;
  }

  // // 电感辨识
  if (0 != motor_params_ident_inductance(&cfg->motor_params,
                                         cfg->motor_params.test_current)) {
    error("Motor parameters ident inductance failed");
    return -1;
  }
  config_set_state(1);
  // 开始离线辨识,磁链辨识
  service_offline_ident_start();

  // =======================================================
  float Kp, Ki;
  // 计算电流环PI控制器参数
  calc_current_pi(cfg->motor_params.phase_resistance,
                  cfg->motor_params.inductance_d, cfg->Tpwm, &Kp, &Ki,
                  CALC_CURRENT_PI_METHOD_BANDWIDTH, 100.0f);

  pid_set_param(&cfg->foc_params.id_pid, Kp, Ki, 0.0f,
                cfg->foc_params.id_pid.output_limit,
                cfg->foc_params.id_pid.output_ramp);

  pid_set_param(&cfg->foc_params.iq_pid, Kp, Ki, 0.0f,
                cfg->foc_params.iq_pid.output_limit,
                cfg->foc_params.iq_pid.output_ramp);

  if_set_pi_params(Kp, Ki);

  return 0;
}