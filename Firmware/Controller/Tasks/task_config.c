#include "task_config.h"

#include "../Algorithm/Modulations/svpwm.h"
#include "../Algorithm/Sensors/velocity_estimator.h"
#include "../Algorithm/foc.h"
#include "../Algorithm/if.h"
#include "../Comm/protocol_config.h"
#include "../Devices/dev_usart.h"
#include "../PWM/pwm.h"
#include "../Utils/log.h"
#include "../config.h"
#include "../controller.h"

int task_config_init(const protocol_header_t* header, const uint8_t* buf) {
  debug("init_config: %f", 0.001);
  return 0;
}

int task_config_load(const protocol_header_t* header, const uint8_t* buf) {
  protocol_config_t cfg;
  protocol_pack_load_config_reply(&cfg);
  dev_usart_write_queue((const uint8_t*)&cfg, sizeof(cfg));
  return 0;
}

int task_config_update(const protocol_header_t* header, const uint8_t* buf) {
  // pwm_stop();
  config_t* cfg = config_get();
  protocol_unpack_update_config(cfg, buf);

  // motor_params_init(&cfg->motor_params, cfg->motor_params.pole_pairs,
  //                   cfg->motor_params.nominal_mechanical_rpm);

  // 更新FOC PI/PID参数
  foc_update_id_pi(cfg->foc_params.id_pid.Kp, cfg->foc_params.id_pid.Ki,
                   cfg->foc_params.id_pid.output_limit);

  foc_update_iq_pi(cfg->foc_params.iq_pid.Kp, cfg->foc_params.iq_pid.Ki,
                   cfg->foc_params.iq_pid.output_limit);

  if_set_pi_params(cfg->foc_params.id_pid.Kp, cfg->foc_params.id_pid.Ki);

  foc_update_velocity_pid(cfg->foc_params.velocity_pid.Kp,
                          cfg->foc_params.velocity_pid.Ki,
                          cfg->foc_params.velocity_pid.Kd,
                          cfg->foc_params.velocity_pid.output_limit,
                          cfg->foc_params.velocity_pid.output_ramp);

  debug("velocity_pid: Kp=%f, Ki=%f, Kd=%f, limit=%f, ramp=%f",
        cfg->foc_params.velocity_pid.Kp, cfg->foc_params.velocity_pid.Ki,
        cfg->foc_params.velocity_pid.Kd,
        cfg->foc_params.velocity_pid.output_limit,
        cfg->foc_params.velocity_pid.output_ramp);

  foc_update_position_pid(cfg->foc_params.position_pid.Kp,
                          cfg->foc_params.position_pid.Ki,
                          cfg->foc_params.position_pid.Kd,
                          cfg->foc_params.position_pid.output_limit);

  foc_reset_pid();

  float lpf_alpha = cfg->Tpwm / (cfg->Tpwm + cfg->foc_params.velocity_lpf_tau);
  velocity_estimator_set_alpha(lpf_alpha);

  debug("FOC PI/PID params updated via command");

  svpwm_init(cfg->Tpwm);

  debug("config updated target=%f", cfg->target);
  return 0;
}

int task_config_flash(const protocol_header_t* header, const uint8_t* buf) {
  debug("flash_config: %f", 0.001);
  return 0;
}