#include "protocol_config.h"

#include <string.h>

#include "../Devices/dev_usart.h"
#include "code.h"

void protocol_pack_load_config_reply(protocol_config_t* self) {
  config_t* cfg = config_get();

  memset(self, 0, sizeof(self));
  self->body.dev_type = cfg->dev_type;
  self->body.dev_id = cfg->dev_id;
  self->body.ctl_mode = cfg->ctl_mode;
  self->body.sensor = cfg->sensor;
  self->body.target = cfg->target;
  self->body.Tpwm = cfg->Tpwm;
  // motor_params_t 字段赋值
  self->body.motor_params.phase_resistance = cfg->motor_params.phase_resistance;
  self->body.motor_params.inductance_d = cfg->motor_params.inductance_d;
  self->body.motor_params.inductance_q = cfg->motor_params.inductance_q;
  self->body.motor_params.flux_linkage = cfg->motor_params.flux_linkage;
  self->body.motor_params.pole_pairs = cfg->motor_params.pole_pairs;
  self->body.motor_params.nominal_mechanical_rpm =
      cfg->motor_params.nominal_mechanical_rpm;
  self->body.motor_params.B = cfg->motor_params.B;
  self->body.motor_params.J = cfg->motor_params.J;
  // foc_params_t 字段赋值
  self->body.foc_params.id_pid.Kp = cfg->foc_params.id_pid.Kp;
  self->body.foc_params.id_pid.Ki = cfg->foc_params.id_pid.Ki;
  self->body.foc_params.id_pid.Kd = cfg->foc_params.id_pid.Kd;
  self->body.foc_params.id_pid.output_limit =
      cfg->foc_params.id_pid.output_limit;

  self->body.foc_params.iq_pid.Kp = cfg->foc_params.iq_pid.Kp;
  self->body.foc_params.iq_pid.Ki = cfg->foc_params.iq_pid.Ki;
  self->body.foc_params.iq_pid.Kd = cfg->foc_params.iq_pid.Kd;
  self->body.foc_params.iq_pid.output_limit =
      cfg->foc_params.iq_pid.output_limit;

  self->body.foc_params.velocity_pid.Kp = cfg->foc_params.velocity_pid.Kp;
  self->body.foc_params.velocity_pid.Ki = cfg->foc_params.velocity_pid.Ki;
  self->body.foc_params.velocity_pid.Kd = cfg->foc_params.velocity_pid.Kd;
  self->body.foc_params.velocity_pid.output_limit =
      cfg->foc_params.velocity_pid.output_limit;
  self->body.foc_params.velocity_pid.output_ramp =
      cfg->foc_params.velocity_pid.output_ramp;
  self->body.foc_params.velocity_lpf_tau = cfg->foc_params.velocity_lpf_tau;

  self->body.foc_params.position_pid.Kp = cfg->foc_params.position_pid.Kp;
  self->body.foc_params.position_pid.Ki = cfg->foc_params.position_pid.Ki;
  self->body.foc_params.position_pid.Kd = cfg->foc_params.position_pid.Kd;
  self->body.foc_params.position_pid.output_limit =
      cfg->foc_params.position_pid.output_limit;

  protocol_pack_header(&(self->header), main_code_motor,
                       sub_code_motor_load_config_reply,
                       (uint8_t*)(&self->body), sizeof(self->body));
}

void protocol_unpack_update_config(config_t* cfg, const uint8_t* buf) {
  if (cfg == NULL || buf == NULL) {
    return;
  }

  const _protocol_config_t* src_protocol_cfg = (const _protocol_config_t*)buf;

  // Copy basic fields from protocol structure to config_t
  cfg->dev_type = src_protocol_cfg->dev_type;
  cfg->dev_id = src_protocol_cfg->dev_id;
  cfg->ctl_mode = src_protocol_cfg->ctl_mode;
  cfg->sensor = src_protocol_cfg->sensor;
  cfg->target = src_protocol_cfg->target;
  cfg->Tpwm = src_protocol_cfg->Tpwm;

  // Copy motor_params from protocol_motor_params_t to motor_params_t
  cfg->motor_params.phase_resistance =
      src_protocol_cfg->motor_params.phase_resistance;
  cfg->motor_params.inductance_d = src_protocol_cfg->motor_params.inductance_d;
  cfg->motor_params.inductance_q = src_protocol_cfg->motor_params.inductance_q;
  cfg->motor_params.flux_linkage = src_protocol_cfg->motor_params.flux_linkage;
  cfg->motor_params.pole_pairs = src_protocol_cfg->motor_params.pole_pairs;
  cfg->motor_params.nominal_mechanical_rpm =
      src_protocol_cfg->motor_params.nominal_mechanical_rpm;
  cfg->motor_params.B = src_protocol_cfg->motor_params.B;
  cfg->motor_params.J = src_protocol_cfg->motor_params.J;

  // Copy foc_params from protocol_foc_params_t to foc_params_t
  // Assuming foc_params_t has id_pid, iq_pid, velocity_pid, position_pid
  // of a type compatible with protocol_pid_t (e.g., pid_t)

  // ID PID parameters
  cfg->foc_params.id_pid.Kp = src_protocol_cfg->foc_params.id_pid.Kp;
  cfg->foc_params.id_pid.Ki = src_protocol_cfg->foc_params.id_pid.Ki;
  cfg->foc_params.id_pid.Kd = src_protocol_cfg->foc_params.id_pid.Kd;
  cfg->foc_params.id_pid.output_limit =
      src_protocol_cfg->foc_params.id_pid.output_limit;

  // IQ PID parameters
  cfg->foc_params.iq_pid.Kp = src_protocol_cfg->foc_params.iq_pid.Kp;
  cfg->foc_params.iq_pid.Ki = src_protocol_cfg->foc_params.iq_pid.Ki;
  cfg->foc_params.iq_pid.Kd = src_protocol_cfg->foc_params.iq_pid.Kd;
  cfg->foc_params.iq_pid.output_limit =
      src_protocol_cfg->foc_params.iq_pid.output_limit;

  // Velocity PID parameters
  cfg->foc_params.velocity_pid.Kp =
      src_protocol_cfg->foc_params.velocity_pid.Kp;
  cfg->foc_params.velocity_pid.Ki =
      src_protocol_cfg->foc_params.velocity_pid.Ki;
  cfg->foc_params.velocity_pid.Kd =
      src_protocol_cfg->foc_params.velocity_pid.Kd;
  cfg->foc_params.velocity_pid.output_limit =
      src_protocol_cfg->foc_params.velocity_pid.output_limit;
  cfg->foc_params.velocity_pid.output_ramp =
      src_protocol_cfg->foc_params.velocity_pid.output_ramp;
  cfg->foc_params.velocity_lpf_tau =
      src_protocol_cfg->foc_params.velocity_lpf_tau;

  // Position PID parameters
  cfg->foc_params.position_pid.Kp =
      src_protocol_cfg->foc_params.position_pid.Kp;
  cfg->foc_params.position_pid.Ki =
      src_protocol_cfg->foc_params.position_pid.Ki;
  cfg->foc_params.position_pid.Kd =
      src_protocol_cfg->foc_params.position_pid.Kd;
  cfg->foc_params.position_pid.output_limit =
      src_protocol_cfg->foc_params.position_pid.output_limit;
}