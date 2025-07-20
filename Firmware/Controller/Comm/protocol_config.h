#ifndef __PROTOCOL_CONFIG_H__
#define __PROTOCOL_CONFIG_H__

#include "../config.h"
#include "protocol_header.h"

#pragma pack(push, 1)
typedef struct {
  float phase_resistance;        // 相电阻 (Ohms)
  float inductance_d;            // d轴电感 (Henrys)
  float inductance_q;            // q轴电感 (Henrys)
  float flux_linkage;            // 转子磁链 (Webers)
  int32_t pole_pairs;            // 电机极对数
  float nominal_mechanical_rpm;  // 额定机械转速
  float B;                       // 阻尼系数
  float J;                       // 转动惯量
} protocol_motor_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float output_limit;
  float output_ramp;
} protocol_pid_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  protocol_pid_t id_pid;
  protocol_pid_t iq_pid;
  protocol_pid_t velocity_pid;
  protocol_pid_t position_pid;
  float velocity_lpf_tau;
} protocol_foc_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  uint16_t dev_type;
  uint16_t dev_id;

  uint16_t ctl_mode;
  uint16_t sensor;
  float target;
  float Tpwm;
  protocol_motor_params_t motor_params;
  protocol_foc_params_t foc_params;
} _protocol_config_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  protocol_header_t header;
  _protocol_config_t body;
} protocol_config_t;
#pragma pack(pop)

void protocol_pack_load_config_reply(protocol_config_t* self);
void protocol_unpack_update_config(config_t* cfg, const uint8_t* buf);

#endif