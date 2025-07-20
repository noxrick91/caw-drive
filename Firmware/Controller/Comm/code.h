#ifndef __CODE_H__
#define __CODE_H__

#include <stdint.h>

typedef enum _main_code_e {
  main_code_other = (uint16_t)0,
  main_code_system = (uint16_t)1,
  main_code_bms = (uint16_t)2,
  main_code_motor = (uint16_t)3,
} main_code_e;

typedef enum _sub_code_other_e {
  unknown = (uint16_t)0,
} sub_code_other_e;

typedef enum _sub_code_system_e {
  sub_code_system_discover = (uint16_t)0,
  sub_code_system_log = (uint16_t)1,
  sub_code_system_discover_reply = (uint16_t)1000,
} sub_code_system_e;

typedef enum _sub_code_bms_e {
  sub_code_bms_info = (uint16_t)0,
} sub_code_bms_e;

typedef enum _sub_code_motor_e {
  sub_code_motor_observation = (uint16_t)0,
  sub_code_motor_load_config = (uint16_t)1,
  sub_code_motor_update_config = (uint16_t)2,
  sub_code_motor_init_config = (uint16_t)3,
  sub_code_motor_flash_config = (uint16_t)4,
  sub_code_motor_ident = (uint16_t)5,
  sub_code_motor_feedback = (uint16_t)6,
  sub_code_motor_update_state = (uint16_t)7,
  sub_code_motor_load_config_reply = (uint16_t)1001,
} sub_code_motor_e;

#endif