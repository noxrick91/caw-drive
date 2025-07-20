#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <stdint.h>

#include "Algorithm/foc.h"
#include "Algorithm/motor_params.h"

typedef struct {
  uint16_t dev_type;
  uint16_t dev_id;
  uint16_t ctl_mode;
  uint16_t sensor;
  float target;
  float Tpwm;
  motor_params_t motor_params;
  foc_params_t foc_params;
} config_t;

void config_init(uint16_t dev_type, uint16_t dev_id);
int config_save();
void config_load();
void config_push();
void config_pop();

config_t* config_get();
void config_set_state(uint16_t state);
uint16_t config_get_state();

#endif