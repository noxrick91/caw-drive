#ifndef __TASK_PARAMS_H__
#define __TASK_PARAMS_H__

#include "../Comm/protocol_header.h"

int task_config_init(const protocol_header_t* header, const uint8_t* buf);
int task_config_load(const protocol_header_t* header, const uint8_t* buf);
int task_config_update(const protocol_header_t* header, const uint8_t* buf);
int task_config_flash(const protocol_header_t* header, const uint8_t* buf);

#endif