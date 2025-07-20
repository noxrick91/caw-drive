#ifndef __TASK_STATE_H__
#define __TASK_STATE_H__

#include "../Comm/protocol_header.h"

int task_state_update(const protocol_header_t* header, const uint8_t* buf);

#endif