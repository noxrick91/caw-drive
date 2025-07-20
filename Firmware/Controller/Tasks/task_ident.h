#ifndef __TASK_IDENT_H__
#define __TASK_IDENT_H__

#include "../Comm/protocol_header.h"

int task_ident(const protocol_header_t* header, const uint8_t* buf);

#endif