#ifndef __PROTOCOL_STATE_H__
#define __PROTOCOL_STATE_H__

#include <stdint.h>

#include "protocol_header.h"

#pragma pack(push, 1)
typedef struct {
  uint16_t state;
} _protocol_state_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  protocol_header_t header;
  _protocol_state_t body;
} protocol_state_t;
#pragma pack(pop)

void protocol_unpack_update_state(uint16_t* state, const uint8_t* buf);

#endif