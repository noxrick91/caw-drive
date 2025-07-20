#ifndef __PROTOCOL_OBSERVER_H__
#define __PROTOCOL_OBSERVER_H__

#include "protocol_header.h"

#pragma pack(push, 1)
typedef struct {
  uint64_t tick;
  float data[6];
} _protocol_observer_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  protocol_header_t header;
  _protocol_observer_t body;
} protocol_observer_t;
#pragma pack(pop)

void protocol_pack_observer(protocol_observer_t* self, const float* data,
                            uint64_t tick);

#endif