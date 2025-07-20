#ifndef __PROTOCOL_FEEDBACK_H__
#define __PROTOCOL_FEEDBACK_H__

#include "protocol_header.h"

#pragma pack(push, 1)
typedef struct {
  float Vbus;
  float position;
  float velocity;
  float temp1;
  float temp2;
} _protocol_feedback_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  protocol_header_t header;
  _protocol_feedback_t body;
} protocol_feedback_t;
#pragma pack(pop)

void protocol_pack_feedback(protocol_feedback_t* self, float Vbus,
                            float position, float velocity, float temp1,
                            float temp2);

#endif