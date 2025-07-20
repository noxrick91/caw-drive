#include "protocol_observer.h"

#include <string.h>

#include "code.h"

void protocol_pack_observer(protocol_observer_t* self, const float* data,
                            uint64_t tick) {
  memcpy(self->body.data, data, sizeof(self->body.data));
  self->body.tick = tick;
  protocol_pack_header(&self->header, main_code_motor,
                       sub_code_motor_observation, (uint8_t*)(&self->body),
                       sizeof(self->body));
}