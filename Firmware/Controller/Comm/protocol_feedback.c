#include "protocol_feedback.h"

#include "code.h"

void protocol_pack_feedback(protocol_feedback_t* self, float Vbus,
                            float position, float velocity, float temp1,
                            float temp2) {
  self->body.Vbus = Vbus;
  self->body.position = position;
  self->body.velocity = velocity;
  self->body.temp1 = temp1;
  self->body.temp2 = temp2;

  protocol_pack_header(&self->header, main_code_motor, sub_code_motor_feedback,
                       (uint8_t*)(&self->body), sizeof(self->body));
}