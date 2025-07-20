#include "service_feedback.h"

#include <stdint.h>

#include "../Comm/protocol_feedback.h"
#include "../Devices/dev_usart.h"
#include "../Utils/log.h"

void service_feedback_periodic(float Vbus, float velocity, float position,
                               float temp1, float temp2) {
  static uint16_t tick = 0;
  tick++;
  if (tick >= 1000) {
    tick = 0;
    protocol_feedback_t fb;
    protocol_pack_feedback(&fb, Vbus, position, velocity, temp1, temp2);
    dev_usart_write_queue((uint8_t*)&fb, sizeof(fb));

    // uint32_t overflow_count = dev_usart_get_tx_overflow_count();
    // if (overflow_count > 0) {
    //   error("USART TX overflow: %d", overflow_count);
    // }
  }
}