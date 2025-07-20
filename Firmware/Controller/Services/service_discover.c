#include "service_discover.h"

#include <stdint.h>
#include <stm32f4xx_hal.h>

#include "../Comm/protocol_discover.h"
#include "../Utils/log.h"

void service_discover_periodic(void) {
  static uint16_t tick = 0;
  tick++;
  if (tick >= 3000) {
    tick = 0;
    protocol_write_discover();
  }
}