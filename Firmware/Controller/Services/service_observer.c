#include "service_observer.h"

#include <stdint.h>

#include "../Comm/protocol_observer.h"
#include "../Devices/dev_usart.h"
#include "../Utils/log.h"

void service_observer_periodic(const float* arr) {
  static uint8_t call_counter = 0;
  static uint64_t tick = 0;

  call_counter++;
  if (call_counter >= 5) {
    call_counter = 0;

    protocol_observer_t obs;
    protocol_pack_observer(&obs, arr, tick);
    dev_usart_write_queue((uint8_t*)&obs, sizeof(obs));
  }
  tick++;
}