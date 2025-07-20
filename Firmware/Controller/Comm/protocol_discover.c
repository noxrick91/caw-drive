#include "protocol_discover.h"

#include "../config.h"
#include "./Devices/dev_usart.h"
#include "code.h"

void protocol_write_discover() {
  protocol_header_t header;
  protocol_pack_header(&header, main_code_system, sub_code_system_discover, 0,
                       0);
  // dev_usart_write((uint8_t*)(&header), sizeof(header));
  dev_usart_write_queue((uint8_t*)(&header), sizeof(header));
}