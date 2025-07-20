#ifndef __SERVICE_MESSAGE_H__
#define __SERVICE_MESSAGE_H__

#include <stdint.h>

#include "Comm/protocol_header.h"

#define MAX_MESSAGE_CHAIN 20

typedef int (*event_processer_cb)(const protocol_header_t*, const uint8_t*);

typedef struct {
  uint16_t main_code;
  uint16_t sub_code;
  event_processer_cb callback;
} service_message_chain_t;

typedef struct {
  service_message_chain_t chains[MAX_MESSAGE_CHAIN];
  uint16_t chain_index;
  uint8_t rx_buffer[128];
  uint16_t rx_size;
} service_message_t;

int service_message_register(uint16_t main_code, uint16_t sub_code,
                             event_processer_cb cbfn);
int service_message_process(const uint8_t* buf, uint16_t size);

#endif