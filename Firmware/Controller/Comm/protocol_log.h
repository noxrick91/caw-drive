#ifndef __PROTOCOL_LOG_H__
#define __PROTOCOL_LOG_H__

#include <stdint.h>

#include "protocol_header.h"

#define MAX_MESSAGE_SIZE 200

#pragma pack(push, 1)
typedef struct {
  uint8_t log_level;
  char message[MAX_MESSAGE_SIZE];
} _protocol_log_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  protocol_header_t header;
  _protocol_log_t body;
} protocol_log_t;
#pragma pack(pop)

void protocol_pack_log(protocol_log_t* self, uint8_t log_level,
                       const char* func_name, const char* message);

#endif