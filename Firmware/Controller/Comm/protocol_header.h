#ifndef __PROTOCOL_HEADER_H__
#define __PROTOCOL_HEADER_H__

#include <stdbool.h>
#include <stdint.h>

#pragma pack(push, 1)
typedef struct {
  uint8_t magic[6];
  uint32_t timestamp;
  uint16_t device_type;
  uint16_t device_id;
  uint16_t main_code;
  uint16_t sub_code;
  uint16_t version;
  uint16_t length;
  uint8_t checksum;
} protocol_header_t;
#pragma pack(pop)

int protocol_pack_header(protocol_header_t* self, uint32_t main_code,
                         uint32_t sub_code, uint8_t* buf, uint32_t size);
bool protocol_check_header(const protocol_header_t* self);

#endif