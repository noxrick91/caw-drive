#include "protocol_header.h"

#include <string.h>

#include "../../config.h"
#include "stm32f4xx_hal.h"

static uint8_t MAGIC[6] = {'C', 'A', 'W', 'X', 'X', 'X'};
static uint16_t VERSION = 0x101;

static uint8_t _crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}

int protocol_pack_header(protocol_header_t* self, uint32_t main_code,
                         uint32_t sub_code, uint8_t* buf, uint32_t size) {
  memcpy(self->magic, MAGIC, sizeof(MAGIC));
  self->timestamp = HAL_GetTick();
  self->version = VERSION;
  self->device_type = config_get()->dev_type;
  self->device_id = config_get()->dev_id;
  self->main_code = main_code;
  self->sub_code = sub_code;
  self->length = size;
  if (buf != 0) {
    self->checksum = _crc8(buf, size);
  } else {
    self->checksum = 0;
  }
}

bool protocol_check_header(const protocol_header_t* self) {
  if (0 != memcmp(self->magic, MAGIC, sizeof(MAGIC))) {
    return false;
  }
  if (self->version != VERSION) {
    return false;
  }

  return true;
}