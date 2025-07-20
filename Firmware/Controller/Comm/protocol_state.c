#include "protocol_state.h"

void protocol_unpack_update_state(uint16_t* state, const uint8_t* buf) {
  const _protocol_state_t* src_protocol_sta = (const _protocol_state_t*)buf;
  *state = src_protocol_sta->state;
}