#include "service_message.h"

#include <assert.h>
#include <stm32f4xx_hal.h>

#include "Devices/dev_usart.h"
#include "Utils/log.h"

service_message_t g_message;

static int find_chain(uint32_t main_code, uint32_t sub_code) {
  assert(g_message.chain_index <= MAX_MESSAGE_CHAIN);
  for (int i = 0; i < g_message.chain_index; i++) {
    if (g_message.chains[i].main_code == main_code &&
        g_message.chains[i].sub_code == sub_code) {
      return i;
    }
  }
  return -1;
}

static int dispatch(const protocol_header_t* header, const uint8_t* buf) {
  int chain_idx = find_chain(header->main_code, header->sub_code);
  if (chain_idx != -1) {
    return g_message.chains[chain_idx].callback(header, buf);
  }
  return -1;
}

int service_message_register(uint16_t main_code, uint16_t sub_code,
                             event_processer_cb cbfn) {
  assert(g_message.chain_index <= MAX_MESSAGE_CHAIN);

  if (g_message.chain_index == MAX_MESSAGE_CHAIN - 1) return -1;

  g_message.chains[g_message.chain_index].main_code = main_code;
  g_message.chains[g_message.chain_index].sub_code = sub_code;
  g_message.chains[g_message.chain_index].callback = cbfn;
  ++g_message.chain_index;
  return 0;
}

int service_message_process(const uint8_t* buf, uint16_t size) {
  const protocol_header_t* header = (const protocol_header_t*)buf;
  if (!protocol_check_header(header)) return -1;
  return dispatch(header, buf + sizeof(protocol_header_t));
}
