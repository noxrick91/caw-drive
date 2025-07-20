#include "task_state.h"

#include "../Comm/protocol_state.h"
#include "../PWM/pwm.h"
#include "../State/state.h"
#include "../Utils/log.h"
#include "../config.h"

int task_state_update(const protocol_header_t* header, const uint8_t* buf) {
  volatile uint16_t state;
  protocol_unpack_update_state(&state, buf);
  config_set_state(state);

  if (0 == state) {
    pwm_stop();
    state_set(STATE_IDLE);
  } else if (1 == state) {
    pwm_start();
    state_set(STATE_RUN);
  }

  debug("update state: %d", state);

  return 0;
}