#ifndef __STATE_H__
#define __STATE_H__

typedef enum _state_e {
  STATE_INIT = 0x0,
  STATE_IDLE,
  STATE_RUN,
  STATE_DEBUG,
  STATE_ERROR,
} state_e;

void state_set(state_e state);

void state_step(void);

#endif