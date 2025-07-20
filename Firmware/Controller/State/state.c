#include "./state.h"

#include <stdint.h>

#include "gpio.h"

static state_e g_state = STATE_INIT;
static uint8_t g_led_state = 0x01;

void state_set(state_e state) { g_state = state; }

void led_ctl(uint8_t led_state) {
  if (led_state & 0x01) {
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
  }

  if (led_state & 0x02) {
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
  }

  if (led_state & 0x04) {
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
  }
}

void state_step(void) {
  static uint16_t led4_cnt = 0;
  if (0 == ++led4_cnt % 10) {
    led4_cnt = 0;
    HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
  }

  if (STATE_INIT == g_state) {
    g_led_state = 0x07;
    led_ctl(g_led_state);
  }

  else if (STATE_DEBUG == g_state) {
    static uint8_t dir = 0;
    led_ctl(g_led_state);
    if (0 == dir) {
      g_led_state = g_led_state << 1;
      if (0x04 == g_led_state) {
        dir = 1;
      }
    } else {
      g_led_state = g_led_state >> 1;
      if (0x01 == g_led_state) {
        dir = 0;
      }
    }
  }

  else if (STATE_IDLE == g_state) {
    g_led_state = 0x04;
    led_ctl(g_led_state);
  }

  else if (STATE_RUN == g_state) {
    if (0x01 != g_led_state) {
      g_led_state = 0x01;
    } else {
      g_led_state = 0x00;
    }
    led_ctl(g_led_state);
  }

  else if (STATE_ERROR == g_state) {
    g_led_state = 0x02;
    led_ctl(g_led_state);
  }
}