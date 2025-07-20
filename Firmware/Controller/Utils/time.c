#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"

#if defined(STM32F446xx)
#define DWT_DELAY_ENABLED
#endif

#ifdef DWT_DELAY_ENABLED

static void _dwt_delay_init(void) {
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }

  if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}

void _delay_us(volatile uint32_t microseconds) {
  if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
    _dwt_delay_init();
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) return;
  }
  uint32_t initial_ticks = DWT->CYCCNT;
  uint32_t ticks_to_wait = microseconds * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - initial_ticks) < ticks_to_wait);
}

#endif