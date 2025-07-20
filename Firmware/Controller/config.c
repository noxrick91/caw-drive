#include "config.h"

#include <stm32f4xx_hal.h>
#include <string.h>

#include "Algorithm/Controllers/params.h"
#include "Utils/log.h"

#define FLASH_PARAM_BASE_ADDR (0x08060000)

static config_t g_config;
static config_t g_config_tmp;
static volatile uint16_t g_state;

void config_init(uint16_t dev_type, uint16_t dev_id) {
  g_state = 0;
  g_config.dev_type = dev_type;
  g_config.dev_id = dev_id;
  g_config.ctl_mode = 0;
  g_config.target = 0.0f;
  g_config.Tpwm = 1e-4f;
}

int config_save() {
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                         FLASH_FLAG_PGSERR);
  FLASH_EraseInitTypeDef flash;
  uint32_t err;

  flash.TypeErase = FLASH_TYPEERASE_SECTORS;
  flash.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  flash.Sector = FLASH_SECTOR_7;
  flash.NbSectors = 1;

  if (HAL_OK != HAL_FLASHEx_Erase(&flash, &err)) {
    error("HAL_FLASHEx_Erase failed: %d", err);
    HAL_FLASH_Lock();
    return -1;
  }

  uint32_t num_double_words =
      (sizeof(config_t) + sizeof(uint64_t) - 1) / sizeof(uint64_t);
  uint64_t buf[num_double_words];

  memset(buf, 0, sizeof(buf));
  memcpy(buf, (void*)&g_config, sizeof(config_t));

  for (uint32_t i = 0; i < num_double_words; i++) {
    if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                    FLASH_PARAM_BASE_ADDR + i * 8, buf[i])) {
      error("HAL_FLASH_Program failed: i=%u", i);
      HAL_FLASH_Lock();
      return -1;
    }
  }

  HAL_FLASH_Lock();
  return 0;
}

void config_load() {
  memcpy(&g_config, (void*)FLASH_PARAM_BASE_ADDR, sizeof(config_t));
}

config_t* config_get() { return &g_config; }

void config_set_state(uint16_t state) { g_state = state; }
uint16_t config_get_state() { return g_state; }

void config_push() { memcpy(&g_config_tmp, &g_config, sizeof(config_t)); }

void config_pop() { memcpy(&g_config, &g_config_tmp, sizeof(config_t)); }