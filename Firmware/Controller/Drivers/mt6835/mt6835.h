#ifndef __MT6835GT_H__
#define __MT6835GT_H__

#include <stdbool.h>
#include <stdint.h>

#include "spi.h"

#define MT6835_ZERO_REG_STEP (0.088f)
#define MT6835_ANGLE_RESOLUTION (1 << 21)

typedef enum mt6835_cmd_enum_t {
  MT6835_CMD_RD = (0b0011),     /**< user read register. */
  MT6835_CMD_WR = (0b0110),     /**< user write register. */
  MT6835_CMD_EEPROM = (0b1100), /**< user erase and program EEPROM. */
  MT6835_CMD_ZERO = (0b0101),   /**< AUTO setting zero. */
  MT6835_CMD_BURST = (0b1010),  /**< burst mode. */
} mt6835_cmd_enum_t;

typedef enum mt6835_reg_enum_t {
  MT6835_REG_ID = (0x001),        // ID
  MT6835_REG_ANGLE3 = (0x003),    // angle 3
  MT6835_REG_ANGLE2 = (0x004),    // angle 2
  MT6835_REG_ANGLE1 = (0x005),    // angle 1 and state
  MT6835_REG_CRC = (0x006),       // CRC
  MT6835_REG_ABZ_RES2 = (0x007),  // ABZ res 2
  MT6835_REG_ABZ_RES1 = (0x008),  // ABZ res 1
  MT6835_REG_ZERO2 = (0x009),     // zero 2
  MT6835_REG_ZERO1 = (0x00A),     // zero 1
  MT6835_REG_UVW = (0x00B),       // UVW
  MT6835_REG_PWM = (0x00C),       // PWM
  MT6835_REG_HYST = (0x00D),      // HYST
  MT6835_REG_AUTOCAL = (0x00E),   // AUTO cal
} mt6835_reg_enum_t;

typedef enum mt6835_state_enum_t {
  MT6835_STATE_OVER_SPEED = 0x01,       // over speed
  MT6835_STATE_MAG_FIELD_WEAK = 0x02,   // mag field weak
  MT6835_STATE_VCC_UNDERVOLTAGE = 0x04  // vcc under voltage
} mt6835_state_enum_t;

typedef struct {
  union {
    uint32_t pack;
    struct {
      uint8_t reserved : 4;
      mt6835_cmd_enum_t cmd : 4;
      mt6835_reg_enum_t reg : 8;
      uint8_t normal_byte : 8;
      uint8_t empty_byte : 8;
    };
  };
} mt6835_data_frame_t;

typedef struct {
  SPI_HandleTypeDef* hspi;
  uint8_t crc;
  bool crc_ret;
  bool crc_check;
  mt6835_state_enum_t state;
  uint32_t last_valid_raw_angle;  // 用于存储上一次CRC校验成功的原始角度值

  // Multi-turn tracking state
  int64_t revolutions;
  float prev_angle;
  float current_angle;
} mt6835_t;

int mt6835_init(SPI_HandleTypeDef* spi);
uint32_t mt6835_get_raw_angle();
float mt6835_update();
int64_t mt6835_get_revolutions();
float mt6835_get_single_turn_angle();
float mt6835_get_multi_turn_angle();
bool mt6835_set_zero_angle(float rad);
void mt6835_spi_reset();
bool mt6835_is_data_valid();
#endif