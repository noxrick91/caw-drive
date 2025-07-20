#include "mt6835.h"

#include <math.h>
#include <string.h>

#include "../../Utils/log.h"
#include "../../math_const.h"

mt6835_t g_mt6835;

#define _NSS(x) \
  HAL_GPIO_WritePin(ENCODER_SPI_NSS_GPIO_Port, ENCODER_SPI_NSS_Pin, x)

static uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3,
};

static uint8_t crc_table(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;  // 初始CRC值

  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];         // 与数据异或
    crc = crc8_table[crc];  // 查表更新CRC
  }

  return crc;
}

static uint8_t mt6835_read_reg(mt6835_reg_enum_t reg) {
  uint8_t result[3] = {0, 0, 0};
  mt6835_data_frame_t frame;
  frame.cmd = MT6835_CMD_RD;
  frame.reg = reg;
  _NSS(0);
  HAL_SPI_TransmitReceive(g_mt6835.hspi, (uint8_t *)&frame.pack,
                          (uint8_t *)result, 3, 100);
  _NSS(1);
  return result[2];
}

static void mt6835_write_reg(mt6835_reg_enum_t reg, uint8_t data) {
  uint8_t result[3] = {0, 0, 0};

  mt6835_data_frame_t frame;
  frame.cmd = MT6835_CMD_WR;
  frame.reg = reg;
  frame.normal_byte = data;

  _NSS(0);
  HAL_SPI_TransmitReceive(g_mt6835.hspi, (uint8_t *)&frame.pack,
                          (uint8_t *)&result, 3, 100);
  _NSS(1);
}

int mt6835_init(SPI_HandleTypeDef *spi) {
  memset(&g_mt6835, 0, sizeof(mt6835_t));
  g_mt6835.crc_check = false;
  g_mt6835.hspi = spi;

  // Reset multi-turn tracking state within the instance
  g_mt6835.revolutions = 0;
  g_mt6835.prev_angle = 0.0f;  // Reset to invalid
  g_mt6835.current_angle = 0.0f;

  do {
    mt6835_update();
    HAL_Delay(1);
  } while (!mt6835_is_data_valid());

  return 0;
}

// These are now part of the g_mt6835 instance
// static int64_t g_revolutions = 0;
// static float g_prev_angle = -1.0f;
// static float g_current_angle = 0.0f;

uint32_t mt6835_get_raw_angle() {
  uint8_t rx_buf[6] = {0};
  mt6835_data_frame_t frame;
  frame.cmd = MT6835_CMD_BURST;
  frame.reg = MT6835_REG_ANGLE3;
  _NSS(0);
  HAL_SPI_TransmitReceive(g_mt6835.hspi, (uint8_t *)&frame.pack,
                          (uint8_t *)rx_buf, 6, 100);
  _NSS(1);
  memmove(rx_buf, &rx_buf[2], 3);
  rx_buf[3] = rx_buf[5];

  g_mt6835.crc = rx_buf[3];
  if (crc_table(rx_buf, 3) != rx_buf[3]) {
    warn("crc check failed");
    g_mt6835.crc_ret = false;
    return 0;
  }
  g_mt6835.crc_ret = true;
  g_mt6835.state = rx_buf[2] & 0x07;
  return (rx_buf[0] << 13) | (rx_buf[1] << 5) | (rx_buf[2] >> 3);
}

float mt6835_update() {
  uint32_t raw_angle = mt6835_get_raw_angle();
  if (!mt6835_is_data_valid()) {
    return g_mt6835.current_angle;
  }

  g_mt6835.current_angle = (float)(raw_angle * 2.996056226329803e-6);

  if (g_mt6835.prev_angle < 0) {
    g_mt6835.prev_angle = g_mt6835.current_angle;
  }

  float delta = g_mt6835.current_angle - g_mt6835.prev_angle;
  if (fabsf(delta) > (0.8f * 2.0f * PI)) {
    if (delta > 0.0f) {
      g_mt6835.revolutions--;
    } else {
      g_mt6835.revolutions++;
    }
  }
  g_mt6835.prev_angle = g_mt6835.current_angle;
  return g_mt6835.current_angle;
}

bool mt6835_set_zero_angle(float rad) {
  uint16_t angle = (uint16_t)roundf(rad * 57.295779513f / MT6835_ZERO_REG_STEP);
  if (angle > 0xFFF) {
    return false;
  }

  uint8_t tx_buf[2] = {0};

  tx_buf[1] = angle >> 4;
  tx_buf[0] = (angle & 0x0F) << 4;
  tx_buf[0] |= mt6835_read_reg(MT6835_REG_ZERO1) & 0x0F;

  mt6835_write_reg(MT6835_REG_ZERO2, tx_buf[1]);
  mt6835_write_reg(MT6835_REG_ZERO1, tx_buf[0]);

  return true;
}

int64_t mt6835_get_revolutions() { return g_mt6835.revolutions; }

float mt6835_get_single_turn_angle() { return g_mt6835.current_angle; }

float mt6835_get_multi_turn_angle() {
  return g_mt6835.current_angle + g_mt6835.revolutions * 2.0f * PI;
}

void mt6835_spi_reset() {
  HAL_SPI_Abort(&hspi3);
  HAL_Delay(10);
  HAL_SPI_DeInit(&hspi3);
  HAL_Delay(10);
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) {
    error("SPI3 init failed");
    Error_Handler();
  }
  HAL_Delay(10);
}

bool mt6835_is_data_valid() { return g_mt6835.crc_ret; }