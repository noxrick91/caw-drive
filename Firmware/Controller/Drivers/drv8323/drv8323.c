#include "drv8323.h"

#include "gpio.h"

// 低电平进入睡眠模式，高电平使能，8~40uS低脉冲重置故障
#define _ENABLE(x) HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, x)
// 放大器较准输入，逻辑高时执行自动偏移较准，校准后需要拉低
#define _CAL(x) HAL_GPIO_WritePin(DRV_CAL_GPIO_Port, DRV_CAL_Pin, x)

// SPI通讯NSS
#define _NSS(x) HAL_GPIO_WritePin(DRV_SPI_NSS_GPIO_Port, DRV_SPI_NSS_Pin, x)

uint16_t _SPI_WRITE(drv8323_t* self, uint16_t data) {
  uint16_t ret;
  _NSS(0);
  HAL_Delay(10);
  HAL_StatusTypeDef sta = HAL_SPI_TransmitReceive(
      self->hspi, (uint8_t*)&data, (uint8_t*)&ret, 1, DRV8323_SPI_TIMEOUT);
  if (HAL_OK != sta) {
    // warn("_SPI_WRITE FAILED: %d", sta);
  }
  _NSS(1);
  return ret;
}

int drv8323_init(drv8323_t* self, SPI_HandleTypeDef* spi) {
  self->hspi = spi;
  _ENABLE(0);
  _CAL(1);
  HAL_Delay(10);
  _ENABLE(1);
  _CAL(0);
  HAL_Delay(10);
}

uint16_t drv8323_read_fsr1(drv8323_t* self) {
  uint16_t val = (1 << 15) | (FSR1 << 11);
  return _SPI_WRITE(self, val);
}

uint16_t drv8323_read_fsr2(drv8323_t* self) {
  uint16_t val = (1 << 15) | (FSR2 << 11);
  return _SPI_WRITE(self, val);
}

uint16_t drv8323_read_register(drv8323_t* self, uint16_t reg) {
  uint16_t ret = _SPI_WRITE(self, (1 << 15) | (reg << 11));
  return ret;
}

void drv8323_write_register(drv8323_t* self, uint16_t reg, uint16_t val) {
  _SPI_WRITE(self, (reg << 11) | val);
}

void drv8323_write_dcr(drv8323_t* self, uint16_t dis_cpuv, uint16_t dis_gdf,
                       uint16_t otw_rep, uint16_t pwm_mode, uint16_t pwm_com,
                       uint16_t pwm_dir, uint16_t coast, uint16_t brake,
                       uint16_t clr_flt) {
  uint16_t val = (DCR << 11) | (dis_cpuv << 9) | (dis_gdf << 8) |
                 (otw_rep << 7) | (pwm_mode << 5) | (pwm_com << 4) |
                 (pwm_dir << 3) | (coast << 2) | (brake << 1) | clr_flt;
  _SPI_WRITE(self, val);
}

void drv8323_write_hsr(drv8323_t* self, uint16_t lock, uint16_t idrivep_hs,
                       uint16_t idriven_hs) {
  uint16_t val = (HSR << 11) | (lock << 8) | (idrivep_hs << 4) | idriven_hs;
  _SPI_WRITE(self, val);
}

void drv8323_write_lsr(drv8323_t* self, uint16_t cbc, uint16_t tdrive,
                       uint16_t idrivep_ls, uint16_t idriven_ls) {
  uint16_t val = (LSR << 11) | (cbc << 10) | (tdrive << 8) | (idrivep_ls << 4) |
                 idriven_ls;
  _SPI_WRITE(self, val);
}

void drv8323_write_ocpcr(drv8323_t* self, uint16_t tretry, uint16_t dead_time,
                         uint16_t ocp_mode, uint16_t ocp_deg,
                         uint16_t vds_lvl) {
  uint16_t val = (OCPCR << 11) | (tretry << 10) | (dead_time << 8) |
                 (ocp_mode << 6) | (ocp_deg << 4) | vds_lvl;
  _SPI_WRITE(self, val);
}

void drv8323_write_csacr(drv8323_t* self, uint16_t csa_fet, uint16_t vref_div,
                         uint16_t ls_ref, uint16_t csa_gain, uint16_t dis_sen,
                         uint16_t csa_cal_a, uint16_t csa_cal_b,
                         uint16_t csa_cal_c, uint16_t sen_lvl) {
  uint16_t val = (CSACR << 11) | (csa_fet << 10) | (vref_div << 9) |
                 (ls_ref << 8) | (csa_gain << 6) | (dis_sen << 5) |
                 (csa_cal_a << 4) | (csa_cal_b << 3) | (csa_cal_c << 2) |
                 sen_lvl;
  _SPI_WRITE(self, val);
}

void drv8323_enable_gd(drv8323_t* self) {
  uint16_t val = (drv8323_read_register(self, DCR)) & (~(0x1 << 2));
  drv8323_write_register(self, DCR, val);
}

void drv8323_disable_gd(drv8323_t* self) {
  uint16_t val = (drv8323_read_register(self, DCR)) | (0x1 << 2);
  drv8323_write_register(self, DCR, val);
}

void drv8323_calibrate(drv8323_t* self) {
  uint16_t val = 0x1 << 4 + 0x1 << 3 + 0x1 << 2;
  drv8323_write_register(self, CSACR, val);
}

void drv8323_print_faults(drv8323_t* self) {
  // uint16_t val1 = drv8323_read_fsr1(self);
  // _printf("fsr1:%x", val1);
  // HAL_Delay(10);
  // uint16_t val2 = drv8323_read_fsr2(self);
  // _printf("fsr2:%x", val2);
  // HAL_Delay(10);

  // if (val1 & (1 << 10)) {
  //   _printf("FAULT");
  // }

  // if (val1 & (1 << 9)) {
  //   _printf("VDS_OCP");
  // }
  // if (val1 & (1 << 8)) {
  //   _printf("GDF");
  // }
  // if (val1 & (1 << 7)) {
  //   _printf("UVLO");
  // }
  // if (val1 & (1 << 6)) {
  //   _printf("OTSD");
  // }
  // if (val1 & (1 << 5)) {
  //   _printf("VDS_HA");
  // }
  // if (val1 & (1 << 4)) {
  //   _printf("VDS_LA");
  // }
  // if (val1 & (1 << 3)) {
  //   _printf("VDS_HB");
  // }
  // if (val1 & (1 << 2)) {
  //   _printf("VDS_LB");
  // }
  // if (val1 & (1 << 1)) {
  //   _printf("VDS_HC");
  // }
  // if (val1 & (1)) {
  //   _printf("VDS_LC");
  // }

  // if (val2 & (1 << 10)) {
  //   _printf("SA_OC");
  // }
  // if (val2 & (1 << 9)) {
  //   _printf("SB_OC");
  // }
  // if (val2 & (1 << 8)) {
  //   _printf("SC_OC");
  // }
  // if (val2 & (1 << 7)) {
  //   _printf("OTW");
  // }
  // if (val2 & (1 << 6)) {
  //   _printf("CPUV");
  // }
  // if (val2 & (1 << 5)) {
  //   _printf("VGS_HA");
  // }
  // if (val2 & (1 << 4)) {
  //   _printf("VGS_LA");
  // }
  // if (val2 & (1 << 3)) {
  //   _printf("VGS_HB");
  // }
  // if (val2 & (1 << 2)) {
  //   _printf("VGS_LB");
  // }
  // if (val2 & (1 << 1)) {
  //   _printf("VGS_HC");
  // }
  // if (val2 & (1)) {
  //   _printf("VGS_LC");
  // }
}

void drv8323_reg_val(drv8323_t* self) {
  // _printf(
  //     "FSR1:%04x FSR2:%04x DCR:%04x HSR:%04x LSR:%04x "
  //     "OCPCR:%04x CSACR:%04x",
  //     drv8323_read_register(self, FSR1), drv8323_read_register(self, FSR2),
  //     drv8323_read_register(self, DCR), drv8323_read_register(self, HSR),
  //     drv8323_read_register(self, LSR), drv8323_read_register(self, OCPCR),
  //     drv8323_read_register(self, CSACR));
}

void drv8323_spi_reset() {
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) {
    Error_Handler();
  }
  HAL_Delay(100);
}