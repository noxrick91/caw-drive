#include "setup.h"

void setup_drv8323(drv8323_t* self) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

  drv8323_init(self, &hspi3);
  drv8323_calibrate(self);
  HAL_Delay(10);
  drv8323_write_dcr(self, 0x0, DIS_GDF_DIS, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0,
                    0x0, 0x1);
  HAL_Delay(10);
  drv8323_write_csacr(self, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x1, 0x1, 0x1,
                      SEN_LVL_1_0);
  HAL_Delay(10);
  drv8323_write_csacr(self, 0x0, 0x1, 0x0, CSA_GAIN_40, 0x1, 0x0, 0x0, 0x0,
                      SEN_LVL_1_0);
  HAL_Delay(10);
  drv8323_write_ocpcr(self, TRETRY_50US, DEADTIME_100NS, OCP_NONE, OCP_DEG_8US,
                      VDS_LVL_1_88);
  drv8323_enable_gd(self);

  HAL_Delay(10);
}