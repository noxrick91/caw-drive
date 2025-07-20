#ifndef DRV8323_H
#define DRV8323_H

#include "spi.h"

#define DRV8323_SPI_TIMEOUT 1000

/// Registers ///
#define FSR1 0x0   /// Fault Status Register 1
#define FSR2 0x1   /// Fault Status Register 2
#define DCR 0x2    /// Drive Control Register
#define HSR 0x3    /// Gate Drive HS Register
#define LSR 0x4    /// Gate Drive LS Register
#define OCPCR 0x5  /// OCP Control Register
#define CSACR 0x6  /// CSA Control Register

/// Drive Control Fields ///
#define DIS_CPUV_EN 0x0  /// Charge pump UVLO fault
#define DIS_CPUV_DIS 0x1
#define DIS_GDF_EN 0x0  /// Gate drive fauilt
#define DIS_GDF_DIS 0x1
#define OTW_REP_EN 0x1  /// Over temp warning reported on nFAULT/FAULT bit
#define OTW_REP_DIS 0x0
#define PWM_MODE_6X 0x0  /// PWM Input Modes
#define PWM_MODE_3X 0x1
#define PWM_MODE_1X 0x2
#define PWM_MODE_IND 0x3
#define PWM_1X_COM_SYNC 0x0  /// 1x PWM Mode synchronou rectification
#define PWM_1X_COM_ASYNC 0x1
#define PWM_1X_DIR_0 \
  0x0  /// In 1x PWM mode this bit is ORed with the INHC (DIR) input
#define PWM_1X_DIR_1 0x1

/// Gate Drive HS Fields ///
#define LOCK_ON 0x6
#define LOCK_OFF 0x3
#define IDRIVEP_HS_10MA 0x0  /// Gate drive high side turn on current
#define IDRIVEP_HS_30MA 0x1
#define IDRIVEP_HS_60MA 0x2
#define IDRIVEP_HS_80MA 0x3
#define IDRIVEP_HS_120MA 0x4
#define IDRIVEP_HS_140MA 0x5
#define IDRIVEP_HS_170MA 0x6
#define IDRIVEP_HS_190MA 0x7
#define IDRIVEP_HS_260MA 0x8
#define IDRIVEP_HS_330MA 0x9
#define IDRIVEP_HS_370MA 0xA
#define IDRIVEP_HS_440MA 0xB
#define IDRIVEP_HS_570MA 0xC
#define IDRIVEP_HS_680MA 0xD
#define IDRIVEP_HS_820MA 0xE
#define IDRIVEP_HS_1000MA 0xF
#define IDRIVEN_HS_20MA 0x0  /// High side turn off current
#define IDRIVEN_HS_60MA 0x1
#define IDRIVEN_HS_120MA 0x2
#define IDRIVEN_HS_160MA 0x3
#define IDRIVEN_HS_240MA 0x4
#define IDRIVEN_HS_280MA 0x5
#define IDRIVEN_HS_340MA 0x6
#define IDRIVEN_HS_380MA 0x7
#define IDRIVEN_HS_520MA 0x8
#define IDRIVEN_HS_660MA 0x9
#define IDRIVEN_HS_740MA 0xA
#define IDRIVEN_HS_880MA 0xB
#define IDRIVEN_HS_1140MA 0xC
#define IDRIVEN_HS_1360MA 0xD
#define IDRIVEN_HS_1640MA 0xE
#define IDRIVEN_HS_2000MA 0xF

/// Gate Drive LS Fields ///
#define TDRIVE_500NS 0x0  /// Peak gate-current drive time
#define TDRIVE_1000NS 0x1
#define TDRIVE_2000NS 0x2
#define TDRIVE_4000NS 0x3
#define IDRIVEP_LS_10MA 0x0  /// Gate drive high side turn on current
#define IDRIVEP_LS_30MA 0x1
#define IDRIVEP_LS_60MA 0x2
#define IDRIVEP_LS_80MA 0x3
#define IDRIVEP_LS_120MA 0x4
#define IDRIVEP_LS_140MA 0x5
#define IDRIVEP_LS_170MA 0x6
#define IDRIVEP_LS_190MA 0x7
#define IDRIVEP_LS_260MA 0x8
#define IDRIVEP_LS_330MA 0x9
#define IDRIVEP_LS_370MA 0xA
#define IDRIVEP_LS_440MA 0xB
#define IDRIVEP_LS_570MA 0xC
#define IDRIVEP_LS_680MA 0xD
#define IDRIVEP_LS_820MA 0xE
#define IDRIVEP_LS_1000MA 0xF
#define IDRIVEN_LS_20MA 0x0  /// High side turn off current
#define IDRIVEN_LS_60MA 0x1
#define IDRIVEN_LS_120MA 0x2
#define IDRIVEN_LS_160MA 0x3
#define IDRIVEN_LS_240MA 0x4
#define IDRIVEN_LS_280MA 0x5
#define IDRIVEN_LS_340MA 0x6
#define IDRIVEN_LS_380MA 0x7
#define IDRIVEN_LS_520MA 0x8
#define IDRIVEN_LS_660MA 0x9
#define IDRIVEN_LS_740MA 0xA
#define IDRIVEN_LS_880MA 0xB
#define IDRIVEN_LS_1140MA 0xC
#define IDRIVEN_LS_1360MA 0xD
#define IDRIVEN_LS_1640MA 0xE
#define IDRIVEN_LS_2000MA 0xF

/// OCP Control Fields ///
#define TRETRY_4MS 0x0  /// VDS OCP and SEN OCP retry time
#define TRETRY_50US 0x1
#define DEADTIME_50NS 0x0  /// Deadtime
#define DEADTIME_100NS 0x1
#define DEADTIME_200NS 0x2
#define DEADTIME_400NS 0x3
#define OCP_LATCH 0x0  /// OCP Mode
#define OCP_RETRY 0x1
#define OCP_REPORT 0x2
#define OCP_NONE 0x3
#define OCP_DEG_2US 0x0  /// OCP Deglitch Time
#define OCP_DEG_4US 0x1
#define OCP_DEG_6US 0x2
#define OCP_DEG_8US 0x3
#define VDS_LVL_0_06 0x0
#define VDS_LVL_0_13 0x1
#define VDS_LVL_0_2 0x2
#define VDS_LVL_0_26 0x3
#define VDS_LVL_0_31 0x4
#define VDS_LVL_0_45 0x5
#define VDS_LVL_0_53 0x6
#define VDS_LVL_0_6 0x7
#define VDS_LVL_0_68 0x8
#define VDS_LVL_0_75 0x9
#define VDS_LVL_0_94 0xA
#define VDS_LVL_1_13 0xB
#define VDS_LVL_1_3 0xC
#define VDS_LVL_1_5 0xD
#define VDS_LVL_1_7 0xE
#define VDS_LVL_1_88 0xF

/// CSA Control Fields ///
#define CSA_FET_SP 0x0  /// Current sense amplifier positive input
#define CSA_FET_SH 0x1
#define VREF_DIV_1 0x0  /// Amplifier reference voltage is VREV/1
#define VREF_DIV_2 0x1  /// Amplifier reference voltage is VREV/2
#define CSA_GAIN_5 0x0  /// Current sensor gain
#define CSA_GAIN_10 0x1
#define CSA_GAIN_20 0x2
#define CSA_GAIN_40 0x3
#define DIS_SEN_EN 0x0  /// Overcurrent Fault
#define DIS_SEN_DIS 0x1
#define SEN_LVL_0_25 0x0  /// Sense OCP voltage level
#define SEN_LVL_0_5 0x1
#define SEN_LVL_0_75 0x2
#define SEN_LVL_1_0 0x3

typedef struct _drv8323_t {
  SPI_HandleTypeDef* hspi;
} drv8323_t;

int drv8323_init(drv8323_t* self, SPI_HandleTypeDef* spi);
uint16_t drv8323_read_fsr1(drv8323_t* self);
uint16_t drv8323_read_fsr2(drv8323_t* self);
uint16_t drv8323_read_register(drv8323_t* self, uint16_t reg);
void drv8323_write_register(drv8323_t* self, uint16_t reg, uint16_t val);
void drv8323_write_dcr(drv8323_t* self, uint16_t dis_cpuv, uint16_t dis_gdf,
                       uint16_t otw_rep, uint16_t pwm_mode, uint16_t pwm_com,
                       uint16_t pwm_dir, uint16_t coast, uint16_t brake,
                       uint16_t clr_flt);
void drv8323_write_hsr(drv8323_t* self, uint16_t lock, uint16_t idrivep_hs,
                       uint16_t idriven_hs);
void drv8323_write_lsr(drv8323_t* self, uint16_t cbc, uint16_t tdrive,
                       uint16_t idrivep_ls, uint16_t idriven_ls);
void drv8323_write_ocpcr(drv8323_t* self, uint16_t tretry, uint16_t dead_time,
                         uint16_t ocp_mode, uint16_t ocp_deg, uint16_t vds_lvl);
void drv8323_write_csacr(drv8323_t* self, uint16_t csa_fet, uint16_t vref_div,
                         uint16_t ls_ref, uint16_t csa_gain, uint16_t dis_sen,
                         uint16_t csa_cal_a, uint16_t csa_cal_b,
                         uint16_t csa_cal_c, uint16_t sen_lvl);
void drv8323_enable_gd(drv8323_t* self);
void drv8323_disable_gd(drv8323_t* self);
void drv8323_calibrate(drv8323_t* self);
void drv8323_print_faults(drv8323_t* self);
void drv8323_reg_val(drv8323_t* self);

void drv8323_spi_reset();

#endif