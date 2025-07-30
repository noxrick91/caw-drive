#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "Algorithm/Models/pmsm_model.h"  // PMSM电机模型头文件
#include "Algorithm/Modulations/svpwm.h"  // SVPWM调制头文件
#include "Algorithm/foc.h"
#include "Algorithm/if.h"
#include "Algorithm/vf.h"
#include "Drivers/drv8323/drv8323.h"
#include "Drivers/mt6835/mt6835.h"

typedef enum _CONTROL_MODE {
  CONTROL_MODE_VF,
  CONTROL_MODE_IF,
  CONTROL_MODE_FOC_TORQUE,
  CONTROL_MODE_FOC_VELOCITY,
  CONTROL_MODE_FOC_POSITION,
} CONTROL_MODE;

typedef enum _SENSOR_TYPE {
  SENSOR_TYPE_SMO,
  SENSOR_TYPE_HALL,
  SENSOR_TYPE_AS5047P,
  SENSOR_TYPE_MT6835GT,
} SENSOR_TYPE;

typedef struct {
  drv8323_t driver;
  vf_controller_t vf_controller;
  if_controller_t if_controller;

  // Real-time data from sensors and calculations
  volatile float electrical_angle;  // Corrected electrical angle (rad)
  volatile float velocity_rpm;      // Estimated speed (rpm)
  volatile float id_measured;       // Measured d-axis current (A)
  volatile float iq_measured;       // Measured q-axis current (A)

  volatile float vd_command;  // Commanded d-axis voltage (V)
  volatile float vq_command;  // Commanded q-axis voltage (V)

  volatile current_vector_t iab_measured;  // Measured alpha-beta current (A)
  volatile voltage_vector_t vab_measured;  // Measured alpha-beta voltage (V)

  volatile float phase_currents[3];  // Phase currents (A)
  volatile float v_bus;              // Bus voltage (V)
  volatile float temp[2];            // Temperatures (C)
  float zero_electric_angle;  // Sensor zero electrical angle offset (rad)
  float multi_turn_angle;     // 多圈角度
  uint32_t last_tick;         // For dt calculation in core step
} controller_t;

extern controller_t g_controller;  // 全局控制器实例声明

void controller_init();
void controller_step();
void controller_core_step();

#endif