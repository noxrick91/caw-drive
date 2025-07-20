#ifndef __MOTOR_PARAMS_H__
#define __MOTOR_PARAMS_H__

#include <stdint.h>

#include "pmsm_model.h"

// 电机参数结构体
typedef struct {
  float phase_resistance;        // 相电阻 (Ohms)
  float inductance_d;            // d轴电感 (Henrys)
  float inductance_q;            // q轴电感 (Henrys)
  float flux_linkage;            // 转子磁链 (Webers)
  int32_t pole_pairs;            // 电机极对数
  float nominal_current;         // 额定电流
  float test_current;            // 测试电流
  float nominal_mechanical_rpm;  // 额定机械转速
  float B;                       // 阻尼系数
  float J;                       // 转动惯量
} motor_params_t;

// 初始化函数
void motor_params_init(motor_params_t* self, int32_t pp, float test_current,
                       float nominal_rpm);
int motor_params_ident_resistance(motor_params_t* self, float test_current);
int motor_params_ident_inductance(motor_params_t* self, float test_current);

#endif