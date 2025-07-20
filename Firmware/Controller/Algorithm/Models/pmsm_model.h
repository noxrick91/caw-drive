#ifndef PMSM_MODEL_H
#define PMSM_MODEL_H

#include <math.h>  // 用于 sinf, cosf, fmodf

#include "../math_const.h"  // 假设 PI 定义在此 (例如，来自您的 Controller 目录)

// 为 pmsm_model_get_measurement_estimation 函数定义测量维度
// 这个值应该与 ekf_estimator.h 中的 EKF_MEAS_DIM 保持一致
#define EKF_MEAS_DIM 2

// PMSM 参数结构体
typedef struct {
  float R_s;         // 定子电阻 (欧姆)
  float L_s;         // 定子电感 (亨利) (对于 SPMSM, Ld = Lq = Ls)
  float psi_f;       // 转子永磁体磁链 (韦伯)
  float Ts;          // 采样时间 (秒)
  float pole_pairs;  // 新增：电机极对数
} pmsm_params_t;

/**
 * @brief 初始化PMSM参数。
 * @param params 指向PMSM参数结构体的指针。
 * @param R_s 定子电阻 (欧姆)。
 * @param L_s 定子电感 (亨利)。
 * @param psi_f 转子磁链 (韦伯)。
 * @param Ts 采样时间 (秒)。
 * @param pole_pairs 电机极对数。
 */
void pmsm_model_params_init(pmsm_params_t *params, float R_s, float L_s,
                            float psi_f, float Ts, float pole_pairs);

#endif  // PMSM_MODEL_H
