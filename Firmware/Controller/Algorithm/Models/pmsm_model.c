#include "pmsm_model.h"

// 初始化PMSM参数
void pmsm_model_params_init(pmsm_params_t *params, float R_s, float L_s,
                            float psi_f, float Ts, float pole_pairs) {
  if (params != NULL) {
    params->R_s = R_s;
    params->L_s = L_s;
    params->psi_f = psi_f;
    params->Ts = Ts;
    params->pole_pairs = pole_pairs;
  }
}
