#ifndef SMO_PLL_ESTIMATOR_H
#define SMO_PLL_ESTIMATOR_H

#include "../Models/pmsm_model.h"
#include "../common_types.h"

typedef struct {
  float Kslide;
  float Kp_pll;
  float Ki_pll;
  float pll_integrator;
  float estimated_theta_e;
  float estimated_speed_rpm;
  float e_alpha;      // Filtered BEMF for PLL
  float e_beta;       // Filtered BEMF for PLL
  float z_alpha;      // Raw BEMF from SMO
  float z_beta;       // Raw BEMF from SMO
  float i_alpha_hat;  // Estimated alpha-axis current
  float i_beta_hat;   // Estimated beta-axis current
  pmsm_params_t
      pmsm_params_data;  // Store motor parameters by value (as a copy)
  float Ts;
  float bemf_alpha_lpf_prev;
  float bemf_beta_lpf_prev;
  float lpf_bemf_coeff;
} smo_pll_estimator_t;

void smo_pll_estimator_init(pmsm_params_t* pmsm_params, float k_slide,
                            float kp_pll, float ki_pll, float ts,
                            float bemf_lpf_cutoff_freq);
void smo_pll_estimator_step(voltage_vector_t v_ab, current_vector_t i_ab);
float smo_pll_estimator_get_angle();
float smo_pll_estimator_get_speed_rpm();
float smo_pll_estimator_get_speed_rad_s();
smo_pll_estimator_t* smo_pll_estimator_get();
#endif
