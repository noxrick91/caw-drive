#include "smo_pll_estimator.h"

#include <math.h>
#include <stdint.h>

#include "../../Utils/log.h"
#include "math_const.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

#ifndef TWO_PI
#define TWO_PI (2.0f * M_PI)
#endif

static smo_pll_estimator_t g_smo_pll_estimator;

static float saturate(float val, float min_val, float max_val) {
  if (val > max_val) return max_val;
  if (val < min_val) return min_val;
  return val;
}

void smo_pll_estimator_init(pmsm_params_t* pmsm_params_input, float k_slide,
                            float kp_pll, float ki_pll, float ts,
                            float bemf_lpf_cutoff_freq) {
  debug("SMO_INIT_ENTRY: k_slide_in=%.2f, kp_pll_in=%.2f, ki_pll_in=%.2f",
        k_slide, kp_pll, ki_pll);  // Log input parameters
  if (pmsm_params_input == NULL) {
    // Handle error: input parameters are null. Log or assert.
    // For now, we can't proceed if motor params are not provided.
    // Consider adding a log message here.
    debug("SMO_INIT_ERROR: pmsm_params_input is NULL");
    return;
  }
  g_smo_pll_estimator.pmsm_params_data = *pmsm_params_input;
  g_smo_pll_estimator.Kslide = k_slide;
  g_smo_pll_estimator.Kp_pll = kp_pll;
  g_smo_pll_estimator.Ki_pll = ki_pll;
  g_smo_pll_estimator.Ts = ts;
  g_smo_pll_estimator.pll_integrator = 0.0f;
  g_smo_pll_estimator.estimated_theta_e = 0.0f;
  g_smo_pll_estimator.estimated_speed_rpm = 0.0f;
  g_smo_pll_estimator.e_alpha = 0.0f;
  g_smo_pll_estimator.e_beta = 0.0f;
  g_smo_pll_estimator.z_alpha = 0.0f;
  g_smo_pll_estimator.z_beta = 0.0f;
  g_smo_pll_estimator.i_alpha_hat = 0.0f;
  g_smo_pll_estimator.i_beta_hat = 0.0f;
  g_smo_pll_estimator.bemf_alpha_lpf_prev = 0.0f;
  g_smo_pll_estimator.bemf_beta_lpf_prev = 0.0f;
  if (bemf_lpf_cutoff_freq > 0.0f && ts > 0.0f) {
    float rc = 1.0f / (2.0f * M_PI * bemf_lpf_cutoff_freq);
    g_smo_pll_estimator.lpf_bemf_coeff = ts / (ts + rc);
  } else {
    g_smo_pll_estimator.lpf_bemf_coeff =
        1.0f;  // No filtering if cutoff_freq is invalid
  }
  debug(
      "SMO_INIT_EXIT: Kslide_set=%.2f, Kp_PLL_set=%.2f, Ki_PLL_set=%.2f, "
      "LPF_Coeff=%.4f",
      g_smo_pll_estimator.Kslide, g_smo_pll_estimator.Kp_pll,
      g_smo_pll_estimator.Ki_pll,
      g_smo_pll_estimator.lpf_bemf_coeff);  // Log set parameters
}

void smo_pll_estimator_step(voltage_vector_t v_ab, current_vector_t i_ab) {
  // Use the local copy of motor parameters
  float R_s = g_smo_pll_estimator.pmsm_params_data.R_s;
  float L_s = g_smo_pll_estimator.pmsm_params_data.L_s;
  float Ts = g_smo_pll_estimator.Ts;
  uint8_t pole_pairs = g_smo_pll_estimator.pmsm_params_data.pole_pairs;

  // Debugging: Counter for periodic logging
  static uint32_t s_debug_smo_counter = 0;
  const uint32_t DEBUG_SMO_LOG_INTERVAL = 1000;  // Log every 1000 calls

  // Safety check for parameters
  if (L_s <= 1e-9f || R_s <= 1e-9f || Ts <= 1e-9f ||
      pole_pairs ==
          0) {  // Added more robust checks, L_s, R_s, Ts should be positive
    // Log error or handle appropriately
    g_smo_pll_estimator.estimated_speed_rpm =
        0.0f;  // Reset speed if params invalid
    g_smo_pll_estimator.estimated_theta_e = 0.0f;
    // Consider logging an error here
    return;
  }

  // Calculate current estimation error
  float err_ia = g_smo_pll_estimator.i_alpha_hat - i_ab.alpha;
  float err_ib = g_smo_pll_estimator.i_beta_hat - i_ab.beta;

  // Initialize states if they become NaN (already done in init, but good for
  // robustness during runtime)
  if (isnan(g_smo_pll_estimator.estimated_speed_rpm)) {
    g_smo_pll_estimator.estimated_speed_rpm = 0.0f;
  }
  if (isnan(g_smo_pll_estimator.pll_integrator)) {
    g_smo_pll_estimator.pll_integrator = 0.0f;
  }
  if (isnan(g_smo_pll_estimator.estimated_theta_e)) {
    g_smo_pll_estimator.estimated_theta_e = 0.0f;
  }
  if (isnan(g_smo_pll_estimator.i_alpha_hat)) {
    g_smo_pll_estimator.i_alpha_hat = 0.0f;
  }
  if (isnan(g_smo_pll_estimator.i_beta_hat)) {
    g_smo_pll_estimator.i_beta_hat = 0.0f;
  }

  // Update estimated currents (Standard SMO formulation, removed
  // compensation_alpha/beta from original attachment) L_s * di/dt = V - R*i -
  // E_bemf di/dt = (1/L_s) * (V - R*i - E_bemf_estimated) E_bemf_estimated is
  // z_alpha, z_beta (before LPF)
  float di_alpha_hat_dt =
      (1.0f / L_s) * (v_ab.alpha - R_s * g_smo_pll_estimator.i_alpha_hat -
                      g_smo_pll_estimator.z_alpha);
  float di_beta_hat_dt =
      (1.0f / L_s) * (v_ab.beta - R_s * g_smo_pll_estimator.i_beta_hat -
                      g_smo_pll_estimator.z_beta);

  // Safety check for current derivatives before integration
  if (isnan(di_alpha_hat_dt) || !isfinite(di_alpha_hat_dt))
    di_alpha_hat_dt = 0.0f;
  if (isnan(di_beta_hat_dt) || !isfinite(di_beta_hat_dt)) di_beta_hat_dt = 0.0f;

  // if (!isnan(di_alpha_hat_dt) && !isnan(di_beta_hat_dt) &&
  // isfinite(di_alpha_hat_dt) && isfinite(di_beta_hat_dt)) { // Original check
  g_smo_pll_estimator.i_alpha_hat += di_alpha_hat_dt * Ts;
  g_smo_pll_estimator.i_beta_hat += di_beta_hat_dt * Ts;
  // } else { // Original else block
  // Potentially log an error if derivatives are NaN/Inf
  // For now, skip current update to prevent propagation of NaN/Inf
  // }

  // Update sliding mode control inputs
  // The saturation range for err_ia, err_ib (-1.0f, 1.0f) might need adjustment
  // based on expected current errors (in Amperes) and Kslide value.
  float sat_alpha =
      saturate(err_ia, -20.0f,
               20.0f);  // Increased saturation range from +/-1.0A to +/-20.0A
  float sat_beta =
      saturate(err_ib, -20.0f,
               20.0f);  // Increased saturation range from +/-1.0A to +/-20.0A

  // The sign of Kslide in z_alpha/beta depends on the error definition and
  // current model. Original: z_alpha = -g_smo_pll_estimator.Kslide * sat_alpha;
  // This was kept.
  g_smo_pll_estimator.z_alpha = -g_smo_pll_estimator.Kslide * sat_alpha;
  g_smo_pll_estimator.z_beta = -g_smo_pll_estimator.Kslide * sat_beta;

  // Low-pass filter the estimated BEMF (z_alpha, z_beta are raw BEMF estimates)
  if (g_smo_pll_estimator.lpf_bemf_coeff < 1.0f &&
      g_smo_pll_estimator.lpf_bemf_coeff > 0.0f) {  // ensure coeff is valid
    g_smo_pll_estimator.e_alpha =
        g_smo_pll_estimator.lpf_bemf_coeff * g_smo_pll_estimator.z_alpha +
        (1.0f - g_smo_pll_estimator.lpf_bemf_coeff) *
            g_smo_pll_estimator.bemf_alpha_lpf_prev;
    g_smo_pll_estimator.e_beta =
        g_smo_pll_estimator.lpf_bemf_coeff * g_smo_pll_estimator.z_beta +
        (1.0f - g_smo_pll_estimator.lpf_bemf_coeff) *
            g_smo_pll_estimator.bemf_beta_lpf_prev;
  } else {  // If LPF coeff is 1.0 or invalid, no filtering or pass through z
            // directly
    g_smo_pll_estimator.e_alpha = g_smo_pll_estimator.z_alpha;
    g_smo_pll_estimator.e_beta = g_smo_pll_estimator.z_beta;
  }

  // Safety check for BEMF estimates before use in PLL
  if (isnan(g_smo_pll_estimator.e_alpha) ||
      !isfinite(g_smo_pll_estimator.e_alpha)) {
    g_smo_pll_estimator.e_alpha = 0.0f;
  }
  if (isnan(g_smo_pll_estimator.e_beta) ||
      !isfinite(g_smo_pll_estimator.e_beta)) {
    g_smo_pll_estimator.e_beta = 0.0f;
  }

  g_smo_pll_estimator.bemf_alpha_lpf_prev = g_smo_pll_estimator.e_alpha;
  g_smo_pll_estimator.bemf_beta_lpf_prev = g_smo_pll_estimator.e_beta;

  float cos_theta_est = cosf(g_smo_pll_estimator.estimated_theta_e);
  float sin_theta_est = sinf(g_smo_pll_estimator.estimated_theta_e);

  // Calculate PLL error (e_q component of BEMF in estimated frame)
  // e_q_est = e_beta * cos_theta_est - e_alpha * sin_theta_est
  float pll_error = -g_smo_pll_estimator.e_alpha * sin_theta_est +
                    g_smo_pll_estimator.e_beta * cos_theta_est;

  // Safety check for PLL error
  if (isnan(pll_error) || !isfinite(pll_error)) {
    pll_error = 0.0f;
  }

  // Update PLL integrator with anti-windup
  // Max electrical speed: 200 Hz. For P=2, this is 6000 RPM.
  // This limit should be appropriate for most motors unless very high speed.
  float max_electrical_speed_rad_s =
      TWO_PI * 200.0f;  // Max 200Hz electrical frequency
  float min_electrical_speed_rad_s =
      -max_electrical_speed_rad_s;  // Allow for negative speeds if applicable

  float integrator_increment = g_smo_pll_estimator.Ki_pll * pll_error * Ts;
  if (isnan(integrator_increment) || !isfinite(integrator_increment))
    integrator_increment = 0.0f;

  g_smo_pll_estimator.pll_integrator += integrator_increment;
  g_smo_pll_estimator.pll_integrator =
      saturate(g_smo_pll_estimator.pll_integrator,
               min_electrical_speed_rad_s,  // Use min for symmetry
               max_electrical_speed_rad_s);

  // Calculate estimated speed (electrical rad/s)
  float estimated_speed_rad_s_prop = g_smo_pll_estimator.Kp_pll * pll_error;
  if (isnan(estimated_speed_rad_s_prop) ||
      !isfinite(estimated_speed_rad_s_prop))
    estimated_speed_rad_s_prop = 0.0f;

  float estimated_speed_rad_s =
      estimated_speed_rad_s_prop + g_smo_pll_estimator.pll_integrator;

  // Safety check for estimated electrical speed
  if (isnan(estimated_speed_rad_s) || !isfinite(estimated_speed_rad_s)) {
    estimated_speed_rad_s = 0.0f;
  }
  // Additional saturation for the final estimated_speed_rad_s can be useful
  estimated_speed_rad_s =
      saturate(estimated_speed_rad_s, min_electrical_speed_rad_s,
               max_electrical_speed_rad_s);

  // Update angle
  float angle_increment = estimated_speed_rad_s * Ts;
  if (isnan(angle_increment) || !isfinite(angle_increment))
    angle_increment = 0.0f;

  g_smo_pll_estimator.estimated_theta_e += angle_increment;
  g_smo_pll_estimator.estimated_theta_e =
      fmodf(g_smo_pll_estimator.estimated_theta_e, TWO_PI);
  if (g_smo_pll_estimator.estimated_theta_e < 0.0f) {
    g_smo_pll_estimator.estimated_theta_e += TWO_PI;
  }

  // Convert to RPM
  if (pole_pairs > 0) {
    g_smo_pll_estimator.estimated_speed_rpm =
        estimated_speed_rad_s * (60.0f / TWO_PI) / pole_pairs;
  } else {
    g_smo_pll_estimator.estimated_speed_rpm = 0.0f;  // Avoid division by zero
  }

  // Final safety check for RPM (redundant if estimated_speed_rad_s is checked,
  // but good practice)
  if (isnan(g_smo_pll_estimator.estimated_speed_rpm) ||
      !isfinite(g_smo_pll_estimator.estimated_speed_rpm)) {
    g_smo_pll_estimator.estimated_speed_rpm = 0.0f;
  }
}

float smo_pll_estimator_get_angle() {
  return g_smo_pll_estimator.estimated_theta_e;
}

float smo_pll_estimator_get_speed_rpm() {
  static uint32_t tick = 0;
  if (tick++ == 10000) {
    tick = 0;
    debug("speed: %f rpm", g_smo_pll_estimator.estimated_speed_rpm);
  }
  return g_smo_pll_estimator.estimated_speed_rpm;
}

float smo_pll_estimator_get_speed_rad_s() {
  // Ensure pole_pairs is not zero to prevent division by zero if
  // pmsm_params_data is not fully initialized
  if (g_smo_pll_estimator.pmsm_params_data.pole_pairs == 0) return 0.0f;
  return g_smo_pll_estimator.estimated_speed_rpm * (TWO_PI / 60.0f) *
         g_smo_pll_estimator.pmsm_params_data.pole_pairs;
}

smo_pll_estimator_t* smo_pll_estimator_get() { return &g_smo_pll_estimator; }
