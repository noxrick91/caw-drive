#include "./controller.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#include <math.h>  // For sqrtf
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "./PWM/pwm.h"          // PWM控制头文件
#include "./Sensors/current.h"  // 电流传感器头文件
#include "./State/state.h"      // 系统状态管理头文件
#include "./Utils/log.h"
#include "./temp.h"  // 温度传感器头文件
#include "Algorithm/Sensors/kalman_velocity_estimator.h"
#include "Algorithm/Sensors/smo_pll_estimator.h"  // SMO和PLL估算器头文件
#include "Algorithm/Sensors/velocity_estimator.h"
#include "Algorithm/if.h"
#include "Algorithm/motor_params.h"
#include "Algorithm/transform.h"
#include "Comm/code.h"
#include "Devices/dev_usart.h"      // USART设备头文件
#include "Drivers/mt6835/mt6835.h"  // 包含MT6835驱动头文件
#include "Sensors/vbus.h"           // VBUS电压传感器头文件
#include "Services/service_discover.h"
#include "Services/service_feedback.h"
#include "Services/service_message.h"
#include "Services/service_observer.h"
#include "Services/service_offline_ident.h"
#include "Services/service_sensor.h"
#include "Tasks/task_config.h"
#include "Tasks/task_ident.h"
#include "Tasks/task_state.h"
#include "Utils/log.h"
#include "setup.h"

controller_t g_controller;
bool g_init = false;

static float normalize_angle(float angle) {
  float result = fmodf(angle, 2.0f * M_PI);
  if (result < 0.0f) {
    result += 2.0f * M_PI;
  }
  return result;
}

void controller_align_sensor() {
  config_t* cfg = config_get();
  const float alignment_duty_cycle = 0.15f;
  const uint32_t alignment_duration_ms = 2000;
  const int num_samples = 100;  // Number of samples to average for offset
  const uint32_t sample_delay_ms = 10;  // Delay between samples

  debug("Aligning motor and sensor...");
  pwm_start();

  // Apply a fixed magnetic field along the A-phase axis
  TIM1->CCR1 = (uint32_t)(alignment_duty_cycle * (float)(TIM1->ARR + 1));
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;

  // Wait for the motor to settle
  HAL_Delay(alignment_duration_ms);

  // Read the mechanical angle multiple times and average to get a stable offset
  float angle_sum = 0.0f;
  for (int i = 0; i < num_samples; ++i) {
    mt6835_update();
    angle_sum += mt6835_get_single_turn_angle();
    HAL_Delay(sample_delay_ms);
  }
  float mechanical_angle_offset = angle_sum / num_samples;

  // Convert the mechanical offset to an electrical offset
  g_controller.zero_electric_angle =
      normalize_angle(mechanical_angle_offset * cfg->motor_params.pole_pairs);

  // Stop all PWM outputs
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  pwm_stop();
  debug(
      "Alignment complete. Zero electrical angle offset = %.4f rad (from %.4f "
      "rad mechanical)",
      g_controller.zero_electric_angle, mechanical_angle_offset);
}

// 控制器初始化函数
void controller_init() {
  config_init(1, 1000);
  config_t* cfg = config_get();
  cfg->target = 0.0f;

  service_message_register(main_code_motor, sub_code_motor_load_config,
                           task_config_load);
  service_message_register(main_code_motor, sub_code_motor_update_config,
                           task_config_update);
  service_message_register(main_code_motor, sub_code_motor_init_config,
                           task_config_init);
  service_message_register(main_code_motor, sub_code_motor_flash_config,
                           task_config_flash);
  service_message_register(main_code_motor, sub_code_motor_ident, task_ident);
  service_message_register(main_code_motor, sub_code_motor_update_state,
                           task_state_update);

  dev_usart_init();

  info("caw drive init");

  drv8323_spi_reset();
  HAL_Delay(100);
  setup_drv8323(&(g_controller.driver));
  mt6835_spi_reset();
  HAL_Delay(100);
  mt6835_init(&hspi3);

  current_init();
  vbus_init();
  pwm_init();
  // 启动PWM前先对当前电流采样进行校准，获得偏移值
  current_offset_calibration();

  motor_params_init(&cfg->motor_params, cfg->motor_params.pole_pairs, 1.0f,
                    1000);

  cfg->sensor = SENSOR_TYPE_SMO;
  cfg->ctl_mode = CONTROL_MODE_VF;

  svpwm_init(cfg->Tpwm);

  g_controller.v_bus = vbus_get();

  pmsm_params_t motor_model_params_for_smo;
  motor_model_params_for_smo.R_s = cfg->motor_params.phase_resistance;
  // SMO通常假设 Ld = Lq。使用其中一个值，例如 Ld。
  motor_model_params_for_smo.L_s = cfg->motor_params.inductance_d;
  motor_model_params_for_smo.psi_f = cfg->motor_params.flux_linkage;
  // SMO可能不直接使用此值，但保持结构完整
  motor_model_params_for_smo.pole_pairs = cfg->motor_params.pole_pairs;

  // SMO和PLL的调试参数
  float k_slide = 1.0f;                 // 滑模增益
  float kp_pll = 200.0f;                // PLL比例增益
  float ki_pll = 50.0f;                 // PLL积分增益
  float bemf_lpf_cutoff_freq = 150.0f;  // BEMF低通滤波器截止频率 (Hz)

  smo_pll_estimator_init(&motor_model_params_for_smo, k_slide, kp_pll, ki_pll,
                         cfg->Tpwm,  // PWM周期/采样时间 (Ts)
                         bemf_lpf_cutoff_freq);

  g_controller.v_bus = vbus_get();
  if (g_controller.v_bus < 1.0f) {
    warn("Vbus reading low (%.2fV) during vf_init, using default.",
         g_controller.v_bus);
  }

  controller_align_sensor();

  float pwm_frequency_hz = 1.0f / cfg->Tpwm;

  vf_init(&g_controller.vf_controller, 100.0f, -100.0f, g_controller.v_bus,
          pwm_frequency_hz, cfg->Tpwm, 3.0);

  if_init(&g_controller.if_controller, cfg->Tpwm);

  if_set_pi_params(cfg->foc_params.iq_pid.Kp, cfg->foc_params.iq_pid.Ki);
  debug("IF PI params loaded from config: Kp=%.2f, Ki=%.2f",
        cfg->foc_params.iq_pid.Kp, cfg->foc_params.iq_pid.Ki);

  foc_init(&cfg->foc_params, cfg->Tpwm, g_controller.v_bus);

  g_foc.control_mode = FOC_CONTROL_MODE_SPEED;
  g_controller.electrical_angle = 0.0f;
  g_controller.velocity_rpm = 0.0f;

  kalman_velocity_estimator_init(&g_controller.velocity_estimator, 0.1f, 5.0f,
                                 0.01f, mt6835_get_single_turn_angle());
  debug("Kalman velocity estimator initialized.");

  g_controller.last_tick = HAL_GetTick();  // Initialize tick for dt calculation
  config_set_state(0);
}

// 控制器步进函数 (当前未使用，但保留用于未来可能的周期性非中断驱动任务)
void controller_step() {
  float local_currents[3];
  float local_V_ab[2];
  float local_theta;

  // 获取参数快照，需短暂禁用中断
  __disable_irq();
  local_currents[0] = g_controller.phase_currents[0];
  local_currents[1] = g_controller.phase_currents[1];
  local_currents[2] = g_controller.phase_currents[2];
  local_V_ab[0] = g_controller.vab_measured.alpha;
  local_V_ab[1] = g_controller.vab_measured.beta;
  local_theta = vf_get_angle(&g_controller.vf_controller);
  __enable_irq();

  g_controller.temp[0] = temp_get();
  g_controller.v_bus = vbus_get();

  service_discover_periodic();

  service_offline_ident_periodic(local_currents, local_V_ab, local_theta);
}

// PWM中断回调函数 (核心控制逻辑)
void controller_core_step() {
  config_t* cfg = config_get();

  // --- Calculate dt ---
  uint32_t current_tick = HAL_GetTick();
  float dt = (current_tick - g_controller.last_tick) / 1000.0f;
  g_controller.last_tick = current_tick;

  float pwm_duties[3];
  float v_ab_command[2] = {0.0f, 0.0f};

  service_sensor_periodic();
  current_get(g_controller.phase_currents);

  float mechanical_angle = mt6835_get_single_turn_angle();

  // 获取多圈角度
  g_controller.multi_turn_angle = mt6835_get_multi_turn_angle();

  // 1.2 计算校正后的电角度
  // 首先将机械角度转换为原始电角度
  float raw_electrical_angle =
      normalize_angle(mechanical_angle * cfg->motor_params.pole_pairs);
  // 然后减去在对齐过程中测得的零点偏移
  g_controller.electrical_angle =
      normalize_angle(raw_electrical_angle - g_controller.zero_electric_angle);

  // 1.3 更新并获取速度
  float velocity_rad_s = kalman_velocity_estimator_update(
      &g_controller.velocity_estimator, mechanical_angle, dt);
  g_controller.velocity_rpm = velocity_rad_s * 60.0f / (2.0f * M_PI);

  // Periodic feedback of key data points
  service_feedback_periodic(g_controller.v_bus, g_controller.velocity_rpm,
                            g_controller.multi_turn_angle, g_controller.temp[0],
                            0.0);

  // 2. Clark & Park 变换

  float i_ab[2];
  clark(g_controller.phase_currents[0], g_controller.phase_currents[1],
        g_controller.phase_currents[2], i_ab);
  g_controller.iab_measured.alpha = i_ab[0];
  g_controller.iab_measured.beta = i_ab[1];

  float idq_meas[2];  // [0] for id, [1] for iq
  park(g_controller.iab_measured.alpha, g_controller.iab_measured.beta,
       g_controller.electrical_angle, idq_meas);

  // 将测量的d-q电流存储在全局控制器结构中，以便其他模块访问
  g_controller.id_measured = idq_meas[0];
  g_controller.iq_measured = idq_meas[1];

  if (0 == config_get_state()) return;

  // 3. 控制模式逻辑 (V/F, IF, FOC)
  if (CONTROL_MODE_VF == cfg->ctl_mode) {
    float target_frequency = cfg->target;
    vf_step(&g_controller.vf_controller, target_frequency);
    voltage_vector_t v_out_vf =
        vf_get_voltage_vector(&g_controller.vf_controller);
    v_ab_command[0] = v_out_vf.alpha;
    v_ab_command[1] = v_out_vf.beta;
  } else if (CONTROL_MODE_IF == cfg->ctl_mode) {
    float target_frequency = cfg->target;
    float target_current = 1.0f;
    float i_mag_measured = sqrtf(
        g_controller.iab_measured.alpha * g_controller.iab_measured.alpha +
        g_controller.iab_measured.beta * g_controller.iab_measured.beta);
    if_step(&g_controller.if_controller, target_current, target_frequency,
            i_mag_measured, &v_ab_command[0], &v_ab_command[1]);
  } else if (CONTROL_MODE_FOC_TORQUE == cfg->ctl_mode ||
             CONTROL_MODE_FOC_VELOCITY == cfg->ctl_mode ||
             CONTROL_MODE_FOC_POSITION == cfg->ctl_mode) {
    if (CONTROL_MODE_FOC_TORQUE == cfg->ctl_mode) {
      g_foc.control_mode = FOC_CONTROL_MODE_TORQUE;
      g_foc.iq_ref = cfg->target;
    } else if (CONTROL_MODE_FOC_VELOCITY == cfg->ctl_mode) {
      g_foc.control_mode = FOC_CONTROL_MODE_SPEED;
      g_foc.speed_ref = cfg->target;
    } else if (CONTROL_MODE_FOC_POSITION == cfg->ctl_mode) {
      g_foc.control_mode = FOC_CONTROL_MODE_POSITION;
      g_foc.position_ref = cfg->target;
    }
    // FOC控制器步进
    float v_d, v_q;
    foc_step(g_controller.id_measured, g_controller.iq_measured,
             g_controller.velocity_rpm, g_controller.multi_turn_angle, &v_d,
             &v_q);

    // 将指令电压存入全局结构体，供外部模块访问
    g_controller.vd_command = v_d;
    g_controller.vq_command = v_q;

    // 反Park变换
    ipark(g_controller.vd_command, g_controller.vq_command,
          g_controller.electrical_angle, v_ab_command);

  } else {
    v_ab_command[0] = 0.0f;
    v_ab_command[1] = 0.0f;
  }

  g_controller.vab_measured.alpha = v_ab_command[0];
  g_controller.vab_measured.beta = v_ab_command[1];

  // 4. 传感器估算 (SMO)
  if (cfg->sensor == SENSOR_TYPE_SMO) {
    // voltage_vector_t v_ab_input_to_smo;
    // v_ab_input_to_smo.alpha = v_ab_command[0];
    // v_ab_input_to_smo.beta = v_ab_command[1];
    // smo_pll_estimator_step(v_ab_input_to_smo, i_ab_measured);
    // g_controller.electrical_angle = smo_pll_estimator_get_angle();
    // g_controller.speed_rpm = smo_pll_estimator_get_speed_rpm();
  }

  // 5. SVPWM 计算
  svpwm_calculate(g_controller.v_bus, v_ab_command[0], v_ab_command[1],
                  pwm_duties, false);

  // 6. 更新PWM占空比
  pwm_set_duty_cycle(pwm_duties[0], pwm_duties[1], pwm_duties[2]);

  static uint16_t tick = 0;
  if (tick++ > 1000) {
    tick = 0;
    debug(
        "Velocity Mode: target=%.2f, speed_ref=%.2f, measured_rpm=%.2f, "
        "speed_error=%.2f",
        cfg->target, g_foc.speed_ref, g_controller.velocity_rpm,
        g_foc.speed_ref - g_controller.velocity_rpm);
    debug("iq_ref=%.2f, electrical_angle=%.2f", g_foc.iq_ref,
          g_controller.electrical_angle);
  }

  // 7. 数据上报
  float ovserver_data[6];
  ovserver_data[0] = g_controller.phase_currents[0];
  ovserver_data[1] = g_controller.phase_currents[1];
  ovserver_data[2] = idq_meas[0];
  ovserver_data[3] = idq_meas[1];
  ovserver_data[4] = g_controller.velocity_rpm;  // 上报滤波后的速度rpm
  ovserver_data[5] = mechanical_angle;

  service_observer_periodic(ovserver_data);
}