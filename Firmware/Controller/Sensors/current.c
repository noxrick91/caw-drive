#include "./current.h"

#include <string.h>

#include "adc.h"
#include "cmsis_os.h"
#include "dev_usart.h"
#include "stm32f4xx_hal_adc_ex.h"

extern osSemaphoreId controlBinarySemHandle;

#define ADC_RESOLUTION 4096.0
#define ADC_VOLTAGE_REF 3.3
#define SHUNT_RESISTANCE 0.005
#define AMP_GAIN 40.0

// 将偏移量初始化为ADC的中间值, 这是一个安全的默认值
static float g_ia_offset = 2048.0f;
static float g_ib_offset = 2048.0f;

static float g_gain = 0.0;
static float g_ia = 0.0;
static float g_ib = 0.0;
static float g_ic = 0.0;

static float g_current[3] = {0.0f, 0.0f, 0.0f};

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (ADC1 == hadc->Instance) {
    float adc1_in0 = hadc1.Instance->JDR1;
    g_ia = (adc1_in0 - g_ia_offset) * g_gain;
  } else if (ADC2 == hadc->Instance) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    float adc2_in1 = hadc2.Instance->JDR1;
    g_ib = (adc2_in1 - g_ib_offset) * g_gain;

    // 1. 计算 Ic
    g_ic = -g_ia - g_ib;
    // 2. 准备好要传递给 FOC 的数据
    g_current[0] = g_ia;
    g_current[1] = g_ib;
    g_current[2] = g_ic;

    // 3. 唤醒 FOC 任务
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(controlBinarySemHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }
}

void current_init(void) {
  // 计算增益(计算出电压值 除以 采样电阻值 除以 放大倍数)
  g_gain = (ADC_VOLTAGE_REF / ADC_RESOLUTION) / (SHUNT_RESISTANCE * AMP_GAIN);
}

void current_offset_calibration(void) {
  const int num_samples = 256;
  uint32_t offset_a_sum = 0;
  uint32_t offset_b_sum = 0;

  HAL_ADCEx_InjectedStart(&hadc1);
  HAL_ADCEx_InjectedStart(&hadc2);

  for (int i = 0; i < num_samples; i++) {
    if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 10) == HAL_OK) {
      offset_a_sum += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    }
    if (HAL_ADCEx_InjectedPollForConversion(&hadc2, 10) == HAL_OK) {
      offset_b_sum += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    }
    HAL_Delay(1);  // 每次采样间短暂延时, 确保稳定性
  }

  HAL_ADCEx_InjectedStop(&hadc1);
  HAL_ADCEx_InjectedStop(&hadc2);

  g_ia_offset = (float)offset_a_sum / num_samples;
  g_ib_offset = (float)offset_b_sum / num_samples;

  // 校准完成后, 以中断模式重启ADC, 用于后续的实时控制
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc2);
  HAL_ADCEx_InjectedStart_IT(&hadc3);
}

int current_get(float* arr) {
  memcpy(arr, g_current, sizeof(g_current));
  return 0;
}