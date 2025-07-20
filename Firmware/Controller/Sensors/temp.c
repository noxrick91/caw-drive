#include "./temp.h"

#include "adc.h"
#include "math.h"

#define ADC_RESOLUTION 4096.0f
#define ADC_VOLTAGE_REF 3.3f
#define R 10000.0f      // 分压电阻阻值
#define BETA 3455.0f    // 热敏电阻的 B 常数
#define K 273.15f       // 摄氏温度转绝对温度的偏移量
#define T0 (K + 25.0f)  // 25°C 的绝对温度 (K)

void temp_init(void) {}

float temp_get(void) {
  // 启动 ADC 并等待转换完成
  HAL_ADC_Start(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 10);
  // 获取 ADC 采样值
  uint32_t adc_value = HAL_ADC_GetValue(&hadc3);
  float vr = (float)adc_value / ADC_RESOLUTION * ADC_VOLTAGE_REF;
  float rt = vr * R / (ADC_VOLTAGE_REF - vr);
  // 根据 B 常数公式计算温度
  float temp_k = 1.0f / (1.0f / T0 + logf(rt / R) / BETA);
  float temp_c = temp_k - K;  // 转换为摄氏温度
  return temp_c;
}
