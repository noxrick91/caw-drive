#include "./vbus.h"

#include "adc.h"

#define ADC_RESOLUTION 4096.0f
#define ADC_VOLTAGE_REF 3.3f
#define R1 75000.0f
#define R2 3000.0f

static float g_vbus_gain = 0.0f;
static float g_vbus = 0.0f;

void vbus_init(void) {
  g_vbus_gain = ADC_VOLTAGE_REF / ADC_RESOLUTION * (R1 + R2) / R2;
}

float vbus_get(void) {
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);
  g_vbus = HAL_ADC_GetValue(&hadc1) * g_vbus_gain;
  return g_vbus;
}