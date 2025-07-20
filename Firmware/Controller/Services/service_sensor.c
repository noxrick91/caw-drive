#include "service_sensor.h"

#include <stdint.h>

#include "../Drivers/mt6835/mt6835.h"

void service_sensor_periodic() {
  static uint8_t call_counter = 0;

  // call_counter++;
  // if (call_counter >= 10) {
  //   call_counter = 0;
  mt6835_update();
  // }
}