#include "./svpwm.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>

#include "../../Utils/log.h"
#include "../../math_const.h"

static svpwm_t g_svpwm;

void svpwm_init(float Tpwm) {
  debug("Tpwm: %f", Tpwm);
  g_svpwm.Tpwm = Tpwm;
}

void svpwm_calculate(float Vdc, float alpha, float beta, float* ret,
                     bool zero_seq) {
  float Vref_mag = sqrtf(alpha * alpha + beta * beta);
  float angle_rad = atan2f(beta, alpha);
  if (angle_rad < 0) angle_rad += 2.0f * PI;

  int sector = (int)(angle_rad / (PI / 3.0f)) + 1;
  sector = (sector - 1) % 6 + 1;
  float angle_in_sector_rad = angle_rad - (sector - 1) * (PI / 3.0f);

  float time_factor = (SQRT3 * g_svpwm.Tpwm) / Vdc;
  float Tx = time_factor * Vref_mag * sinf(PI / 3.0f - angle_in_sector_rad);
  float Ty = time_factor * Vref_mag * sinf(angle_in_sector_rad);

  // 过调制区归一化
  if (Tx + Ty > g_svpwm.Tpwm) {
    float scale = g_svpwm.Tpwm / (Tx + Ty);
    Tx *= scale;
    Ty *= scale;
  }
  if (Tx < 0) Tx = 0;
  if (Ty < 0) Ty = 0;

  float T0 = g_svpwm.Tpwm - Tx - Ty;
  if (T0 < 0) T0 = 0;

  float Tga = 0, Tgb = 0, Tgc = 0;
  switch (sector) {
    case 1:
      Tga = Tx + Ty + T0 / 2.0f;
      Tgb = Ty + T0 / 2.0f;
      Tgc = T0 / 2.0f;
      break;
    case 2:
      Tga = Tx + T0 / 2.0f;
      Tgb = Tx + Ty + T0 / 2.0f;
      Tgc = T0 / 2.0f;
      break;
    case 3:
      Tga = T0 / 2.0f;
      Tgb = Tx + Ty + T0 / 2.0f;
      Tgc = Ty + T0 / 2.0f;
      break;
    case 4:
      Tga = T0 / 2.0f;
      Tgb = Tx + T0 / 2.0f;
      Tgc = Tx + Ty + T0 / 2.0f;
      break;
    case 5:
      Tga = Ty + T0 / 2.0f;
      Tgb = T0 / 2.0f;
      Tgc = Tx + Ty + T0 / 2.0f;
      break;
    case 6:
      Tga = Tx + Ty + T0 / 2.0f;
      Tgb = T0 / 2.0f;
      Tgc = Tx + T0 / 2.0f;
      break;
    default:
      Tga = Tgb = Tgc = 0;
      break;
  }

  // 零序注入
  if (zero_seq) {
    float vmax = fmaxf(fmaxf(Tga, Tgb), Tgc);
    float vmin = fminf(fminf(Tga, Tgb), Tgc);
    float v0 = -0.5f * (vmax + vmin);
    Tga += v0;
    Tgb += v0;
    Tgc += v0;
  }

  // 占空比归一化并限幅
  ret[0] = fminf(fmaxf(Tga / g_svpwm.Tpwm, 0.0f), 1.0f);
  ret[1] = fminf(fmaxf(Tgb / g_svpwm.Tpwm, 0.0f), 1.0f);
  ret[2] = fminf(fmaxf(Tgc / g_svpwm.Tpwm, 0.0f), 1.0f);
}