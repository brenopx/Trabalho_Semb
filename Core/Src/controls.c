#include "controls.h"
#include <string.h>

// ===================== Configs =====================
#define AVG_WINDOW_FRAMES      2   // média de 2 envios
#define SPEED_DZ_LOW_RAW       80  // zona morta baixa (ADC counts)
#define SPEED_DZ_HIGH_RAW      80  // zona morta alta  (ADC counts)
#define STEER_CENTER_DZ_DEG    8   // ±8° no centro -> 180 travado
#define STEER_QUANT_STEP_DEG   5   // volante sensível em passos de 5°
#define ADC_POLL_TIMEOUT_MS    5
// ===================================================

static ADC_HandleTypeDef *s_hadc = NULL;

static ControlsCalib s_cal = {
  .steer_raw_min = 0, .steer_raw_max = 4095,
  .speed_raw_min = 0, .speed_raw_max = 4095,
};

static uint16_t s_hist_steer[AVG_WINDOW_FRAMES] = {0};
static uint16_t s_hist_speed[AVG_WINDOW_FRAMES] = {0};
static uint8_t  s_hist_index = 0;
static uint32_t s_frames_sent = 0;

static inline uint16_t clamp_u16(uint16_t x, uint16_t lo, uint16_t hi){
  return (x < lo) ? lo : (x > hi ? hi : x);
}

static inline uint16_t map_u16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max){
  if (in_max <= in_min) return out_min;
  x = clamp_u16(x, in_min, in_max);
  uint32_t num = (uint32_t)(x - in_min) * (out_max - out_min);
  uint32_t den = (in_max - in_min);
  return (uint16_t)(out_min + (num / den));
}

static uint8_t speed_with_edge_deadzones(uint16_t raw, uint16_t raw_min, uint16_t raw_max){
  raw = clamp_u16(raw, raw_min, raw_max);
  uint16_t lo = raw_min + SPEED_DZ_LOW_RAW;
  uint16_t hi = raw_max - SPEED_DZ_HIGH_RAW;

  if (lo >= hi) {
    return (uint8_t)map_u16(raw, raw_min, raw_max, 0, 255);
  }
  if (raw <= lo) return 0;
  if (raw >= hi) return 255;
  return (uint8_t)map_u16(raw, lo, hi, 0, 255);
}

static uint16_t quantize_to_step_u16(uint16_t value, uint16_t step){
  uint16_t q = (uint16_t)((value + step/2) / step) * step; // arredonda
  return q;
}

static uint16_t steer_deg_with_center_deadzone(uint16_t raw, const ControlsCalib *cal){
  uint16_t deg = map_u16(raw, cal->steer_raw_min, cal->steer_raw_max, 0, 360);
  if (deg >= (180 - STEER_CENTER_DZ_DEG) && deg <= (180 + STEER_CENTER_DZ_DEG)) {
    deg = 180;
  }
  deg = quantize_to_step_u16(deg, STEER_QUANT_STEP_DEG);
  if (deg > 360) deg = 360;
  return deg;
}

void Controls_Init(ADC_HandleTypeDef *hadc){
  s_hadc = hadc;
  s_hist_index = 0;
  s_frames_sent = 0;
  memset(s_hist_steer, 0, sizeof(s_hist_steer));
  memset(s_hist_speed, 0, sizeof(s_hist_speed));
}

void Controls_SetCalibration(const ControlsCalib *c){
  if (!c) return;
  s_cal = *c;
  if (s_cal.steer_raw_max <= s_cal.steer_raw_min) { s_cal.steer_raw_min = 0; s_cal.steer_raw_max = 4095; }
  if (s_cal.speed_raw_max <= s_cal.speed_raw_min) { s_cal.speed_raw_min = 0; s_cal.speed_raw_max = 4095; }
}

void Controls_ReadOnce(ControlsState *out){
  if (!s_hadc || !out) return;

  HAL_ADC_Start(s_hadc);
  uint16_t raw_steer = 0, raw_speed = 0;

  if (HAL_ADC_PollForConversion(s_hadc, ADC_POLL_TIMEOUT_MS) == HAL_OK)
    raw_steer = (uint16_t)HAL_ADC_GetValue(s_hadc);
  if (HAL_ADC_PollForConversion(s_hadc, ADC_POLL_TIMEOUT_MS) == HAL_OK)
    raw_speed = (uint16_t)HAL_ADC_GetValue(s_hadc);

  HAL_ADC_Stop(s_hadc);

  s_hist_steer[s_hist_index] = raw_steer;
  s_hist_speed[s_hist_index] = raw_speed;
  s_hist_index = (s_hist_index + 1) % AVG_WINDOW_FRAMES;

  uint32_t sum_steer = 0, sum_speed = 0;
  for (uint8_t i = 0; i < AVG_WINDOW_FRAMES; ++i) {
    sum_steer += s_hist_steer[i];
    sum_speed += s_hist_speed[i];
  }
  uint16_t avg_steer = (uint16_t)(sum_steer / AVG_WINDOW_FRAMES);
  uint16_t avg_speed = (uint16_t)(sum_speed / AVG_WINDOW_FRAMES);

  if (s_frames_sent < 2) {
    out->adc_raw_steer = 0;
    out->adc_raw_speed = 0;
    out->curve_direction = 180; // centro
    out->speed = 0;
  } else {
    out->adc_raw_steer = avg_steer;
    out->adc_raw_speed = avg_speed;
    out->curve_direction = steer_deg_with_center_deadzone(avg_steer, &s_cal);
    out->speed = speed_with_edge_deadzones(avg_speed, s_cal.speed_raw_min, s_cal.speed_raw_max);
  }

  GPIO_PinState s = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
  out->movement_direction = (s == GPIO_PIN_SET) ? 1 : 0;

  s_frames_sent++;
}
