#pragma once
#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  // ADC brutos (após média de janela)
  uint16_t adc_raw_steer;   // 0..4095
  uint16_t adc_raw_speed;   // 0..4095
  // Valores normalizados
  uint16_t curve_direction; // 0..360 (quantizado em passos de 5°)
  uint8_t  speed;           // 0..255 (com zonas mortas nas pontas)
  uint8_t  movement_direction; // 1 frente, 0 ré
} ControlsState;

typedef struct {
  // Calibração do volante
  uint16_t steer_raw_min;   // valor no mínimo mecânico
  uint16_t steer_raw_max;   // valor no máximo mecânico
  // Calibração da velocidade (opcional)
  uint16_t speed_raw_min;   // normalmente 0
  uint16_t speed_raw_max;   // normalmente 4095
} ControlsCalib;

// Inicializa módulo (guarda ponteiro do ADC1)
void Controls_Init(ADC_HandleTypeDef *hadc);

// Define calibração medida em runtime (ou fixa)
void Controls_SetCalibration(const ControlsCalib *c);

// Lê 2 canais (PA0/PA1), faz média de 2 envios, zonas mortas e normalização
void Controls_ReadOnce(ControlsState *out);

#ifdef __cplusplus
}
#endif
