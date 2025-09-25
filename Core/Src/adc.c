#include "adc.h"
#include "stm32f4xx_hal.h"

/**
 * STM32F411CEU6 (Black Pill) ADC1 helper
 * - ADC1 channels physically bonded on UFQFPN48 package:
 *   IN0..IN7  -> PA0..PA7
 *   IN8       -> PB0
 *   IN9       -> PB1
 *   (IN10..IN15 on PC0..PC5 are NOT available on this package)
 */

ADC_HandleTypeDef hadc1;

/* === Public init (called from main.c) === */
void MX_ADC1_Init(void)
{
  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV8;     // Lower ADC clock to improve settling
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode          = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion   = 0;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }
}

/* === Helpers === */

/* Configure one regular channel (Rank 1) with long sampling time (good for high source impedance) */
static HAL_StatusTypeDef prv_config_channel(uint32_t ch)
{
  ADC_ChannelConfTypeDef s = {0};
  s.Channel      = ch;
  s.Rank         = 1;
  s.SamplingTime = ADC_SAMPLETIME_480CYCLES; // start conservative; you can reduce later
  return HAL_ADC_ConfigChannel(&hadc1, &s);
}

/* Read a single conversion from the given channel */
uint16_t ADC1_ReadRaw(uint32_t ch)
{
  if (prv_config_channel(ch) != HAL_OK) return 0;

  /* One dummy conversion improves accuracy when switching channels */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 5);
  (void)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) {
    HAL_ADC_Stop(&hadc1);
    return 0;
  }
  uint32_t v = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return (uint16_t)v;
}

/* Take N samples with optional simple outlier rejection and average them */
uint16_t ADC1_ReadAveraged(uint32_t ch, uint8_t samples)
{
  if (samples == 0) samples = 1;
  uint32_t acc = 0;
  uint16_t minv = 0xFFFF, maxv = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    uint16_t v = ADC1_ReadRaw(ch);
    if (v < minv) minv = v;
    if (v > maxv) maxv = v;
    acc += v;
  }
  if (samples >= 4) {
    acc -= minv; acc -= maxv;
    return (uint16_t)(acc / (samples - 2));
  }
  return (uint16_t)(acc / samples);
}
