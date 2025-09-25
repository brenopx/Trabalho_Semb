/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "usbd_cdc_if.h"
#include "adc.h"                 /* MX_ADC1_Init() gerado pela IDE */
#include "datagen_simple.h"      /* seu simulador */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Seleção de modo — MUDE AQUI com facilidade
   APP_MODE_SIM_ALL          -> simula carro + controles
   APP_MODE_SIM_CAR_REAL_CTRL-> simula carro e usa controles reais (potenciômetros/botão)
   APP_MODE_REAL_ALL         -> tudo real (sem simulação)                                    */
typedef enum {
  APP_MODE_SIM_ALL = 0,
  APP_MODE_SIM_CAR_REAL_CTRL = 1,
  APP_MODE_REAL_ALL = 2
} AppMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -------- PERFIL “estilo COMPOSE_PROFILES” (só informativo) -------- */
#define APP_PROFILE_STR "serial,sim-car"  /* opções: "serial,sim-all" | "serial,sim-car" | "serial,real-all" */

/* -------- MODO ATIVO (MUDE AQUI) -------- */
#define APP_MODE APP_MODE_SIM_CAR_REAL_CTRL

/* Periodicidade de envio por USB CDC */
#define TX_PERIOD_MS   200U  /* 5 Hz */

/* Canais ADC para os potenciômetros (ajuste se necessário) */
#define ADC_CH_POT_ANGLE   ADC_CHANNEL_0   /* PA0 */
#define ADC_CH_POT_SPEED   ADC_CHANNEL_1   /* PA1 */

/* Tamanho do payload JSON */
#define JSON_CAP           512U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* hadc1 é definido em Core/Src/adc.c (gerado pela IDE).
   Se adc.h não declarar extern, este extern garante a visibilidade sem duplicar definição. */
extern ADC_HandleTypeDef hadc1;

/* Buffer de envio */
static char payload[JSON_CAP];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
/* Nota: MX_ADC1_Init() vem de adc.c gerado pela IDE */

/* Utilitários */
static uint16_t adc_read_channel(uint32_t ch);
static void     build_controls_override(char *out, size_t cap, uint16_t pot_angle, uint16_t pot_speed, GPIO_PinState btn);
static int      cdc_send_blocking(const uint8_t *buf, uint16_t len, uint32_t timeout_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t adc_read_channel(uint32_t ch)
{
  ADC_ChannelConfTypeDef s = (ADC_ChannelConfTypeDef){0};
  s.Channel = ch;
/* Em F4, Rank é numérico (1..16); em outros HALs existe ADC_REGULAR_RANK_1 */
#ifdef ADC_REGULAR_RANK_1
  s.Rank = ADC_REGULAR_RANK_1;
#else
  s.Rank = 1U;
#endif
  s.SamplingTime = ADC_SAMPLETIME_84CYCLES;

  if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) {
    return 0;
  }
  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return 0;
  }
  if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) {
    HAL_ADC_Stop(&hadc1);
    return 0;
  }
  uint32_t v = HAL_ADC_GetValue(&hadc1) & 0x0FFFU;
  HAL_ADC_Stop(&hadc1);
  return (uint16_t)v;
}

static void build_controls_override(char *out, size_t cap,
                                    uint16_t pot_angle, uint16_t pot_speed,
                                    GPIO_PinState btn)
{
  /* Mapas simples: 12 bits -> 0..359 e 0..100 */
  unsigned angle = (unsigned)((pot_angle * 360U) / 4096U);
  if (angle >= 360U) angle = 359U;
  unsigned speed = (unsigned)((pot_speed * 100U) / 4095U);
  if (speed > 100U) speed = 100U;

  /* Botão com pull-up: pressionado = 0 (ré), solto = 1 (frente) -> ajuste se desejar */
  unsigned movement_dir = (btn == GPIO_PIN_RESET) ? 1U : 0U;

  /* Gera apenas o bloco "controls":{...} para injetar no simulador */
  (void)snprintf(out, cap,
                 "\"controls\":{\"curve_direction\":%u,\"speed\":%u,\"movement_direction\":%u}",
                 angle, speed, movement_dir);
}

static int cdc_send_blocking(const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < timeout_ms) {
    if (CDC_Transmit_FS((uint8_t*)buf, len) == (uint8_t)USBD_OK) return 0;
    /* USB-CDC busy: aguarda um pouco e tenta de novo */
    HAL_Delay(2);
  }
  return -1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();   /* vem do adc.c gerado pela IDE */

  /* USER CODE BEGIN 2 */
  /* Inicializa o simulador conforme o modo desejado */
  uint32_t sim_mask = 0;
  if (APP_MODE == APP_MODE_SIM_ALL) {
    sim_mask = SIM_MODE_CAR | SIM_MODE_CONTROLS;
  } else if (APP_MODE == APP_MODE_SIM_CAR_REAL_CTRL) {
    sim_mask = SIM_MODE_CAR; /* controles virão do ADC (override) */
  } else { /* APP_MODE_REAL_ALL */
    sim_mask = 0; /* sem simulação; apenas override de controls */
  }
  DataGenSimple_Init(sim_mask, "central");

  uint32_t last_tx = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    if ((now - last_tx) >= TX_PERIOD_MS) {
      last_tx = now;

      /* Leituras reais (mesmo nos modos com sim do carro, os controles podem vir daqui) */
      uint16_t pot_angle = adc_read_channel(ADC_CH_POT_ANGLE);
      uint16_t pot_speed = adc_read_channel(ADC_CH_POT_SPEED);
      GPIO_PinState btn   = HAL_GPIO_ReadPin(Botao_GPIO_Port, Botao_Pin);

      char controls_override[96] = {0};

      const char *override_ptr = NULL;
      if (APP_MODE == APP_MODE_SIM_ALL) {
        /* Nada a sobrescrever: simula também os controles */
        override_ptr = NULL;
      } else {
        /* SIM_CAR_REAL_CTRL ou REAL_ALL — controles reais */
        build_controls_override(controls_override, sizeof(controls_override),
                                pot_angle, pot_speed, btn);
        override_ptr = controls_override;
      }

      /* Monta JSON final (sim + override opcional) */
      size_t n = DataGenSimple_BuildFullJSON(payload, sizeof(payload), now, override_ptr);
      if (n > 0 && n < sizeof(payload)) {
        (void)cdc_send_blocking((uint8_t*)payload, (uint16_t)n, 50);
      }
    }

    /* Pequeno atraso para aliviar CPU/USB */
    HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Esta função é a padrão gerada pela IDE. Mantenha a sua versão se já existe.
     Abaixo vai um esqueleto seguro; ajuste conforme seu projeto. */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* Gerado pela IDE normalmente — aqui fica só o relógio e o botão */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Botão com pull-up (ajuste Botao_Pin/Botao_GPIO_Port no .ioc) */
  GPIO_InitStruct.Pin  = Botao_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Botao_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) { /* trap */ }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file; (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
