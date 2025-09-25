# README_SERIAL — Teste via USB/Serial com a Black Pill (STM32F411)

Este guia mostra, do zero, como **gerar dados na Black Pill** e **enviar JSON pela USB (CDC/ACM)** até o seu PC. Você vai conseguir:

1) **Testar sem a aplicação** (só PC ↔ Black Pill)  
2) **Testar com a aplicação** (bridge → MQTT/HTTP → API → Frontend)

O texto é para quem **não conhece o CubeIDE/CubeMX**. Siga na ordem.

---

## 0) Pré‑requisitos

- **Placa:** STM32F411 “Black Pill” (ex.: WeAct F411CEU6)  
- **Cabo micro‑USB de dados** (não pode ser só de carga!)  
- **Programador ST‑LINK** (ou Nucleo como ST‑Link)  
- **PC Linux (Ubuntu)** com:
  - STM32CubeIDE e/ou STM32CubeProgrammer instalados
  - `jq`, `screen` e `python3-serial` (opcional):
    ```bash
    sudo apt update
    sudo apt install -y jq screen python3-serial
    ```
- **Projeto Telemetria** (backend+frontend) clonado (opcional para a parte 2).

---

## 1) Criar o projeto no STM32CubeIDE/CubeMX

### 1.1. Novo projeto
- Abra o **STM32CubeIDE** → **File → New → STM32 Project**.  
- Target: **STM32F411CEUx** (ou selecione pelo board se existir).  
- Toolchain/IDE: **STM32CubeIDE** → Finish.

### 1.2. Ativar USB CDC (Virtual COM)
- Aba **Pinout & Configuration**:
  - Em **Connectivity → USB_OTG_FS**: selecione **Device_Only**.  
  - Em **Middleware → USB_DEVICE**: **Class = CDC**.  
  - **VBUS sensing**: **Disable** (tique a opção de desabilitar se aparecer).

### 1.3. Clock USB = 48 MHz (obrigatório)
O USB **só funciona** se o clock de USB for **48 MHz** exatos (via PLL).

- Aba **Clock Configuration**:
  - **Fonte: HSE (cristal externo)**  
    - WeAct (mais comum): **HSE = 25 MHz**  
    - Outras: às vezes **HSE = 8 MHz**
  - **Se HSE = 25 MHz:**
    - PLLM = **25**, PLLN = **336**, PLLP = **4**, **PLLQ = 7**  
    - SYSCLK = 84 MHz; USB = **48 MHz** (área “48 MHz clocks” deve ficar **verde**)
  - **Se HSE = 8 MHz:**
    - PLLM = **8**, PLLN = **336**, PLLP = **4**, **PLLQ = 7**  
    - SYSCLK = 84 MHz; USB = **48 MHz**

> Se a área de 48 MHz ficar vermelha, ajuste os valores conforme acima.

### 1.4. GPIOs (opcional — LED teste)
- Em **System Core → GPIO**: configure **PC13** como **Output Push‑Pull** (LED on‑board).

### 1.5. Gere o código
- **Project → Generate Code**.

---

## 2) Organização dos arquivos (compatível com o Cube)

Crie **estes arquivos** dentro de `Core/Inc` e `Core/Src` (nomes claros para reuso).  
> Obs.: este repositório tem uma pasta **Core** com exemplo real.

```
Core/
  Inc/
    telemetry_model.h
    comm_usb_cdc.h
    datagen_simple.h
  Src/
    comm_usb_cdc.c
    datagen_simple.c
    main.c
```

### 2.1. `telemetry_model.h`
```c
#ifndef TELEMETRY_MODEL_H
#define TELEMETRY_MODEL_H
#include <stdint.h>

typedef struct { float latitude, longitude; } GPSData;

typedef struct {
  int8_t accelerationX, accelerationY, accelerationZ;
  int8_t spinX, spinY, spinZ;
  int16_t scale_dps;                 // 250 | 500 | 1000 | 2000
} IMUData;

typedef struct { uint8_t pwm; float speed_est_mps; } DriveData;

typedef struct {
  uint16_t curve_direction;          // 0..360
  uint8_t  speed;                    // 0..255
  uint8_t  movement_direction;       // 1=frente, 0=ré
} Controls;

typedef struct {
  struct { GPSData gps; IMUData imu; DriveData drive; } car;
  struct { Controls controls; } centric;
  const char* src;                   // "central"
} TelemetryPacket;

#endif
```

### 2.2. `comm_usb_cdc.h`
```c
#pragma once
#include <stddef.h>
#include "telemetry_model.h"

#ifdef __cplusplus
extern "C" {
#endif

void CommUSB_Init(void);
int  CommUSB_SendLine(const char* s);
int  CommUSB_BuildJSON(const TelemetryPacket* p, char* out, size_t outlen);
int  CommUSB_SendPacketJSON(const TelemetryPacket* p);

#ifdef __cplusplus
}
#endif
```

### 2.3. `comm_usb_cdc.c`
```c
#include "comm_usb_cdc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

static int CDC_SendBlocking(const uint8_t *buf, uint16_t len, uint32_t timeout_ms) {
  uint32_t start = HAL_GetTick();
  while (CDC_Transmit_FS((uint8_t*)buf, len) == USBD_BUSY) {
    if ((HAL_GetTick() - start) > timeout_ms) return -1;
    HAL_Delay(1);
  }
  return 0;
}

static int append_fixed(char *out, size_t outlen, size_t *pos, float v, int decimals) {
  if (!out || !pos || *pos >= outlen) return -1;
  static const long factors[] = {1L,10L,100L,1000L,10000L,100000L,1000000L};
  if (decimals < 0) decimals = 0;
  if (decimals > 6) decimals = 6;
  long factor = factors[decimals];
  long sign = (v < 0.0f) ? -1 : 1;
  float av = v * sign;
  long scaled = (long)(av * factor + 0.5f);
  long ip = scaled / factor;
  long frac = scaled % factor;
  int n = 0;
  if (sign < 0) {
    if ((*pos + 1) >= outlen) return -1;
    out[(*pos)++] = '-';
  }
  n = snprintf(out + *pos, outlen - *pos, "%ld", ip);
  if (n < 0 || (size_t)n >= (outlen - *pos)) return -1;
  *pos += (size_t)n;
  if (decimals > 0) {
    if ((*pos + 1) >= outlen) return -1;
    out[(*pos)++] = '.';
    long pad = factor / 10;
    while (pad > 0 && frac < pad) {
      if (*pos >= outlen) return -1;
      out[(*pos)++] = '0';
      pad /= 10;
    }
    n = snprintf(out + *pos, outlen - *pos, "%ld", frac);
    if (n < 0 || (size_t)n >= (outlen - *pos)) return -1;
    *pos += (size_t)n;
  }
  return 0;
}

void CommUSB_Init(void) {
  MX_USB_DEVICE_Init();
  const char *hello = "{\"hello\":\"stm32f411-central\",\"src\":\"central\"}\n";
  CDC_SendBlocking((const uint8_t*)hello, (uint16_t)strlen(hello), 50);
}

int CommUSB_SendLine(const char* s) {
  char buf[600];
  size_t n = strlen(s);
  if (n >= sizeof(buf) - 2) n = sizeof(buf) - 2;
  memcpy(buf, s, n);
  if (n == 0 || buf[n-1] != '\n') buf[n++] = '\n';
  return CDC_SendBlocking((const uint8_t*)buf, (uint16_t)n, 50);
}

int CommUSB_BuildJSON(const TelemetryPacket* p, char* out, size_t outlen) {
  if (!p || !out || outlen < 64) return -1;
  size_t pos = 0;
  int n;

  n = snprintf(out + pos, outlen - pos,
    "{"
      "\"car\":{"
        "\"gps\":{\"latitude\":");
  if (n < 0 || (size_t)n >= (outlen - pos)) return -1; pos += (size_t)n;

  if (append_fixed(out, outlen, &pos, p->car.gps.latitude, 6) < 0) return -1;

  n = snprintf(out + pos, outlen - pos, ",\"longitude\":");
  if (n < 0 || (size_t)n >= (outlen - pos)) return -1; pos += (size_t)n;
  if (append_fixed(out, outlen, &pos, p->car.gps.longitude, 6) < 0) return -1;

  n = snprintf(out + pos, outlen - pos,
        "},"
        "\"imu\":{"
          "\"accelerationX\":%d,\"accelerationY\":%d,\"accelerationZ\":%d,"
          "\"spinX\":%d,\"spinY\":%d,\"spinZ\":%d,"
          "\"scale_dps\":%d"
        "},"
        "\"drive\":{\"pwm\":%u,\"speed_est_mps\":",
        (int)p->car.imu.accelerationX, (int)p->car.imu.accelerationY, (int)p->car.imu.accelerationZ,
        (int)p->car.imu.spinX, (int)p->car.imu.spinY, (int)p->car.imu.spinZ,
        (int)p->car.imu.scale_dps,
        (unsigned)p->car.drive.pwm
  );
  if (n < 0 || (size_t)n >= (outlen - pos)) return -1; pos += (size_t)n;

  if (append_fixed(out, outlen, &pos, p->car.drive.speed_est_mps, 3) < 0) return -1;

  n = snprintf(out + pos, outlen - pos,
        "}"
      "},"
      "\"centric\":{"
        "\"controls\":{"
          "\"curve_direction\":%u,"
          "\"speed\":%u,"
          "\"movement_direction\":%u"
        "}"
      "},"
      "\"src\":\"%s\""
    "}",
    (unsigned)p->centric.controls.curve_direction,
    (unsigned)p->centric.controls.speed,
    (unsigned)p->centric.controls.movement_direction,
    (p->src ? p->src : "central")
  );
  if (n < 0 || (size_t)n >= (outlen - pos)) return -1; pos += (size_t)n;

  out[pos] = '\0';
  return (int)pos;
}

int CommUSB_SendPacketJSON(const TelemetryPacket* p) {
  char line[600];
  int n = CommUSB_BuildJSON(p, line, sizeof(line));
  if (n < 0) return -1;
  line[(size_t)n < sizeof(line)-1 ? n : (int)sizeof(line)-1] = '\0';
  return CommUSB_SendLine(line);
}
```

### 2.4. `datagen_simple.h`
```c
#pragma once
#include <stdint.h>
#include "telemetry_model.h"

#ifdef __cplusplus
extern "C" {
#endif

void DataGenSimple_Init(void);
void DataGenSimple_Step(TelemetryPacket* p, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
```

### 2.5. `datagen_simple.c`
```c
#include "datagen_simple.h"
#include <math.h>

#define VMAX_MPS  12.0f
#define LAT0      (-23.5586f)
#define LON0      (-46.6492f)

static uint32_t t0_ms;
static int8_t clamp_i8(int v){ if(v<-128)return-128; if(v>127)return 127; return (int8_t)v; }

void DataGenSimple_Init(void){ t0_ms = 0; }

void DataGenSimple_Step(TelemetryPacket* p, uint32_t now_ms) {
  if (!p) return;
  if (t0_ms == 0) t0_ms = now_ms;
  float t = (now_ms - t0_ms) / 1000.0f;

  p->car.gps.latitude  = LAT0 + 0.00015f * sinf(t / 20.0f);
  p->car.gps.longitude = LON0 + 0.00015f * cosf(t / 20.0f);

  p->car.imu.accelerationX = clamp_i8((int)( 40.0f * sinf(t / 3.0f)));
  p->car.imu.accelerationY = clamp_i8((int)( 40.0f * cosf(t / 5.0f)));
  p->car.imu.accelerationZ = clamp_i8((int)( 10.0f * sinf(t / 7.0f)));
  p->car.imu.spinX         = clamp_i8((int)( 50.0f * sinf(t / 4.0f)));
  p->car.imu.spinY         = clamp_i8((int)( 50.0f * cosf(t / 6.0f)));
  p->car.imu.spinZ         = clamp_i8((int)( 70.0f * sinf(t / 2.0f)));
  static const int16_t ring[4] = {250,500,1000,2000};
  p->car.imu.scale_dps = ring[((int)(t/20.0f)) & 3];

  float duty = (sinf(t / 5.0f) + 1.0f) * 0.5f;
  if (duty < 0) duty = 0; if (duty > 1) duty = 1;
  p->car.drive.pwm = (uint8_t)(duty * 255.0f + 0.5f);
  p->car.drive.speed_est_mps = VMAX_MPS * duty;

  float steering_deg = 90.0f * sinf(t / 4.0f);
  int curve = (int)fmodf(steering_deg + 360.0f, 360.0f);
  if (curve < 0) curve += 360;
  p->centric.controls.curve_direction = (uint16_t)curve;
  p->centric.controls.speed = p->car.drive.pwm;
  p->centric.controls.movement_direction = (sinf(t / 15.0f) > -0.2f) ? 1 : 0;

  p->src = "central";
}
```

### 2.6. `main.c` (exemplo mínimo)
O `main.c` deve seguir o **formato padrão do CubeIDE** (seções `USER CODE`). Exemplo mínimo:
```c
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
#include "telemetry_model.h"
#include "comm_usb_cdc.h"
#include "datagen_simple.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum {
  MODE_SIM_ALL = 0,   // Simula tudo (carro + central)
  MODE_SIM_CAR,       // Simula carro, lê central (pot/btn)
  MODE_REAL_ALL       // Tudo real (simulador OFF)
} AppMode;
/* Fácil de mudar aqui: */
static volatile AppMode g_mode = MODE_SIM_ALL;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t adc_read_once(uint32_t channel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  CommUSB_Init();
  DataGenSimple_Init();

  TelemetryPacket pkt;
  uint32_t last = HAL_GetTick();
  const uint32_t period_ms = 100;

  while (1) {
    uint32_t now = HAL_GetTick();
    if ((now - last) >= period_ms) {
      last = now;

      /* Base: simulador gera um pacote */
      DataGenSimple_Step(&pkt, now);

      /* Ajustes por modo */
      if (g_mode == MODE_SIM_CAR || g_mode == MODE_REAL_ALL) {
        /* Lê potenciômetros/botão via ADC/GPIO e injeta no pkt */
        uint16_t raw_pot_speed  = adc_read_once(ADC_CHANNEL_1); // PA1
        uint16_t raw_pot_curve  = adc_read_once(ADC_CHANNEL_0); // PA0
        uint8_t  btn_state = (HAL_GPIO_ReadPin(Botao_GPIO_Port, Botao_Pin) == GPIO_PIN_RESET) ? 1 : 0;

        pkt.centric.controls.speed = (uint8_t)((raw_pot_speed * 255U) / 4095U);
        pkt.centric.controls.curve_direction = (uint16_t)((raw_pot_curve * 360U) / 4095U);
        pkt.centric.controls.movement_direction = btn_state ? 1 : 0;
      }

      /* Envia JSON por USB CDC */
      CommUSB_SendPacketJSON(&pkt);
    }
  }
  /* USER CODE END 2 */
}

/* Clock, GPIO e ADC seguem o gerado pelo CubeIDE. Apenas garanta:
   - ADC1.ScanConvMode = ENABLE
   - NbrOfConversion >= 2
   - Canais rankeados: CH1 (rank 1) e CH0 (rank 2) ou vice-versa
   - SampleTime alto (ex.: 480 cycles) para reduzir ruído de fonte */
```

> **Dica anti‑ruído:** deixe `SamplingTime = 480 cycles`, configure os pinos como **Analog** (sem pull), e teste com fonte estável. Se o pot oscilar muito em 3.3V, verifique **fio/ground**, **filtro RC** (ex.: 1–10 kΩ + 100 nF), e **VREF+**.

---

## 3) Gravar o firmware

Use o **procedimento padrão** (ambos funcionam):
- **STM32CubeIDE**: **Debug/Run** via **ST‑LINK (ST‑LINK GDB Server)** ou OpenOCD.  
- **STM32CubeProgrammer**:
  - Conecte via **ST‑LINK/SWD** → **Connect**  
  - **Open file** (`.elf` ou `.bin` em `Debug/`)  
  - **Download** (se `.bin`, endereço `0x08000000`)  
  - **Verify** → **Reset**

---

## 4) Testar **sem** a aplicação (apenas PC ↔ Black Pill)

1) Plugue o **micro‑USB da Black Pill** (porta do MCU)  
2) Verifique o device:
   ```bash
   ls -l /dev/ttyACM*
   ```
   Deve aparecer `/dev/ttyACM0`.

3) Dê acesso (uma vez):
   ```bash
   sudo usermod -aG dialout $USER && newgrp dialout
   sudo systemctl stop ModemManager brltty 2>/dev/null || true
   ```

4) Leia os dados (NDJSON):
   ```bash
   stty -F /dev/ttyACM0 115200 -echo -icanon -ixoff -ixon -crtscts
   stdbuf -oL cat /dev/ttyACM0 | jq -c .
   ```
   Ou com `screen`:
   ```bash
   screen /dev/ttyACM0 115200
   ```
   Para sair do screen:
   ```bash
   # kill
   CTRL + A, depois K
   # detach
   CTRL + A, depois D
   ```

Se você vê **um JSON por linha**, a placa está OK.

---

## 5) Testar **com** a aplicação (bridge → backend → frontend)

### 5.1. `.env` (exemplo)
```dotenv
# ===== User =====
UID=1000
GID=1000

# ===== Select profiles =====
# sim    -> MQTT simulator (no board)
# serial -> Real board via USB CDC (serial bridge)
# http   -> Direct HTTP feeder to API (no MQTT)
COMPOSE_PROFILES=serial

# ===== Ports =====
MOSQUITTO_PORT=1883
API_PORT=8000
FRONT_PORT=5173

# ===== Backend API =====
UVICORN_HOST=0.0.0.0
UVICORN_PORT=8000
SQLITE_PATH=/data/telemetry.db
API_TZ=America/Sao_Paulo
CORS_ORIGINS=["http://localhost:5173"]
VMAX_MPS=12

# ===== MQTT / Broker =====
MQTT_URL=mqtt://mosquitto:1883
MQTT_TOPIC=telemetry/combined/1
MQTT_USERNAME=
MQTT_PASSWORD=

# ===== SIMULATOR (perfil: sim) =====
SIM_INTERVAL_MS=200
SIM_SRC=sim

# ===== HTTP FEEDER (perfil: http) =====
API_INGEST_URL=http://api:8000/api/v1/telemetry/ingest
FEEDER_INTERVAL_MS=200

# ===== SERIAL BRIDGE (perfil: serial) =====
SERIAL_PORT=/dev/ttyACM0
SERIAL_BAUD=115200
BRIDGE_MODE=mqtt   # mqtt | http

# ===== Frontend (Vite) ===== 
FRONT_PORT=5173
VITE_API_HTTP_URL=http://localhost:8000
VITE_API_WS_URL=ws://localhost:8000/ws
```

### 5.2. Subir serviços
```bash
docker compose down -v
docker compose up -d --build
```

### 5.3. Verificar o bridge
```bash
docker logs -f serial_bridge
```

### 5.4. Se BRIDGE_MODE=mqtt: ver tópico
```bash
docker exec -it mosquitto sh -c 'mosquitto_sub -t "telemetry/combined/1" -v'
```

### 5.5. API (bruto e processado)
```bash
curl -s http://localhost:8000/api/v1/telemetry_raw/latest | jq
curl -s http://localhost:8000/api/v1/telemetry/latest | jq
```

### 5.6. Frontend
Abra: `http://localhost:5173`

---

## 6) Troubleshooting rápido

- **/dev/ttyACM0 não aparece:**  
  Cabo só de carga? Porta errada? Clock USB ≠ 48 MHz? Middleware CDC desativado?
- **Permission denied no container:**  
  - Host: `sudo usermod -aG dialout $USER && newgrp dialout`  
  - Regra udev (persistente):
    ```bash
    echo 'KERNEL=="ttyACM[0-9]*", MODE:="0666"' | sudo tee /etc/udev/rules.d/99-ttyacm.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```
  - Compose: `devices: - "/dev/ttyACM0:/dev/ttyACM0"` e `group_add: - dialout`

- **Leituras de ADC instáveis (potenciômetros):**  
  - Configure pinos como **Analog** (sem pull‑ups/pull‑downs).  
  - **Sample time alto** (ex.: 480 cycles).  
  - Use **terra comum** e cabos curtos.  
  - Se preciso, **filtro RC** (1–10 kΩ + 100 nF) no cursor do pot.  
  - Evite fonte ruidosa do 3.3 V; se necessário, regulador melhor/LC.  