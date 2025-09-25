#include "datagen_simple.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <stdint.h>
#include "main.h"          // puxa stm32f4xx_hal.h e cia
#include "usbd_cdc_if.h"   // CDC_Transmit_FS
#include "stm32f4xx_hal_adc.h"  // garante defs do ADC (por via das dúvidas)
#include "stm32f4xx_hal.h"
extern ADC_HandleTypeDef hadc1;

// -------------------------
// Estado interno do simulador
// -------------------------
typedef struct {
  uint32_t mode_mask;
  char who[32];

  // Estado do "carro"
  float car_speed_kmh;
  float car_batt_v;
  float car_temp_c;
  float car_rpm;

  // Estado dos "controles"
  float ctrl_angle_deg;       // 0..360
  float ctrl_speed_pct;       // 0..100
  uint8_t ctrl_move_dir;      // 0=frente, 1=ré

  // Tempo
  uint32_t last_ms;
} SimState;

static SimState g;

// -------------------------
// Utils de append seguro
// -------------------------
static int append_snprintf(char *out, size_t cap, size_t *pos, const char *fmt, ...){
  if (*pos >= cap) return -1;
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(out + *pos, cap - *pos, fmt, ap);
  va_end(ap);
  if (n < 0) return -1;
  if ((size_t)n >= (cap - *pos)) return -1;
  *pos += (size_t)n;
  return n;
}

// -------------------------
// Inicialização
// -------------------------
void DataGenSimple_Init(uint32_t mode_mask, const char *who_tag){
  memset(&g, 0, sizeof(g));
  g.mode_mask = mode_mask;
  if (who_tag && who_tag[0]) {
    snprintf(g.who, sizeof(g.who), "%s", who_tag);
  } else {
    snprintf(g.who, sizeof(g.who), "central");
  }

  // Estados iniciais "ok"
  g.car_speed_kmh = 0.0f;
  g.car_batt_v    = 12.3f;
  g.car_temp_c    = 35.0f;
  g.car_rpm       = 900.0f;

  g.ctrl_angle_deg  = 180.0f;  // centro
  g.ctrl_speed_pct  = 0.0f;
  g.ctrl_move_dir   = 0;       // frente

  g.last_ms = 0;
}

// -------------------------
// Avanço do simulador
// -------------------------
static void sim_tick(uint32_t now_ms){
  // Δt em segundos
  float dt = (g.last_ms == 0) ? 0.0f : (now_ms - g.last_ms) / 1000.0f;
  g.last_ms = now_ms;

  // ------- CARRO -------
  if (g.mode_mask & SIM_MODE_CAR){
    // velocidade oscila 0..60 km/h com senóide lenta
    float w = 2.0f * 3.14159265f / 12.0f; // período ~12 s
    float v = (sinf(w * (now_ms / 1000.0f)) * 0.5f + 0.5f) * 60.0f;
    // suaviza
    g.car_speed_kmh += (v - g.car_speed_kmh) * fminf(1.0f, dt * 0.8f);

    // rpm proporcional à velocidade (simples)
    float target_rpm = 800.0f + g.car_speed_kmh * 50.0f;
    g.car_rpm += (target_rpm - g.car_rpm) * fminf(1.0f, dt * 1.2f);

    // bateria oscila levemente
    float batt = 12.2f + 0.2f * sinf(w * 0.5f * (now_ms / 1000.0f));
    g.car_batt_v += (batt - g.car_batt_v) * fminf(1.0f, dt * 0.5f);

    // temperatura sobe devagar e cai devagar
    float temp_target = 30.0f + (g.car_speed_kmh * 0.15f);
    g.car_temp_c += (temp_target - g.car_temp_c) * fminf(1.0f, dt * 0.2f);
  }

  // ------- CONTROLES -------
  if (g.mode_mask & SIM_MODE_CONTROLS){
    // Ângulo gira de forma contínua 0..360 (uma volta ~8 s)
    float angle_rate_dps = 45.0f; // 45 deg/s
    g.ctrl_angle_deg += angle_rate_dps * dt;
    while (g.ctrl_angle_deg >= 360.0f) g.ctrl_angle_deg -= 360.0f;

    // Velocidade de comando oscila 0..100% (período ~10 s)
    float w2 = 2.0f * 3.14159265f / 10.0f;
    float sp = (sinf(w2 * (now_ms / 1000.0f)) * 0.5f + 0.5f) * 100.0f;
    g.ctrl_speed_pct += (sp - g.ctrl_speed_pct) * fminf(1.0f, dt * 1.0f);

    // Direção frente/ré alterna a cada 5 s
    uint32_t phase = (now_ms / 5000u) % 2u;
    g.ctrl_move_dir = (phase == 0) ? 0u : 1u;
  }
}

// -------------------------
// Builders de JSON
// -------------------------
static int build_controls_json(char *out, size_t cap, size_t *pos){
  unsigned angle = (unsigned)(g.ctrl_angle_deg + 0.5f);  // arredonda
  if (angle >= 360u) angle = 359u;
  unsigned spd   = (unsigned)(g.ctrl_speed_pct + 0.5f);  // 0..100
  if (spd > 100u) spd = 100u;
  unsigned dir   = (unsigned)g.ctrl_move_dir;

  return append_snprintf(out, cap, pos,
    "\"controls\":{"
      "\"curve_direction\":%u,"
      "\"speed\":%u,"
      "\"movement_direction\":%u"
    "}",
    angle, spd, dir
  );
}

static int build_car_json(char *out, size_t cap, size_t *pos){
  // Valores inteiros razoáveis
  int speed10 = (int)(g.car_speed_kmh * 10.0f + 0.5f); // uma casa decimal
  int batt_mv = (int)(g.car_batt_v * 1000.0f + 0.5f);  // em mV
  int temp10  = (int)(g.car_temp_c * 10.0f + 0.5f);    // uma casa decimal
  int rpm     = (int)(g.car_rpm + 0.5f);

  return append_snprintf(out, cap, pos,
    "\"car\":{"
      "\"speed_kmh_x10\":%d,"   // ex.: 253 => 25.3 km/h
      "\"battery_mv\":%d,"      // ex.: 12340 => 12.34 V
      "\"temp_c_x10\":%d,"      // ex.: 356 => 35.6 °C
      "\"rpm\":%d"
    "}",
    speed10, batt_mv, temp10, rpm
  );
}

// JSON completo
size_t DataGenSimple_BuildFullJSON(char *out, size_t cap, uint32_t now_ms,
                                   const char *controls_override_json)
{
  if (!out || cap < 16) return 0;

  sim_tick(now_ms);

  size_t pos = 0;
  if (append_snprintf(out, cap, &pos, "{") < 0) return 0;

  // who + timestamp
  if (append_snprintf(out, cap, &pos,
        "\"who\":\"%s\",\"ts\":%lu,",
        g.who, (unsigned long)now_ms) < 0) return 0;

  // car (se habilitado)
  int wrote_field = 0;
  if (g.mode_mask & SIM_MODE_CAR){
    if (build_car_json(out, cap, &pos) < 0) return 0;
    wrote_field = 1;
  }

  // separador, se já escreveu algo
  if (wrote_field && ((g.mode_mask & SIM_MODE_CONTROLS) || controls_override_json)) {
    if (append_snprintf(out, cap, &pos, ",") < 0) return 0;
  }

  // controls: usa override se fornecido; caso contrário, gera se SIM_MODE_CONTROLS
  if (controls_override_json && controls_override_json[0]){
    if (append_snprintf(out, cap, &pos, "%s", controls_override_json) < 0) return 0;
    wrote_field = 1;
  } else if (g.mode_mask & SIM_MODE_CONTROLS){
    if (build_controls_json(out, cap, &pos) < 0) return 0;
    wrote_field = 1;
  }

  // modo (só informativo)
  if (wrote_field) {
    if (append_snprintf(out, cap, &pos, ",") < 0) return 0;
  }
  if (append_snprintf(out, cap, &pos, "\"mode\":\"sim\"") < 0) return 0;

  if (append_snprintf(out, cap, &pos, "}\r\n") < 0) return 0;

  return pos;
}

// --- PROBE: alerta quando qualquer canal passar do limiar ---
static void Debug_ProbeAnalogPins(void){
  const struct { uint32_t ch; const char *name; } list[] = {
    {ADC_CHANNEL_0,  "IN0_PA0"}, {ADC_CHANNEL_1,  "IN1_PA1"},
    {ADC_CHANNEL_4,  "IN4_PA4"}, {ADC_CHANNEL_5,  "IN5_PA5"},
    {ADC_CHANNEL_6,  "IN6_PA6"}, {ADC_CHANNEL_7,  "IN7_PA7"},
    {ADC_CHANNEL_8,  "IN8_PB0"}, {ADC_CHANNEL_10, "IN10_PC0"},
    {ADC_CHANNEL_11, "IN11_PC1"},{ADC_CHANNEL_12, "IN12_PC2"},
    {ADC_CHANNEL_13, "IN13_PC3"},{ADC_CHANNEL_14, "IN14_PC4"},
    {ADC_CHANNEL_15, "IN15_PC5"},
  };
  ADC_ChannelConfTypeDef s = {0};
  s.Rank = 1; s.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  for (size_t i=0;i<sizeof(list)/sizeof(list[0]);++i){
    s.Channel = list[i].ch;
    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) continue;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK){
      uint32_t v = HAL_ADC_GetValue(&hadc1);
      if (v >= 3800 || v <= 50){
        char buf[96];
        int n = snprintf(buf, sizeof(buf),
          "{\"probe_hit\":{\"ch\":\"%s\",\"raw\":%lu}}\r\n",
          list[i].name, (unsigned long)v);
        CDC_Transmit_FS((uint8_t*)buf, (uint16_t)n);
      }
    }
    HAL_ADC_Stop(&hadc1);
  }
}

