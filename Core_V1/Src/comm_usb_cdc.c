#include "comm_usb_cdc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* --------------------------------------------------------------------------
   Transmissão bloqueante simples via USB CDC
   -------------------------------------------------------------------------- */
static int CDC_SendBlocking(const uint8_t *buf, uint16_t len, uint32_t timeout_ms) {
  uint32_t start = HAL_GetTick();
  while (CDC_Transmit_FS((uint8_t*)buf, len) == USBD_BUSY) {
    if ((HAL_GetTick() - start) > timeout_ms) return -1;
    HAL_Delay(1);
  }
  return 0;
}

/* --------------------------------------------------------------------------
   Helper: escreve float com N casas decimais SEM usar %f
   Ex.: decimals=6 -> 12.345678 ; decimals=3 -> 12.346
   Garante zero-pad na parte fracionária e arredondamento correto.
   Retorna 0 em sucesso; -1 se faltar espaço no buffer.
   -------------------------------------------------------------------------- */
static int append_fixed(char *out, size_t outlen, size_t *pos, float v, int decimals) {
  if (!out || !pos || *pos >= outlen) return -1;

  static const long factors[] = {1L,10L,100L,1000L,10000L,100000L,1000000L};
  if (decimals < 0) decimals = 0;
  if (decimals > 6) decimals = 6;
  long factor = factors[decimals];

  long sign = (v < 0.0f) ? -1 : 1;
  float av = v * sign;
  long scaled = (long)(av * factor + 0.5f);  // round half up
  long ip = scaled / factor;                  // parte inteira
  long frac = scaled % factor;                // parte fracionária

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

/* -------------------------------------------------------------------------- */

void CommUSB_Init(void) {
  MX_USB_DEVICE_Init();
  // Mensagem opcional de hello
  const char *hello = "{\"hello\":\"stm32f411-central\",\"src\":\"central\"}\n";
  CDC_SendBlocking((const uint8_t*)hello, (uint16_t)strlen(hello), 50);
}

/* garante \n no final e envia */
int CommUSB_SendLine(const char* s) {
  char buf[600];
  size_t n = strlen(s);
  if (n >= sizeof(buf) - 2) n = sizeof(buf) - 2;
  memcpy(buf, s, n);
  if (n == 0 || buf[n-1] != '\n') buf[n++] = '\n';
  return CDC_SendBlocking((const uint8_t*)buf, (uint16_t)n, 50);
}

/* Monta JSON no buffer (SEM usar %f) */
int CommUSB_BuildJSON(const TelemetryPacket* p, char* out, size_t outlen) {
  if (!p || !out || outlen < 64) return -1;
  size_t pos = 0;
  int n;

  // início + GPS.lat
  n = snprintf(out + pos, outlen - pos,
    "{"
      "\"car\":{"
        "\"gps\":{\"latitude\":");
  if (n < 0 || (size_t)n >= (outlen - pos)) return -1; pos += (size_t)n;

  // latitude (6 casas)
  if (append_fixed(out, outlen, &pos, p->car.gps.latitude, 6) < 0) return -1;

  // longitude (6 casas)
  n = snprintf(out + pos, outlen - pos, ",\"longitude\":");
  if (n < 0 || (size_t)n >= (outlen - pos)) return -1; pos += (size_t)n;
  if (append_fixed(out, outlen, &pos, p->car.gps.longitude, 6) < 0) return -1;

  // IMU (inteiros) + drive.pwm
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

  // drive.speed_est_mps (3 casas)
  if (append_fixed(out, outlen, &pos, p->car.drive.speed_est_mps, 3) < 0) return -1;

  // centric + src
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

/* Atalho: monta e envia uma linha NDJSON */
int CommUSB_SendPacketJSON(const TelemetryPacket* p) {
  char line[600];
  int n = CommUSB_BuildJSON(p, line, sizeof(line));
  if (n < 0) return -1;
  // garante newline e envia
  line[(size_t)n < sizeof(line)-1 ? n : (int)sizeof(line)-1] = '\0';
  return CommUSB_SendLine(line);
}
