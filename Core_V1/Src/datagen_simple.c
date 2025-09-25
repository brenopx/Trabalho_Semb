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
