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
