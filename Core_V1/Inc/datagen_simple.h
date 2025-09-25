#ifndef DATAGEN_SIMPLE_H
#define DATAGEN_SIMPLE_H
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
#endif
