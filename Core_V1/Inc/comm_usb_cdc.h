#ifndef COMM_USB_CDC_H
#define COMM_USB_CDC_H
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
#endif
