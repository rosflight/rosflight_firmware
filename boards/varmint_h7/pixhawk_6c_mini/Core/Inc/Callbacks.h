#ifndef PIXHAWK_6C_MINI_CALLBACKS_H
#define PIXHAWK_6C_MINI_CALLBACKS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void CDC_Receive_Callback(uint8_t chan, uint8_t * buffer, uint16_t size);
void CDC_TransmitCplt_Callback(uint8_t chan, uint8_t * buffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
