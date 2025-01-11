
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

extern CAN_HandleTypeDef hcan1;

void MX_CAN1_Init(void);
void CAN_ConfigFilters(void);
void CAN_Notifications_Init(void);
void CAN_Start();
void CAN_SendMessage(uint8_t *data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

