
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

void MX_TIM3_Init(void);
void MX_TIM6_Init(void);
void MX_TIM7_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

extern void timer6();                                                             // Обработчик прерывания таймера TIM6	1 раз в 1 милисекунду
extern void timer7();                                                             // Обработчик прерывания таймера TIM7	1 раз в 1 милисекунду

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); //Обработчик прерываний для таймеров

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

