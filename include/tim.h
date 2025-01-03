
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

void MX_TIM6_Init(void);
void MX_TIM7_Init(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); //Обработчик прерываний для таймеров

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

