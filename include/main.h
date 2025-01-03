
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

#define Analiz_Pin GPIO_PIN_3
#define Analiz_GPIO_Port GPIOA
#define Led1_Pin GPIO_PIN_1
#define Led1_GPIO_Port GPIOB
#define Pwm_Pin GPIO_PIN_4
#define Pwm_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
