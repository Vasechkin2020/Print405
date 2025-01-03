
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


    // Установите DEBUG для включения отладочной информации
#define DEBUG 1 // Поставьте 0 для отключения отладочной информации

#if DEBUG
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
    // #define DEBUG_PRINTF(fmt,...) printf("GREEN" fmt , ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(...) (void)0 // Приведение 0 к типу void, ничего не делает
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
