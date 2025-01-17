// ver 1.1

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//----
#include "main.h"
#include "dma.h"
#include "spi.h"

#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "can.h"

#include "code.h"
#include "slaveSPI.h"
#include "gim43xx.h"

void SystemClock_Config(void);
volatile uint32_t millisCounter = 0;

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();

  MX_DMA_Init();

  MX_SPI1_Init();

  MX_USART1_UART_Init();

  // MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  HAL_TIM_Base_Start_IT(&htim6); // Таймер для общего цикла
  HAL_TIM_Base_Start_IT(&htim7); // Таймер для моторов шаговых для датчиков

  // MX_CAN1_Init();
  // CAN_ConfigFilters();      // Настройка фильтров
  // CAN_Notifications_Init(); // Активация прерываний
  // CAN_Start();              // Запуск CAN

  // initFirmware();
  printf("\r\n *** printBIM.ru 2025. ***\r\n");
  // printf("Firmware gen %hu ver %hu laser %hu motor %.1f debug %hu\n", Print2Data_send.firmware.gen, Print2Data_send.firmware.ver,Print2Data_send.firmware.laser,Print2Data_send.firmware.motor,Print2Data_send.firmware.debug);

  initSPI_slave(); //

  // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1635); // Запуск ШИМ на канале TIM8_CH1
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // HAL_Delay(5000);
  // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1200); // Запуск ШИМ на канале TIM8_CH1
  // HAL_Delay(40);
  // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1600); // Запуск ШИМ на канале TIM8_CH1
  // HAL_Delay(5000000);

  // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  // for (int i = 0; i < 5; i++)
  // {
  //   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1700);// Запуск ШИМ на канале TIM8_CH1
  //   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  //   HAL_Delay(3000);

  //   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1550);// Запуск ШИМ на канале TIM8_CH1
  //   //HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);// Остановка ШИМ на канале TIM8_CH1
  //   HAL_Delay(100);

  //   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1400);
  //   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  //   HAL_Delay(3000);
  //   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1550);// Запуск ШИМ на канале TIM8_CH1
  //   HAL_Delay(100);
  // }

  // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

  // uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // uint8_t stop[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFD};
  // uint8_t start[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFC};

  // uint8_t speedMode[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFA};
  // uint8_t torqueMode[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xF9};
  // uint8_t positionMode[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFB};

  // uint8_t setZero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFE};

  timeSpi = millis(); // Запоминаем время начала цикла
  // HAL_Delay(2000);
  DEBUG_PRINTF("%lli LOOP !!!!!!!!!!!!!!!!!!!!!!!!!!! \r\n", timeSpi);

  // CAN_SendMessage(stop, 8); // Отправляем данные
  // printf("%u CAN_SendMessage stop1\n", millis());
  // HAL_Delay(2000);
  // // CAN_SendMessage(positionMode, 8); // Отправляем данные
  // // printf("%u CAN_SendMessage positionMode\n", millis());
  // // HAL_Delay(2000);
  // // CAN_SendMessage(speedMode, 8); // Отправляем данные
  // // printf("%u CAN_SendMessage speedMode\n", millis());
  // // HAL_Delay(200);
  // CAN_SendMessage(torqueMode, 8); // Отправляем данные
  // printf("%u CAN_SendMessage torqueMode\n", millis());
  // HAL_Delay(200);
  // // setData(0, 20, 1, 1, 0, data);
  // // CAN_SendMessage(data, 8);  // Отправляем данные
  // HAL_Delay(100);
  // CAN_SendMessage(start, 8); // Отправляем данные
  // printf("%lu CAN_SendMessage start1\n", millis());
  // HAL_Delay(100);
  // // CAN_SendMessage(setZero, 8); // Отправляем данные
  // HAL_Delay(100);
  // CAN_SendMessage(stop, 8); // Отправляем данные
  // printf("%lu CAN_SendMessage stop2\n", millis());
  // HAL_Delay(5000000);
  // HAL_Delay(500);
  // CAN_SendMessage(setZero, 8); // Отправляем данные
  // HAL_Delay(500);
  // CAN_SendMessage(setZero, 8); // Отправляем данные
  // HAL_Delay(1000);
  bool flagStop = true;
  float pos = 0;
  while (1)
  {
    // if (millis() < 10000)
    // {
    //   setData(0, 0, 500, 5, -2.5, data);
    //   if (pos < 0.4)
    //   {
    //     pos = pos + 0.1;
    //   }
    //   else
    //   {
    //     pos = pos - 0.1;
    //   }
    //   printf("pos= %.2f \n", pos);
    //   CAN_SendMessage(data, 8); // Отправляем данные
    //   HAL_Delay(500);           // Задержка 1 секунда
    //   printf("data \n");
    // }
    // else if (flagStop)
    // {
    //   CAN_SendMessage(stop, 8); // Отправляем данные
    //   flagStop = false;
    //   printf("STOP !!! \n");
    // }

    workingSPI();             // Отработка действий по обмену по шине SPI
    workingTimer(); // Отработка действий по таймеру в 1, 50, 60 милисекунд

    // DEBUG_PRINTF("float %.2f Привет \n", 3.1415625);
    // HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);     // Инвертирование состояния выхода.
    // HAL_GPIO_TogglePin(Led2_GPIO_Port, Led2_Pin);     // Инвертирование состояния выхода.
    // HAL_GPIO_TogglePin(Analiz_GPIO_Port, Analiz_Pin); // Инвертирование состояния выхода.
    // HAL_Delay(500);
  }
}

#if DEBUG
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
#endif

/**  * @brief System Clock Configuration  */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */