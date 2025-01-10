
#include "can.h"

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16; // Настройка скорости ( делитель при частоте APB1)
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (canHandle->Instance == CAN1)
  {
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{

  if (canHandle->Instance == CAN1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  }
}

void CAN_ConfigFilters(void)
{
  CAN_FilterTypeDef filterConfig = {0};

  // Активируем фильтр для приема всех сообщений
  filterConfig.FilterIdHigh = 0x0000;
  filterConfig.FilterIdLow = 0x0000;
  filterConfig.FilterMaskIdHigh = 0x0000;
  filterConfig.FilterMaskIdLow = 0x0000;
  filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filterConfig.FilterBank = 0;
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  filterConfig.FilterActivation = ENABLE;

  // // Фильтр для ID 0x100 -> FIFO0
  // sFilterConfig.FilterIdHigh = 0x100 << 5; // ID 0x100 (11 бит)
  // sFilterConfig.FilterIdLow = 0x0000;
  // sFilterConfig.FilterMaskIdHigh = 0x7FF << 5; // Полная маска для ID
  // sFilterConfig.FilterMaskIdLow = 0x0000;
  // sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // В FIFO0
  // sFilterConfig.FilterBank = 0;                          // Первый фильтр
  // sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  // sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  // sFilterConfig.FilterActivation = ENABLE;

  // // Настройка фильтра
  //     sFilterConfig.FilterIdHigh = 0x123 << 5; // Старшие биты стандартного ID (11 бит сдвигаются влево на 5)
  //     sFilterConfig.FilterIdLow = 0x0000;     // Не используется для стандартного ID
  //     sFilterConfig.FilterMaskIdHigh = 0x7FF << 5; // Маска для проверки всех бит ID
  //     sFilterConfig.FilterMaskIdLow = 0x0000;     // Не используется для стандартного ID
  //     sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // Сообщения идут в FIFO0
  //     sFilterConfig.FilterBank = 0;             // Номер банка фильтра (используем первый)
  //     sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // Режим маски (для фильтрации ID)
  //     sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // Масштаб фильтра (один 32-битный фильтр)
  //     sFilterConfig.FilterActivation = ENABLE;  // Включение фильтра
  //     sFilterConfig.SlaveStartFilterBank = 14;  // Используется только для CAN2
  /*
    Структура CAN_FilterTypeDef:
  Эта структура задает параметры фильтра:
  FilterIdHigh	        Старшие 16 бит идентификатора фильтра.
  FilterIdLow	          Младшие 16 бит идентификатора фильтра.
  FilterMaskIdHigh	    Старшие 16 бит маски фильтра.
  FilterMaskIdLow   	  Младшие 16 бит маски фильтра.
  FilterFIFOAssignment	Назначение фильтра: CAN_FILTER_FIFO0 или CAN_FILTER_FIFO1.
  FilterBank	          Номер банка фильтра (0–13 для CAN1, 14–27 для CAN2, 0–27 при CAN1+CAN2).
  FilterMode          	Режим фильтра: CAN_FILTERMODE_IDMASK или CAN_FILTERMODE_IDLIST.
  FilterScale	          Масштаб фильтра: CAN_FILTERSCALE_16BIT или CAN_FILTERSCALE_32BIT.
  FilterActivation	    Включение фильтра: ENABLE или DISABLE.
  SlaveStartFilterBank	Начальный банк фильтров для CAN2 (только для двухконтроллерных систем).
  */

  if (HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void CAN_Notifications_Init(void)
{
  // Активация прерываний на передачу и прием в FIFO0
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING |
                                               CAN_IT_RX_FIFO1_MSG_PENDING |
                                               CAN_IT_ERROR_WARNING) != HAL_OK)
  {
    // Обработка ошибки
    Error_Handler();
  }
  /*
  События (маски прерываний):
CAN_IT_TX_MAILBOX_EMPTY       	Прерывание при освобождении почтового ящика (передача завершена).
CAN_IT_RX_FIFO0_MSG_PENDING   	Прерывание при получении сообщения в FIFO0.
CAN_IT_RX_FIFO1_MSG_PENDING	    Прерывание при получении сообщения в FIFO1.
CAN_IT_RX_FIFO0_FULL	          Прерывание при заполнении FIFO0.
CAN_IT_RX_FIFO1_FULL	          Прерывание при заполнении FIFO1.
CAN_IT_RX_FIFO0_OVERRUN       	Прерывание при переполнении FIFO0.
CAN_IT_RX_FIFO1_OVERRUN       	Прерывание при переполнении FIFO1.
CAN_IT_ERROR_WARNING	          Прерывание при предупреждении об ошибке.
CAN_IT_ERROR_PASSIVE          	Прерывание при переходе в пассивный режим из-за ошибок.
CAN_IT_BUSOFF	                  Прерывание при отключении CAN от шины.
CAN_IT_LAST_ERROR_CODE	        Прерывание для последнего кода ошибки.
CAN_IT_ERROR	                  Общее прерывание ошибок.
*/
}

void CAN_Start()
{
  // Запускаем CAN
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    // Запуск не удался
    Error_Handler();
  }
}
