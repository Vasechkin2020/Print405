#ifndef GIM43XX_H
#define GIM43XX_H

#define P_MIN 0
#define P_MAX 65534
#define V_MIN 0
#define V_MAX 4095
#define KP_MIN 0
#define KP_MAX 4095
#define KD_MIN 0
#define KD_MAX 4095
#define C_MIN 0
#define C_MAX 4095

uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t stop[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFD};
uint8_t start[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFC};

uint8_t speedMode[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFA};
uint8_t torqueMode[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xF9};
uint8_t positionMode[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFB};

uint8_t setZero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFE};

struct SGim43 dataGim43;

extern uint32_t millis();

// Установка режима работы мотора
void initGim43()
{
    printf("--- initGim43...\n");
    CAN_SendMessage(stop, 8); // Отправляем данные
    printf("Set CAN_SendMessage stop\n");
    HAL_Delay(100);
    CAN_SendMessage(torqueMode, 8); // Отправляем данные
    printf("Set CAN_SendMessage torqueMode\n");
    HAL_Delay(100);
    // CAN_SendMessage(positionMode, 8); // Отправляем данные
    // printf("%u CAN_SendMessage positionMode\n", millis());
    // HAL_Delay(100);
    // CAN_SendMessage(speedMode, 8); // Отправляем данные
    // printf("%u CAN_SendMessage speedMode\n", millis());
    // HAL_Delay(100);
}

uint16_t float_to_uint(float v, float v_min, float v_max, uint32_t width)
{
    float temp;
    int32_t utemp;
    temp = ((v - v_min) / (v_max - v_min)) * ((float)width);
    utemp = (int32_t)temp;
    // printf("float_to_uint %.2f %.2f %.2f | %lu = %i \n", v, v_min, v_max, width, utemp);
    if (utemp < 0)
        utemp = 0;
    if (utemp > width)
        utemp = width;
    return utemp;
}

void setData(float position, float velocity, float kp, float kd, float current, uint8_t *data)
{
    // Расчет значений
    float f_position = position / 12.5 * 32768 + 32768;
    float f_velocity = velocity / 65 * 2048 + 2048;
    float f_kp = kp / 500 * 4096;
    float f_kd = kd / 5 * 4096;
    float f_current = ((current / 4.0) * 2048) + 2048;

    // Расчет проверка значений на диапазон разрешенный
    uint16_t s_p_int = float_to_uint(f_position, P_MIN, P_MAX, 65534);
    uint16_t s_v_int = float_to_uint(f_velocity, V_MIN, V_MAX, 4096);
    uint16_t s_Kp_int = float_to_uint(f_kp, 0, KP_MAX, 4096);
    uint16_t s_Kd_int = float_to_uint(f_kd, 0, KD_MAX, 4096);
    uint16_t s_c_int = float_to_uint(f_current, 0, C_MAX, 4096);

    // printf("s_p_int= %u ", s_p_int);
    // printf("s_v_int= %u ", s_v_int);
    // printf("s_Kp_int= %u ", s_Kp_int);
    // printf("s_Kd_int= %u ", s_Kd_int);
    // printf("current= %f | ", current);
    // printf("f_current= %f | ", f_current);
    // printf("s_c_int= %u | ", s_c_int);
    // Запись в массив данных для оправки по CAN шине / 0x80 0 0x93 0xb3 0x33 0x33 0x3a 0/ 0x80 0 0xa7 0x66 0x66 0x99 0x9c 0
    data[0] = s_p_int >> 8;
    data[1] = s_p_int & 0xFF;
    data[2] = s_v_int >> 4;
    data[3] = ((s_v_int & 0xF) << 4) + (s_Kp_int >> 8);
    data[4] = s_Kp_int & 0xFF;
    data[5] = s_Kd_int >> 4;
    data[6] = ((s_Kd_int & 0xF) << 4) + (s_c_int >> 8);
    data[7] = s_c_int & 0xFF;
    // ChatGPT вариант 0x80 0 0x93 0xb3 0x33 0x3 0x3a 0 / 0x80 0 0xa7 0x66 0x66 0x9 0x9c 0/
    // data[0] = (s_p_int >> 8) & 0xFF;                              // Upper 8 bits
    // data[1] = s_p_int & 0xFF;                                     // Lower 8 bits

    // data[2] = (s_v_int >> 4) & 0xFF;                              // Upper 4 bits
    // data[3] = ((s_v_int & 0x0F) << 4) | ((s_Kp_int >> 8) & 0x0F); // Lower 4 bits of speed, upper 4 bits of Kp

    // data[4] = s_Kp_int & 0xFF;                                    // Lower 8 bits
    // data[5] = (s_Kd_int >> 4) & 0xFF;                             // Upper 4 bits

    // data[6] = ((s_Kd_int & 0x0F) << 4) | ((s_c_int >> 8) & 0x0F); // Lower 4 bits of Kd, upper 4 bits of torque
    // data[7] = s_c_int & 0xFF;                                     // Lower 8 bits

    // for (int i = 0; i < 8; i++)
    // {
    //     printf("%#x ", data[i]);
    // }
    // printf("\r\n");
}
typedef struct
{
    uint8_t host_id;
    float position; // 16 бит
    float velocity; // 12 бит
    float torque;   // 12 бит
} CAN_ACK_Message;

CAN_ACK_Message parse_CAN_ACK(uint8_t *data)
{
    uint8_t host_id = data[0];                                    // Извлечение Host ID
    uint16_t position = (data[1] << 8) | data[2];                 // Текущее положение
    uint16_t velocity = (data[3] << 4) | ((data[4] & 0xF0) >> 4); // Текущая скорость
    uint16_t torque = ((data[4] & 0x0F) << 8) | data[5];          // Текущий момент

    // data[3] = 0x12;
    // data[4] = 0x34;
    // data[5] = 0x56;
    // velocity = (data[3] << 4) | ((data[4] & 0xF0) >> 4);
    // torque = ((data[4] & 0x0F) << 8) | data[5];

    // printf("vv vel = %#x tor = %#x \n", velocity, torque);

    CAN_ACK_Message message;
    // Вычисляем итоговые значения
    message.host_id = host_id;
    message.position = (position - 32768) / 32768.0 * 12.5;
    message.velocity = (velocity - 2048) / 2048.0 * 65;
    message.torque = 2 * (torque - 2048) / 2048.0 * 4;

    // printf("host_id= %u ",host_id);
    // printf("position= %u ",position);
    // printf("velocity= %u ",velocity);
    // printf("torque= %u \n",torque);

    dataGim43.position = message.position;
    dataGim43.velocity = message.velocity;
    dataGim43.torque = message.torque;

    printf("host_id= %u ", host_id);
    printf("position= %.2f ", message.position);
    printf("velocity= %.2f ", message.velocity);
    printf("torque= %.2f \n", message.torque);
    return message;
}

// Колбек на отправку сообщения
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    // Данные успешно переданы
    // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Мигание светодиодом на успешную отправку
    // DEBUG_PRINTF("HAL_CAN_TxMailbox0CompleteCallback. \r\n");
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // DEBUG_PRINTF("HAL_CAN_RxFifo0MsgPendingCallback. \r\n");
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) // Считывание сообщения из FIFO0
    {
        // Обработка данных
        // printf("Data receive: ");
        // for (int i = 0; i < 8; i++)
        // {
        //     printf("%#x ", rxData[i]);
        // }
        // printf("\r\n");
        parse_CAN_ACK(rxData);
        // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Пример: переключение светодиода
    }
    else
    {
        Error_Handler(); // Обработка ошибок
    }
}

// void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//     CAN_RxHeaderTypeDef rxHeader;
//     uint8_t rxData[8];
//     DEBUG_PRINTF("HAL_CAN_RxFifo1MsgPendingCallback. \r\n");

//     // Считывание сообщения из FIFO1
//     if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK)
//     {
//         // Обработка данных
//         // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // Пример: переключение другого светодиода
//     }
//     else
//     {
//         // Обработка ошибок
//         Error_Handler();
//     }
// }

// void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//     CAN_RxHeaderTypeDef rxHeader;
//     uint8_t rxData[8];

//     // Считывание сообщения из FIFO1
//     if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK) {
//         // Обработка данных
//         // Например, мигнуть светодиодом или сохранить данные в буфер
//         HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//     } else {
//         // Обработка ошибки приема
//         Error_Handler();
//     }
// }

void testPWM_Gim43()
{

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1635); // Запуск ШИМ на канале TIM8_CH1
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_Delay(5000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1200); // Запуск ШИМ на канале TIM8_CH1
    HAL_Delay(40);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1600); // Запуск ШИМ на канале TIM8_CH1
    HAL_Delay(5000);

    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    for (int i = 0; i < 5; i++)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1700); // Запуск ШИМ на канале TIM8_CH1
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_Delay(3000);

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1550); // Запуск ШИМ на канале TIM8_CH1
        // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);// Остановка ШИМ на канале TIM8_CH1
        HAL_Delay(100);

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1400);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_Delay(3000);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1550); // Запуск ШИМ на канале TIM8_CH1
        HAL_Delay(100);
    }

    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

// void testCAN_Gim43()
// {

// HAL_Delay(2000);

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
// bool flagStop = true;
// float pos = 0;
// }

// void loop_gim43()
// {
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
// }

#endif