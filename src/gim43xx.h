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

    printf("s_p_int= %u ", s_p_int);
    printf("s_v_int= %u ", s_v_int);
    printf("s_Kp_int= %u ", s_Kp_int);
    printf("s_Kd_int= %u ", s_Kd_int);
    // printf("current= %f | ", current);
    // printf("f_current= %f | ", f_current);
    printf("s_c_int= %u | ", s_c_int);
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

    for (int i = 0; i < 8; i++)
    {
        printf("%#x ", data[i]);
    }
    printf("\r\n");
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

#endif