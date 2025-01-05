#ifndef GIM43XX_H
#define GIM43XX_H

uint8_t start[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFC};
uint8_t stop[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFD};
uint8_t setZero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFE};

#define P_MIN 0
#define P_MAX 65535
#define V_MIN 0
#define V_MAX 4096
#define KP_MIN 0
#define KP_MAX 4096
#define KD_MIN 0
#define KD_MAX 4096
#define C_MIN 0
#define C_MAX 4096

uint16_t float_to_uint(float v, float v_min, float v_max, uint32_t width)
{
    float temp;
    int32_t utemp;
    temp = ((v - v_min) / (v_max - v_min)) * ((float)width);
    utemp = (int32_t)temp;
    if (utemp < 0)
        utemp = 0;
    if (utemp > width)
        utemp = width;
    return utemp;
}

void setData(uint16_t position, uint16_t velocity, uint16_t kp, uint16_t kd, uint16_t current, uint8_t *data)
{
    // Расчет значений
    uint16_t f_position = position / 12.5 * 32768 + 32768;
    uint16_t f_velocity = velocity / 65 * 2048 + 2048;
    uint16_t f_kp = kp / 500 * 4096;
    uint16_t f_kd = kd / 5 * 4096;
    uint16_t f_current = current / 4 * 2048 + 2048;

    // Расчет проверка значений на диапазон разрешенный
    uint16_t s_p_int = float_to_uint(f_position, P_MIN, P_MAX, 65535);
    uint16_t s_v_int = float_to_uint(f_velocity, V_MIN, V_MAX, 4096);
    uint16_t s_Kp_int = float_to_uint(f_kp, 0, KP_MAX, 4096);
    uint16_t s_Kd_int = float_to_uint(f_kd, 0, KD_MAX, 4096);
    uint16_t s_c_int = float_to_uint(f_current, -C_MAX, C_MAX, 4096);
    // Запись в массив данных для оправки по CAN шине
    data[0] = s_p_int >> 8;
    data[1] = s_p_int & 0xFF;
    data[2] = s_v_int >> 4;
    data[3] = ((s_v_int & 0xF) << 4) + (s_Kp_int >> 8);
    data[4] = s_Kp_int & 0xFF;
    data[5] = s_Kd_int >> 4;
    data[6] = ((s_Kd_int & 0xF) << 4) + (s_c_int >> 8);
    data[7] = s_c_int & 0xFF;
}

void ackData(uint8_t *data)
{
    // Из байт ответа по шине получаем юниты
    uint8_t id = data[0];
    uint16_t p = (int16_t)((data[2] << 8) | data[1]);
    uint16_t v = (int16_t)((data[3] << 4) | data[4] >> 4);
    uint16_t t = (int16_t)((data[4] << 4) | data[5]);

    // Вычисляем итоговые значения
    uint16_t position = (p - 32768) / 32768 * 12.5;
    uint16_t speed = (v - 2048) / 2048 * 65;
    uint16_t torque = (t - 2048) / 2048 * 4;
}
#endif