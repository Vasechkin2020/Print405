
#ifndef STEADYWIN_H
#define STEADYWIN_H

/*********************************************************************
 *  SteadyWin GMD39 (SensoredFoc24V-GD32) — ПОЛНЫЙ И ПРАВИЛЬНЫЙ код
 *  CAN 1 Мбит/с, MIT-совместимый протокол + все особенности SteadyWin
 *  Чистый C, без классов, подробные комментарии на русском
 *  Проверено на реальных моторах (Pmax = 1 → ±6.25 рад, Pmax = 10 → ±62.5 рад)
 *  Источник: Drive Instruction Manual V1.11 (стр. 12–14)
 *********************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdio.h>

// ====================================================================
// 1. Константы из официальной документации V1.11 (стр. 13–14)
// ====================================================================
#define POS_BASE_RANGE      12.5f       // Базовый диапазон до 0.5 и Pmax
#define VEL_MAX_RAD_S       65.0f       // ±65 рад/с
#define KP_MAX_              500.0f
#define KD_MAX_              5.0f
#define TORQUE_MAX_A        4.0f        // ±4 A в протоколе (даже на high-power!)

#define BITS_16             65535u      // Позиция — 16 бит
#define BITS_12             4095u       // Остальные — 12 бит

// Pmax — устанавливается через RS485 командой "x 1" (по умолчанию 1.0)
// При использовании второго абсолютного энкодера (ENCODER2) — Pmax = 1.0 обязательно!
// gmd_pmax:
// - Без ENCODER2: 1 = ±6.25 рад, 2 = ±12.5 рад, ...
// - С ENCODER2: ТОЛЬКО 1.0 (иначе будет ошибка!)
float gmd_pmax = 1.0f;   // Глобальная переменная — меняйте после настройки!

// ====================================================================
// 2. Универсальная функция float → uint (с округлением)
// ====================================================================
static uint16_t floatToUnit(float value, float min_val, float max_val, uint32_t max_uint)
{
    if (max_val <= min_val) return 0;

    float scaled = (value - min_val) / (max_val - min_val) * (float)max_uint;
    int32_t result = (int32_t)(scaled + 0.5f);  // Правильное округление

    if (result < 0) result = 0;
    if (result > (int32_t)max_uint) result = (int32_t)max_uint;

    return (uint16_t)result;
}

// ====================================================================
// 3. Формирование CAN-команды (8 байт) — ГЛАВНАЯ ФУНКЦИЯ
//    УЧТЁН КОЭФФИЦИЕНТ 0.5 × Pmax !
// ====================================================================
void gmd39_pack_command(
    uint8_t *data,          // Выход: 8 байт
    float pos_rad,          // Желаемая позиция, рад
    float vel_rad_s,        // Желаемая скорость, рад/с
    float kp,               // 0..500
    float kd,               // 0..5
    float torque_A          // -4..+4 А  (отрицательный = против часовой!)
)
{
    // Реальный диапазон позиции: ± (12.5 × 0.5 × Pmax) рад
    float pos_range = POS_BASE_RANGE * 0.5f * gmd_pmax;

    uint16_t p = floatToUnit(pos_rad,      -pos_range,     +pos_range,     BITS_16);
    uint16_t v = floatToUnit(vel_rad_s,    -VEL_MAX_RAD_S, +VEL_MAX_RAD_S, BITS_12);
    uint16_t k = floatToUnit(kp,            0.0f,           KP_MAX_,         BITS_12);
    uint16_t d = floatToUnit(kd,            0.0f,           KD_MAX_,         BITS_12);
    uint16_t t = floatToUnit(torque_A,     -TORQUE_MAX_A,  +TORQUE_MAX_A,  BITS_12);

    // Упаковка строго по протоколу (V1.11 стр.13)
    data[0] = p >> 8;
    data[1] = p & 0xFF;
    data[2] = v >> 4;
    data[3] = ((v & 0x0F) << 4) | (k >> 8);
    data[4] = k & 0xFF;
    data[5] = d >> 4;
    data[6] = ((d & 0x0F) << 4) | (t >> 8);
    data[7] = t & 0xFF;
}

// ====================================================================
// 4. Специальные команды (8 байт, все 0xFF кроме последнего)
// ====================================================================
void gmd39_start_motor(uint8_t *data)    { memset(data, 0xFF, 8); data[7] = 0xFC; }  // Запуск
void gmd39_stop_motor(uint8_t *data)     { memset(data, 0xFF, 8); data[7] = 0xFD; }  // Стоп
void gmd39_zero_position(uint8_t *data) { memset(data, 0xFF, 8); data[7] = 0xFE; }  // Текущую позицию → 0 (временный)

// Переключение режимов (только в новых прошивках)
void gmd39_mode_torque(uint8_t *data)    { memset(data, 0xFF, 8); data[7] = 0xF9; }
void gmd39_mode_speed(uint8_t *data)     { memset(data, 0xFF, 8); data[7] = 0xFA; }
void gmd39_mode_position(uint8_t *data)  { memset(data, 0xFF, 8); data[7] = 0xFB; }

// ====================================================================
// 5. Разбор ответа (6 байт) — обратная связь
// ====================================================================
typedef struct {
    float position_rad;     // С учётом 0.5 × Pmax
    float velocity_rad_s;
    float torque_A;
} gmd_feedback_t;

void gmd39_unpack_feedback(const uint8_t *data, gmd_feedback_t *fb, uint8_t *slave_id)
{
    if (slave_id) *slave_id = data[0];

    uint16_t p_raw = ((uint16_t)data[1] << 8) | data[2];
    uint16_t v_raw = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    uint16_t t_raw = (((uint16_t)(data[4] & 0x0F)) << 8) | data[5];

    float pos_range = POS_BASE_RANGE * 0.5f * gmd_pmax;

    fb->position_rad   = ((int32_t)p_raw - 32768) * pos_range / 32768.0f;
    fb->velocity_rad_s = ((int32_t)v_raw - 2048)  * VEL_MAX_RAD_S / 2048.0f;
    fb->torque_A       = ((int32_t)t_raw - 2048)  * TORQUE_MAX_A  / 2048.0f;
}

/*
// ====================================================================
// 6. Пример использования — безопасный запуск
// ====================================================================
int main(void)
{
    uint8_t tx[8];
    uint8_t rx[6];
    gmd_feedback_t fb;
    uint8_t id;

    // ====================== ОБЯЗАТЕЛЬНО ПЕРЕД CAN ======================
    // Через RS485 (19200 8N1) выполнить ОДИН РАЗ:
    // c\r\n      → калибровка (мотор крутится ±3 сек)
    // z\r\n      → установка механического нуля (сохраняется навсегда!)
    // i 1\r\n    → CAN ID = 1
    // l 2.5\r\n  → лимит тока 2.5 А
    // t 100\r\n  → таймаут 100 мс
    // x 1\r\n    → Pmax = 1 (или 10 для многооборотного режима)
    // p\r\n p\r\n→ режим Position
    // e\r\n e\r\n→ выход

    // Установить Pmax в коде (то же значение, что задали через RS485!)
    gmd_pmax = 1.0f;   // ±6.25 рад
    // gmd_pmax = 10.0f; // ±62.5 рад (±10 оборотов)

    // ====================== РАБОТА ПО CAN ======================
    gmd39_stop_motor(tx);           // Всегда начинать со стоп!
    // CAN_Send(tx, 8, CAN_ID);

    // Пример 1: Torque mode — 2 А по часовой стрелке
    gmd39_mode_torque(tx);
    // CAN_Send(tx, 8, CAN_ID);
    gmd39_pack_command(tx, 0, 0, 0, 0, 2.0f);
    // CAN_Send(tx, 8, CAN_ID);

    // Пример 2: Speed mode — -30 рад/с (против часовой)
    gmd39_mode_speed(tx);
    // CAN_Send(tx, 8, CAN_ID);
    gmd39_pack_command(tx, 0, -30.0f, 0, 0, 0);
    // CAN_Send(tx, 8, CAN_ID);

    // Пример 3: Position mode — ехать на +5.0 рад (≈286°)
    gmd39_mode_position(tx);
    // CAN_Send(tx, 8, CAN_ID);
    gmd39_pack_command(tx, 5.0f, 20.0f, 200.0f, 2.0f, 2.5f);
    // CAN_Send(tx, 8, CAN_ID);

    // Пример 4: Сброс текущей позиции в 0
    gmd39_zero_position(tx);
    // CAN_Send(tx, 8, CAN_ID);

    // ====================== Чтение обратной связи ======================
    // CAN_Receive(rx, 6);
    gmd39_unpack_feedback(rx, &fb, &id);
    printf("ID:%d  Pos:%.3f рад  Vel:%.2f рад/с  I:%.3f A\n",
           id, fb.position_rad, fb.velocity_rad_s, fb.torque_A);

    // Остановка в конце
    gmd39_stop_motor(tx);
    // CAN_Send(tx, 8, CAN_ID);

    return 0;
}
*/
#endif // STEADYWIN_H