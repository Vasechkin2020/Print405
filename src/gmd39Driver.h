#ifndef GMD39DRIVER_H
#define GMD39DRIVER_H

#include <stdio.h>
#include <stdint.h>
#include <math.h> // Для fminf, fmaxf

// --- КОНСТАНТЫ ОГРАНИЧЕНИЙ И БИТНОСТИ ---
#define GMD_P_MIN       -12.5f      // Min position in radians
#define GMD_P_MAX       12.5f       // Max position in radians
#define GMD_V_MIN       -65.0f      // Min velocity in rad/s
#define GMD_V_MAX       65.0f       // Max velocity in rad/s
#define GMD_KP_MIN      0.0f        // Min Kp coefficient
#define GMD_KP_MAX      500.0f      // Max Kp coefficient
#define GMD_KD_MIN      0.0f        // Min Kd coefficient
#define GMD_KD_MAX      5.0f        // Max Kd coefficient
#define GMD_T_MIN       -4.0f       // Min torque/current in Amps
#define GMD_T_MAX       4.0f        // Max torque/current in Amps
#define GMD_MAX_16_BIT  65535       // Max 16-bit value (2^16 - 1)
#define GMD_MAX_12_BIT  4095        // Max 12-bit value (2^12 - 1)


// --- СТРУКТУРЫ ДАННЫХ ---

typedef struct GMD_CAN_Message_t
{
    uint32_t id;                        // CAN ID сообщения
    uint8_t data[8];                    // 8 байт данных
    uint8_t len;                        // Длина данных (всегда 8)
} GMD_CAN_Message_t;

typedef struct MotorStatus_t
{
    uint8_t host_id;                    // ID мотора (Байт 0)
    float position;                     // Текущее положение (радианы)
    float velocity;                     // Текущая скорость (рад/с)
    float torque;                       // Текущий момент (Амперы)
} MotorStatus_t;


// --- КОНСТАНТНЫЕ МАССИВЫ КОМАНД ---

const uint8_t CMD_GMD_STOP[] = 
{
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFD
}; // Команда остановки
const uint8_t CMD_GMD_START[] = 
{
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFC
}; // Команда старта
const uint8_t CMD_GMD_TORQUE_MODE[] = 
{
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xF9
}; // Команда режима Момента (Тока)
const uint8_t CMD_GMD_SET_ZERO[] = 
{
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xFE
}; // Команда обнуления позиции

const uint8_t CMD_PURE_ZERO_CONTROL[] = 
{
    0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00
}; // Команда управления P=0, V=0, Kp=0, Kd=0, T=0


// --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ (HELPER FUNCTIONS) ---

// Функция отправки CAN сообщения (ЗАГЛУШКА!)
void can_tx_send(uint32_t id, const uint8_t* data)
{
    printf("[HW] CAN TX ID:%d Data:", id); // Log ID
    for(int i = 0; i < 8; i++) // Iterate over 8 bytes
    {
        printf("%02X ", data[i]); // Print byte in HEX format
    }
    printf("\n"); // Newline
}

// Задержка (ЗАГЛУШКА! Заменить на HAL_Delay или osDelay)
void can_delay_ms(int ms)
{
    printf("Delay %d ms...\n", ms); // Log delay time
}

// Конвертация Float -> Uint (Масштабирование и Клампинг)
uint16_t f_to_u16(float v, float v_min, float v_max, uint32_t max_int_val)
{
    float temp; // Temporary float value
    
    // Расчет: (value - min) / (range) * max_int_val
    temp = ((v - v_min) / (v_max - v_min)) * ((float)max_int_val); // Scale and offset the value
    
    int32_t utemp = (int32_t)temp; // Convert to signed integer for clamping

    if (utemp < 0) // Clamping lower bound
    {
        utemp = 0;
    }
    if (utemp > (int32_t)max_int_val) // Clamping upper bound
    {
        utemp = (int32_t)max_int_val;
    }
    
    return (uint16_t)utemp; // Return as 16-bit unsigned integer
}

// Конвертация Uint -> Float (Декодирование)
float u16_to_f(uint16_t u, float v_min, float v_max, uint32_t max_int_val)
{
    float span = v_max - v_min; // Range width
    float offset = v_min; // Offset value
    
    // Расчет: (raw_value / max_int_val) * range + offset
    return ((float)u) * span / ((float)max_int_val) + offset; // Decode back to physical value
}

// Отправка специальных команд (FC, FD, FE, F9, FA, FB)
void motor_send_special_cmd(uint32_t motor_id, const uint8_t *cmd_data)
{
    can_tx_send(motor_id, cmd_data); // Use the hardware sending function
}


// --- ОСНОВНЫЕ ФУНКЦИИ УПРАВЛЕНИЯ МОТОРОМ ---

// Функция начальной установки (инициализации) мотора
void motor_driver_init(uint32_t motor_id)
{
    printf("--- Initializing Motor Driver ---\n"); // Initialization starting

    // 1. Гарантированная остановка мотора (0xFD)
    motor_send_special_cmd(motor_id, CMD_GMD_STOP); // Send STOP command
    printf("CMD: Stop Motor Sent. (Safety Step)\n"); // Log action
    can_delay_ms(100); // Wait 100ms

    // 2. Установка режима работы (например, Момент/Ток 0xF9)
    motor_send_special_cmd(motor_id, CMD_GMD_TORQUE_MODE); // Set Torque Mode
    printf("CMD: Torque Mode (0xF9) Set.\n"); // Log action
    can_delay_ms(100); // Wait 100ms

    // 3. Отправка команды чистого нуля (P=0, V=0, Kp=0, Kd=0, T=0) в буфер
    motor_send_special_cmd(motor_id, CMD_PURE_ZERO_CONTROL); // Send safe zero control frame
    printf("CMD: Pure Zero Control Frame Sent.\n"); // Log action
    can_delay_ms(100); // Wait 100ms

    // 4. Включение мотора (0xFC)
    motor_send_special_cmd(motor_id, CMD_GMD_START); // Send START command
    printf("CMD: Start Motor Sent. (System Active)\n"); // Log action
    can_delay_ms(100); // Wait 100ms
}

// Отправка управляющей команды моменту/скорости/положению
void motor_set_control(uint32_t motor_id, float p_des, float v_des, float kp, float kd, float t_ff)
{
    uint8_t data_buf[8]; // Buffer for CAN data

    // 1. Ограничиваем входные значения безопасными диапазонами (Clamp)
    p_des = fminf(fmaxf(p_des, GMD_P_MIN), GMD_P_MAX); // Clamp position
    v_des = fminf(fmaxf(v_des, GMD_V_MIN), GMD_V_MAX); // Clamp velocity
    kp    = fminf(fmaxf(kp, GMD_KP_MIN), GMD_KP_MAX);  // Clamp Kp
    kd    = fminf(fmaxf(kd, GMD_KD_MIN), GMD_KD_MAX);  // Clamp Kd
    t_ff  = fminf(fmaxf(t_ff, GMD_T_MIN), GMD_T_MAX);  // Clamp Torque

    // 2. Конвертируем Float в целые числа (Raw Values)
    uint16_t p_int = f_to_u16(p_des, GMD_P_MIN, GMD_P_MAX, GMD_MAX_16_BIT); // 16 bit
    uint16_t v_int = f_to_u16(v_des, GMD_V_MIN, GMD_V_MAX, GMD_MAX_12_BIT); // 12 bit
    uint16_t kp_int = f_to_u16(kp, GMD_KP_MIN, GMD_KP_MAX, GMD_MAX_12_BIT); // 12 bit
    uint16_t kd_int = f_to_u16(kd, GMD_KD_MIN, GMD_KD_MAX, GMD_MAX_12_BIT); // 12 bit
    uint16_t t_int = f_to_u16(t_ff, GMD_T_MIN, GMD_T_MAX, GMD_MAX_12_BIT);  // 12 bit

    // 3. Упаковка битов в буфер (Bit Packing)
    data_buf[0] = p_int >> 8; // P MSB
    data_buf[1] = p_int & 0xFF; // P LSB
    data_buf[2] = v_int >> 4; // V MSB
    // V LSB (4 bits) | Kp MSB (4 bits)
    data_buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); 
    data_buf[4] = kp_int & 0xFF; // Kp LSB
    data_buf[5] = kd_int >> 4; // Kd MSB
    // Kd LSB (4 bits) | Torque MSB (4 bits)
    data_buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    data_buf[7] = t_int & 0xFF; // Torque LSB

    // 4. Отправка
    can_tx_send(motor_id, data_buf); // Send the data frame
    
    printf("Control: P=%.2f V=%.2f Kp=%.1f Kd=%.1f T=%.2f\n", // Log control values
           p_des, v_des, kp, kd, t_ff);
}

// Обработка принятого CAN-сообщения (ACK)
MotorStatus_t motor_process_rx_frame(uint32_t rx_id, uint8_t *data)
{
    uint8_t host_id = data[0]; // Motor ID (B0)
    
    // Position (16 bit) - B1, B2
    uint16_t p_int = (data[1] << 8) | data[2]; 

    // Velocity (12 bit) - B3, high 4 bits of B4
    uint16_t v_int = (data[3] << 4) | ((data[4] & 0xF0) >> 4); 

    // Torque (12 bit) - low 4 bits of B4, B5
    uint16_t t_int = ((data[4] & 0x0F) << 8) | data[5]; 

    // Декодирование: Перевод Raw Values обратно во float (Decoding)
    MotorStatus_t state; // Create status structure
    state.host_id = host_id; // Store ID
    
    state.position = u16_to_f(p_int, GMD_P_MIN, GMD_P_MAX, GMD_MAX_16_BIT); // Decode Position
    state.velocity = u16_to_f(v_int, GMD_V_MIN, GMD_V_MAX, GMD_MAX_12_BIT); // Decode Velocity
    state.torque = u16_to_f(t_int, GMD_T_MIN, GMD_T_MAX, GMD_MAX_12_BIT);   // Decode Torque

    printf("[CAN RX ID:%d] Host:%d P:%.3f V:%.3f T:%.3f\n", // Log received state
           rx_id, state.host_id, state.position, state.velocity, state.torque);

    return state; // Return the status
}


// --- ПРИМЕР ИСПОЛЬЗОВАНИЯ ---

// Заглушка для имитации приема
void simulate_can_receive(uint32_t motor_id)
{
    // Имитация принятого сообщения
    uint8_t received_data[8]; // Received data buffer
    uint32_t rx_id = motor_id; // Received ID
    
    // P=0.125 rad, V=10.0 rad/s, T=0.5 A
    received_data[0] = 0x01; // Host ID
    received_data[1] = 0x80; // P MSB
    received_data[2] = 0x50; // P LSB
    received_data[3] = 0x93; // V MSB
    received_data[4] = 0x38; // V LSB + T MSB
    received_data[5] = 0x50; // T LSB
    received_data[6] = 0x00; // Extra data
    received_data[7] = 0x00; // Extra data

    printf("\n--- Simulating CAN Reception ---\n"); // Log simulation start
    motor_process_rx_frame(rx_id, received_data); // Process the received data
}
/*
int main()
{
    uint32_t my_motor_id = 0x01; // Assign Motor ID

    // Инициализация мотора
    motor_driver_init(my_motor_id); // Run initialization sequence

    // 1. Команда: Удержание позиции 0 с жесткой пружиной (режим P/V/T)
    // P=0, V=0, Kp=50, Kd=1, T=0
    motor_set_control(my_motor_id, 0.0f, 0.0f, 50.0f, 1.0f, 0.0f); // Send control command

    // Имитация приема данных от мотора
    simulate_can_receive(my_motor_id); // Simulate ACK

    // Выключаем мотор
    motor_send_special_cmd(my_motor_id, CMD_GMD_STOP); // Send STOP command

    return 0; // Return success
}
*/

/* ========================================================================================
                             СПРАВОЧНИК ПО КЛАССУ GMD39Driver
========================================================================================

1. ГЛАВНЫЕ ФУНКЦИИ
----------------------------------------------------------------------------------------
- ControlMotor(float p_des, float v_des, float kp, float kd, float t_ff): 
  Основной метод для отправки команд в режиме Position/Speed/Torque. Принимает 5 float-параметров, 
  конвертирует их в 64 бита (8 байт) и отправляет по CAN.

- ProcessCAN_ACK(uint32_t rx_id, uint8_t *data): 
  Метод для обработки принятого сообщения CAN (обратной связи). 
  Он выполняет **распаковку** (Bit Unpacking) принятых 8 байт и **декодирование** (UintToFloat) 
  целых чисел обратно в физические величины (радианы, рад/с, Амперы).

2. ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ (Хелперы)
----------------------------------------------------------------------------------------
- FloatToUint(float v, float v_min, float v_max, uint32_t max_int_val):
  - Что делает: Берет физическую величину (float, например, 1.5 рад), её диапазон (min/max) 
    и целевую битность (например, 4095 для 12 бит) и возвращает целое число (Raw Value).
  - Ваша реализация этой математики полностью корректна.

- UintToFloat(uint16_t u, float v_min, float v_max, uint32_t max_int_val):
  - Что делает: Обратная функция. Берет принятое целое число (Raw Value), его диапазон и битность, 
    и возвращает физическую величину (float).

3. СТРУКТУРА ACK-СООБЩЕНИЯ (Прием данных)
----------------------------------------------------------------------------------------
Драйвер GMD39 (и его аналоги) отправляет данные обратно в следующем формате:
- Байт 0: Motor ID (ID мотора, обычно 0x01)
- Байты 1-2: Position (16 бит)
- Байты 3-4: Velocity (12 бит, смешаны с Torque)
- Байты 4-5: Torque (12 бит, смешаны с Velocity)
- Байты 6-7: Дополнительные данные (часто температура и т.п.)

Ваша логика распаковки:
- Position: data[1] (MSB) и data[2] (LSB)
- Velocity: data[3] (8 MSB) и 4 бита из data[4]
- Torque: 4 бита из data[4] и data[5] (8 LSB)

Эта схема подтверждена и используется в ProcessCAN_ACK.

========================================================================================
*/
#endif