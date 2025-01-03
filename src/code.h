#ifndef CODE_H
#define CODE_H

#define SPI_protocol yes

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "slaveSPI.h"

//********************************* ПЕРЕМЕННЫЕ ***************************************************************************

bool flag_timer_10millisec = false;
bool flag_timer_50millisec = false;
bool flag_timer_1sec = false;

GPIO_TypeDef *myPort;

void timer6();                                                             // Обработчик прерывания таймера TIM6	1 раз в 1 милисекунду
void timer7();                                                             // Обработчик прерывания таймера TIM7	1 раз в 1 милисекунду
void workingTimer();                                                       // Отработка действий по таймеру в 1, 50, 60 милисекунд
void workingSPI();                                                         // Отработка действий по обмену по шине SPI
void initFirmware();                                                       // Заполнение данными Прошивки

HAL_StatusTypeDef status;
HAL_SPI_StateTypeDef statusGetState;

bool flagTimeOut = true;       // Флаг таймаута при обрыве связи по SPI
bool flagCallBackUart = false; // Флаг для указания нужно ли отрабатывать в колбеке  или обраьотка с самой функции

extern volatile uint32_t millisCounter;

//********************************* ФУНКЦИИ ***************************************************************************
// Функция для возврата количества миллисекунд
uint32_t millis()
{
    return millisCounter;
}

void timer7() // Обработчик прерывания таймера TIM7
{
}
void timer6() // Обработчик прерывания таймера TIM6	1 раз в 1 милисекунду
{
    static int count_timer_10millisec = 0; // Счетчик для запуска обработки движения моторов в лупе по флагу
    static int count_timer_50millisec = 0; // Счетчик для запуска каждые 50 милисекунд
    static int count_timer_1sec = 0;       // Счетчик для запуска

    count_timer_10millisec++;
    count_timer_50millisec++;
    count_timer_1sec++;

    millisCounter++; // Увеличиваем счетчик миллисекунд

    //  каждые 10 милисекунд
    if (count_timer_10millisec >= 10)
    {
        count_timer_10millisec = 0;
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
        flag_timer_10millisec = true;
    }
    // каждые 50 милисекунд
    if (count_timer_50millisec >= 50)
    {
        count_timer_50millisec = 0;
        flag_timer_50millisec = true;
    }
    // 1 seconds
    if (count_timer_1sec >= 1000)
    {
        count_timer_1sec = 0;
        flag_timer_1sec = true;
    }
}

void workingTimer() // Отработка действий по таймеру в 1, 50, 60 милисекунд
{
    // HAL_Delay(); // Пауза 500 миллисекунд.
    //----------------------------- 10 миллисекунд --------------------------------------
    if (flag_timer_10millisec)
    {
        flag_timer_10millisec = false;
        // HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);             // Инвертирование состояния выхода.
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10); // Инвертирование состояния выхода.
    }
    //----------------------------- 50 миллисекунд --------------------------------------
    if (flag_timer_50millisec)
    {
        flag_timer_50millisec = false;
        // DEBUG_PRINTF("50msec %li \r\n", millis());
        //  flag_data = true; // Есть новые данные по шине // РУчной вариант имитации пришедших данных с частотой 20Гц
    }

    //----------------------------- 1 секунда --------------------------------------
    if (flag_timer_1sec) // Вызывается каждую секунду
    {
        flag_timer_1sec = false;
        // statusGetState = HAL_SPI_GetState(&hspi1);
        // if (statusGetState == HAL_SPI_STATE_READY)
        // {
        //     DEBUG_PRINTF("Timer SPI. ПЕРЕЗАПУСК ПО ТАЙМЕРУ. ЭТО ОШИБКА\n");
        //     HAL_SPI_Abort(&hspi1);
        //     HAL_SPI_DMAStop(&hspi1);
        //     HAL_SPI_TransmitReceive_DMA(&hspi1, txBuffer, rxBuffer, BUFFER_SIZE); // // Перезапуск функции для следующего обмена// Запуск обмена данными по SPI с использованием DMA
        // }
        // else
        // {
        //     // DEBUG_PRINTF("Timer HAL_SPI_STATE_BUSY_TX_RX %u \n", statusGetState);
        // }
        // HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin); // Инвертирование состояния выхода.
        DEBUG_PRINTF("%li \r\n", millis());
        //  uint8_t UART1_rxBuffer[4] = {0xAA,0xFF,0xAA,0xFF};
        //   uint8_t UART1_rxBuffer[1] = {0x56}; //Запрос версии "V"
        //   uint8_t UART1_rxBuffer[1] = {0x4F}; // Включить лазер "O"
        //   uint8_t UART1_rxBuffer[1] = {0x43}; // Выключить лазер "C"
    }
}

// Собираем нужные данные и пишем в структуру на отправку
void collect_Data_for_Send()
{
    /*
    Modul2Data_send.id++;
    Modul2Data_send.pinMotorEn = HAL_GPIO_ReadPin(En_Motor_GPIO_Port, En_Motor_Pin); // Считываем состояние пина драйверов

    for (int i = 0; i < 4; i++) // Информация по моторам всегда
    {
        Modul2Data_send.motor[i].status = motor[i].status;                                       // Считываем состояние пина драйверов
        Modul2Data_send.motor[i].position = getAngle(motor[i].position);                         // Записываем текущую позицию преобразуя из импульсов в градусы, надо еще в глобальную систему преобразовывать
        Modul2Data_send.motor[i].destination = getAngle(motor[i].destination);                   // Считываем цель по позиции, надо еще в глобальную систему преобразовывать
        Modul2Data_send.micric[i] = HAL_GPIO_ReadPin(motor[i].micric_port, motor[i].micric_pin); //
    }

    for (int i = 0; i < 4; i++) // Информация по лазерам по ситуации
    {
        if (Data2Modul_receive.controlLaser.mode != 0) // Если команда работать с датчиком
        {
            Modul2Data_send.laser[i].status = dataUART[i].status;                                // Считываем статаус дальномера
            Modul2Data_send.laser[i].distance = (float)dataUART[i].distance * 0.001;             // Считываем измерение растояния и пересчитываем в метры !!!
            Modul2Data_send.laser[i].signalQuality = dataUART[i].quality;                        // Считываем качество сигнала измерение
            Modul2Data_send.laser[i].angle = (float)dataUART[i].angle;                           // Считываем угол в котором произвели измерение
            Modul2Data_send.laser[i].time = dataUART[i].time;                                    // Считываем время в котором произвели измерение
            Modul2Data_send.laser[i].numPillar = Data2Modul_receive.controlMotor.numPillar[i];   // Переписываем номер столба на который измеряли расстояние
            Modul2Data_send.laser[i].rate = dataUART[i].rate;
        }
        else
        {
            Modul2Data_send.laser[i].status = 888; // Статус не работаем с датчиком
            Modul2Data_send.laser[i].distance = 0;
            Modul2Data_send.laser[i].signalQuality = 0;
            Modul2Data_send.laser[i].angle = 0;
            Modul2Data_send.laser[i].time = 0;
            Modul2Data_send.laser[i].numPillar = -1; // Номер не существующего столба
            Modul2Data_send.laser[i].rate = 0;       //
        }
    }

    Modul2Data_send.bno055 = bno055;
    Modul2Data_send.spi = spi;

    uint32_t cheksum_send = 0;                                          // Считаем контрольную сумму отправляемой структуры
    unsigned char *adr_structura = (unsigned char *)(&Modul2Data_send); // Запоминаем адрес начала структуры. Используем для побайтной передачи
    for (int i = 0; i < sizeof(Modul2Data_send) - 4; i++)
    {
        cheksum_send += adr_structura[i]; // Побайтно складываем все байты структуры кроме последних 4 в которых переменная в которую запишем результат
    }
    Modul2Data_send.cheksum = cheksum_send;
    // Modul2Data_send.cheksum = 0x0101;

    // DEBUG_PRINTF(" id= %0#6lX cheksum_send =  %0#6lX \n", Modul2Data_send.id, Modul2Data_send.cheksum);

    // Modul2Data_send.cheksum = measureCheksum_Modul2Data(Modul2Data_send); // Вычисляем контрольную сумму структуры и пишем ее значение в последний элемент

    // копировнаие данных из моей уже заполненной структуры в буфер для DMA
    memset(txBuffer, 0, sizeof(txBuffer));                                          // Очистка буфера
    struct Struct_Modul2Data *copy_txBuffer = (struct Struct_Modul2Data *)txBuffer; // Создаем переменную в которую пишем адрес буфера в нужном формате
    *copy_txBuffer = Modul2Data_send;                                               // Копируем данные

    // *******************************************************
    statusGetState = HAL_SPI_GetState(&hspi1);
    if (statusGetState == HAL_SPI_STATE_READY)

    {
        // DEBUG_PRINTF("SPI_GetState ok.");
        ;
    }
    else
        DEBUG_PRINTF("SPI_GetState ERROR %u ", statusGetState);

    // HAL_SPI_DMAStop(&hspi1);
    HAL_SPI_Abort(&hspi1);
    status = HAL_SPI_TransmitReceive_DMA(&hspi1, txBuffer, rxBuffer, BUFFER_SIZE); // // Перезапуск функции для следующего обмена// Запуск обмена данными по SPI с использованием DMA                                       // Копируем из структуры данные в пвмять начиная с адреса в котором начинаяется буфер для передачи
    if (status == HAL_OK)
    {
        // DEBUG_PRINTF("DMA OK \n");
        ;
    }
    else
    {
        DEBUG_PRINTF("DMA ERROR \n");
        statusGetState = HAL_SPI_GetState(&hspi1);
        if (statusGetState == HAL_SPI_STATE_READY)
            DEBUG_PRINTF("2SPI готов к передаче данных.\n");
        else
            DEBUG_PRINTF("2HAL_SPI_GetState ERROR %u \n", statusGetState);
    }
    // *******************************************************
    */
}

// Отработка пришедших команд. Изменение скорости, траектории и прочее
void executeDataReceive()
{
    /*
    // DEBUG_PRINTF("executeDataReceive... motor= %u laser= %u ", modeControlMotor, modeControlLaser);
    // DEBUG_PRINTF("in... motor= %lu laser= %lu \r\n", Data2Modul_receive.controlMotor.mode, Data2Modul_receive.controlLaser.mode);
    // Команда УПРАВЛЕНИЯ УГЛАМИ
    if (Data2Modul_receive.controlMotor.mode == 0) // Если пришла команда 0 Управления
    {
        modeControlMotor = 0; // Запоминаем в каком режиме Motor
        // Ничего не делаем
    }
    if (Data2Modul_receive.controlMotor.mode == 1) // Если пришла команда 1 Управления
    {
        modeControlMotor = 1; // Запоминаем в каком режиме Motor
        for (int i = 0; i < 4; i++)
        {
            // DEBUG_PRINTF("executeDataReceive = %i status = %i \r\n",Data2Modul_receive.controlMotor.mode,motor[i].status);
            setMotorAngle(i, Data2Modul_receive.controlMotor.angle[i]);
            // DEBUG_PRINTF("status = %i \r\n", motor[i].status);
        }
    }
    // Команда КОЛИБРОВКИ И УСТАНОВКИ В 0
    if (Data2Modul_receive.controlMotor.mode == 9 && modeControlMotor != 9) // Если пришла команда 9 Колибровки и предыдущая была другая
    {
        modeControlMotor = 9;  // Запоминаем в каком режиме Motor
        setMotor10();          // Отводим мотор на 10 градусов
        timerMode9 = millis(); // Запоминаем время начал
        flagMode9 = true;      // что мы начали режим колибровки
    }
    // Команда ВКЛЮЧЕНИЯ ЛАЗЕРНЫХ ДАТЧИКОВ
    if (Data2Modul_receive.controlLaser.mode == 1 && modeControlLaser != 1) // Если пришла команда и предыдущая была другая
    {
        modeControlLaser = 1; // Запоминаем в каком режиме Лазер
#ifdef LASER80
        // Непрерывное измерение
        laser80_continuousMeasurement(0); // Данные пойдут только через 500 милисекунд
        laser80_continuousMeasurement(1); // Данные пойдут только через 500 милисекунд
        laser80_continuousMeasurement(2); // Данные пойдут только через 500 милисекунд
        laser80_continuousMeasurement(3); // Данные пойдут только через 500 милисекунд
#endif
#ifdef LASER60
        sk60plus_startContinuousSlow(0);
        sk60plus_startContinuousSlow(1);
        sk60plus_startContinuousSlow(2);
        sk60plus_startContinuousSlow(3);
#endif
    }
    if (Data2Modul_receive.controlLaser.mode == 2 && modeControlLaser != 2) // Если пришла команда и находимся не в этом режиме
    {
        modeControlLaser = 2; // Запоминаем в каком режиме Лазер
#ifdef LASER60
        sk60plus_startContinuousAuto(0);
        sk60plus_startContinuousAuto(1);
        sk60plus_startContinuousAuto(2);
        sk60plus_startContinuousAuto(3);
#endif
    }
    // Команда ВЫЛЮЧЕНИЯ ЛАЗЕРНЫХ ДАТЧИКОВ
    if (Data2Modul_receive.controlLaser.mode == 0 && modeControlLaser != 0) // Если пришла команда и предыдущая была другая
    {
        modeControlLaser = 0; // Запоминаем в каком режиме Лазер
#ifdef LASER80
        laser80_stopMeasurement(0);
        laser80_stopMeasurement(1);
        laser80_stopMeasurement(2);
        laser80_stopMeasurement(3);
#endif
#ifdef LASER60
        sk60plus_stopContinuous(0);
        sk60plus_stopContinuous(1);
        sk60plus_stopContinuous(2);
        sk60plus_stopContinuous(3);
#endif
    }
    */
}

// Отработка действий по обмену по шине SPI
void workingSPI()
{
    //----------------------------- По факту обмена данными с верхним уровнем --------------------------------------
#ifdef SPI_protocol
    if (flag_data) // Если обменялись данными
    {
        // HAL_GPIO_WritePin(Analiz2_GPIO_Port, Analiz2_Pin, GPIO_PIN_SET); // Инвертирование состояния выхода.
        flag_data = false;
        flagTimeOut = true; // Флаг для выключения по таймауту
        timeSpi = millis(); // Запоминаем время обмена
        // DEBUG_PRINTF ("In = %#x %#x %#x %#x \r\n",rxBuffer[0],rxBuffer[1],rxBuffer[2],rxBuffer[3]);
        // DEBUG_PRINTF ("Out = %#x %#x %#x %#x \r\n",txBuffer[0],txBuffer[1],txBuffer[2],txBuffer[3]);
        // DEBUG_PRINTF("+\n");
        // processingDataReceive(); // Обработка пришедших данных после состоявшегося обмена  !!! Подумать почему меняю данные даже если они с ошибкой, потом по факту когда будет все работать
        // DEBUG_PRINTF(" mode= %i \n",Data2Modul_receive.controlMotor.mode);
        // executeDataReceive(); // Выполнение пришедших команд

        // DEBUG_PRINTF(" Receive id= %i cheksum= %i command= %i ", Data2Modul_receive.id, Data2Modul_receive.cheksum,Data2Modul_receive.command );
        // DEBUG_PRINTF("start = ");
        // for (int i = 0; i < sizeof(txBuffer); i++)
        // {
        //     DEBUG_PRINTF(" %x", txBuffer[i]);
        // }
        // DEBUG_PRINTF("\n");
        // collect_Data_for_Send(); // Собираем данные в структуре для отправки на момент прихода команлы, но БЕЗ учета команды.До исполнения команды.

        // DEBUG_PRINTF(" angle0= %.2f angle1= %.2f angle2= %.2f angle3= %.2f", Data2Modul_receive.angle[0], Data2Modul_receive.angle[1], Data2Modul_receive.angle[2], Data2Modul_receive.angle[3] );

        // spi_slave_queue_Send();  // Закладываем данные в буфер для передачи(обмена)

        // DEBUG_PRINTF("end   = ");
        // for (int i = 0; i < sizeof(txBuffer); i++)
        // {
        //     DEBUG_PRINTF(" %x", txBuffer[i]);
        // }
        // DEBUG_PRINTF("-----\n");
        // HAL_GPIO_WritePin(Analiz2_GPIO_Port, Analiz2_Pin, GPIO_PIN_RESET); // Инвертирование состояния выхода.
    }
#endif
}

// Заполнение данными Прошивки
void initFirmware()
{
    /*
    Modul2Data_send.firmware.gen = 1;
    Modul2Data_send.firmware.ver = 22;
    Modul2Data_send.firmware.debug = DEBUG;
#ifdef LASER60
    Modul2Data_send.firmware.laser = 60;
#endif
#ifdef LASER80
    Modul2Data_send.firmware.laser = 80;
#endif
    Modul2Data_send.firmware.motor = STEPMOTOR;
    */
}
#endif /*CODE_H*/
