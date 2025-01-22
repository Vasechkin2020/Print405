#ifndef CODE_H
#define CODE_H

#define SPI_protocol yes

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "slaveSPI.h"
#include "gim43xx.h"

//********************************* ПЕРЕМЕННЫЕ ***************************************************************************

bool flagCAN = false;
uint32_t timeCAN = 0;

GPIO_TypeDef *myPort;

void timer6();       // Обработчик прерывания таймера TIM6	1 раз в 1 милисекунду
void timer7();       // Обработчик прерывания таймера TIM7	1 раз в 1 милисекунду
void workingTimer(); // Отработка действий по таймеру в 1, 50, 60 милисекунд
void workingSPI();   // Отработка действий по обмену по шине SPI
void workingCAN();   // Отработка действий по шине CAN
void initFirmware(); // Заполнение данными прошивки

HAL_StatusTypeDef status;
HAL_SPI_StateTypeDef statusGetState;

bool flagTimeOut = true;       // Флаг таймаута при обрыве связи по SPI
bool flagCallBackUart = false; // Флаг для указания нужно ли отрабатывать в колбеке  или обраьотка с самой функции

extern volatile uint64_t millisCounter;
extern struct SGim43 dataGim43;
//********************************* ФУНКЦИИ ***************************************************************************

void timer7() // Обработчик прерывания таймера TIM7
{
}
void timer6() // Обработчик прерывания таймера TIM6	1 раз в 1 милисекунду
{
    millisCounter++; // Увеличиваем счетчик миллисекунд
}
// Функция для возврата количества миллисекунд
uint32_t millis()
{
    return millisCounter;
}

// Собираем нужные данные и пишем в структуру на отправку
void collect_Data_for_Send()
{
    Print2Data_send.id++;
    // Print2Data_send.firmware  Заполняем при старете
    Print2Data_send.spi = spi;
    Print2Data_send.gim43 = dataGim43;

    uint32_t cheksum_send = 0;                                          // Считаем контрольную сумму отправляемой структуры
    unsigned char *adr_structura = (unsigned char *)(&Print2Data_send); // Запоминаем адрес начала структуры. Используем для побайтной передачи
    for (int i = 0; i < sizeof(Print2Data_send) - 4; i++)
    {
        cheksum_send += adr_structura[i]; // Побайтно складываем все байты структуры кроме последних 4 в которых переменная в которую запишем результат
    }
    Print2Data_send.cheksum = cheksum_send;

    // Print2Data_send.cheksum = 0x1A1B1C1D;
    // DEBUG_PRINTF(" id= %0#6lX cheksum_send =  %0#6lX \n", Print2Data_send.id, Print2Data_send.cheksum);
    // Print2Data_send.cheksum = measureCheksum_Print2Data(Print2Data_send); // Вычисляем контрольную сумму структуры и пишем ее значение в последний элемент

    // копировнаие данных из моей уже заполненной структуры в буфер для DMA
    memset(txBuffer, 0, sizeof(txBuffer));                                          // Очистка буфера
    struct Struct_Print2Data *copy_txBuffer = (struct Struct_Print2Data *)txBuffer; // Создаем переменную в которую пишем адрес буфера в нужном формате
    *copy_txBuffer = Print2Data_send;                                               // Копируем данные

    // *******************************************************
    statusGetState = HAL_SPI_GetState(&hspi1);
    if (statusGetState == HAL_SPI_STATE_READY)
    {
        // DEBUG_PRINTF("SPI_GetState ok.\n");
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
}

// Отработка пришедших команд. Исполнение.
void executeDataReceive()
{
    DEBUG_PRINTF("--- executeDataReceive... mode= %lu status= %lu \n", Data2Print_receive.controlPrint.mode, Data2Print_receive.controlPrint.status);
    static uint32_t statusPred = 0; // Предыдущий статус
    //  0 - Выполняем команды по status 1- посылаем на CAN данные по position, velocity, torque
    if (Data2Print_receive.controlPrint.mode == 0 && !flagCAN) // Если режим работы по командам и не взведен флаг что исполняем команду
    {
        if (Data2Print_receive.controlPrint.status == 0 && statusPred != Data2Print_receive.controlPrint.status) // Если статус поменялся
        {
            setData(0, 0, 0, 0, -1.5, buffCAN); // Отводим маркер быстро и запускаем флаг что надо остановить обратное движение через несколько милисекунд
            // CAN_SendMessage(zero, 8);     // Отправляем данные
            // memcpy(buffCAN, stop, 8);   // Копируем 8 байт из массива в буфер
            flagCAN = true;
            timeCAN = millis();
            DEBUG_PRINTF("++++++ mode 0 \n");
        }
        if (Data2Print_receive.controlPrint.status == 1)
        {
            setData(0, 0, 0, 0, Data2Print_receive.controlPrint.torque, buffCAN); // Давим с определенным моментом пока не будет команды отмены.
            DEBUG_PRINTF("======= mode 1 \n");
            // CAN_SendMessage(zero, 8);     // Отправляем данные
        }
        statusPred = Data2Print_receive.controlPrint.status;
    }
}

// Отработка действий по шине CAN
void workingCAN()
{
    if (flagCAN && millis() > timeCAN + 50) // Если есть флаг и прогло более милиисекунд то сбрасываем флаг и исполняем
    {
        flagCAN = false;
        memcpy(buffCAN, zero, 8);   // Копируем 8 байт из массива в буфер
        DEBUG_PRINTF("*** mode NEW \n");
        // CAN_SendMessage(stop, 8); // Останавливаем мотор
    }
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
        flagTimeOut = true;                           // Флаг для выключения по таймауту
        timeSpi = millis();                           // Запоминаем время обмена
        HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin); // Инвертирование состояния выхода.
        // DEBUG_PRINTF ("In = %#x %#x %#x %#x \r\n",rxBuffer[0],rxBuffer[1],rxBuffer[2],rxBuffer[3]);
        // DEBUG_PRINTF ("Out = %#x %#x %#x %#x \r\n",txBuffer[0],txBuffer[1],txBuffer[2],txBuffer[3]);
        // DEBUG_PRINTF("+\n");
        processingDataReceive(); // Обработка пришедших данных после состоявшегося обмена  !!! Подумать почему меняю данные даже если они с ошибкой, потом по факту когда будет все работать
        // DEBUG_PRINTF(" mode= %i \n",Data2Print_receive.controlMotor.mode);
        executeDataReceive(); // Выполнение пришедших команд

        // DEBUG_PRINTF(" Receive id= %i cheksum= %i command= %i ", Data2Print_receive.id, Data2Print_receive.cheksum,Data2Print_receive.command );
        // DEBUG_PRINTF("start = ");
        // for (int i = 0; i < sizeof(txBuffer); i++)
        // {
        //     DEBUG_PRINTF(" %x", txBuffer[i]);
        // }
        // DEBUG_PRINTF("\n");
        collect_Data_for_Send(); // Собираем данные в структуре для отправки на момент прихода команлы, но БЕЗ учета команды.До исполнения команды.

        // DEBUG_PRINTF(" angle0= %.2f angle1= %.2f angle2= %.2f angle3= %.2f", Data2Print_receive.angle[0], Data2Print_receive.angle[1], Data2Print_receive.angle[2], Data2Print_receive.angle[3] );

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
    Print2Data_send.firmware.gen = 1;
    Print2Data_send.firmware.ver = 2;
    Print2Data_send.firmware.debug = DEBUG;
    Print2Data_send.firmware.test = 0x1A;
    printf("Firmware gen %hu ver %hu debug %hu\n", Print2Data_send.firmware.gen, Print2Data_send.firmware.ver, Print2Data_send.firmware.debug);
}

// Исполнение с периодичностью
void time_DataGim43(uint32_t time_)
{
    static uint32_t time = 0; //
    if ((millis() - time) >= time_)
    {
        // printf("timeCAN %lu msec \n", millis());
        DEBUG_PRINTF("Gim43 position = %.2f velocity = %.2f torque = %.2f \n", dataGim43.position, dataGim43.velocity, dataGim43.torque);
        time = millis();
    }
}

void time_CAN(uint32_t time_)
{
    static uint32_t time = 0; //
    if ((millis() - time) >= time_)
    {
        CAN_SendMessage(buffCAN, 8); // 
        // printf("timeCAN %lu msec \n", millis());
        time = millis();
    }
}

void time_LED(uint32_t time_)
{
    static uint32_t time = 0; //
    if ((millis() - time) >= 1000)
    {
      setData(0, 0, 0, 0, 0, buffCAN); // Давим с определенным моментом пока не будет команды отмены.
      printf("print_modul %lu msec | spi.all = %lu spi.bed= %lu | \n", millis(),spi.all,spi.bed);
      time = millis();
    }
}

#endif /*CODE_H*/
