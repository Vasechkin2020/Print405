#ifndef CONFIG_H
#define CONFIG_H

enum codeOperation // коды операций запросов по UART
{
  No,           // нет операций
  Stop,         // стоп измерения
  Continuous,   // Непрерывное измерение
  Single,       // Одиночное измерние
  Address,      // Установка адреса
  Cache,        // запрос кэш тз датчика
  Status,       // Включение отключение адтчика
  Broadcast,    // Запрос на измерение ко всем датчикам на линии, потом считать из кэша
  StartPoint,   // Установка точки отсчета
  TimeInterval, // Установка интревала вывода при постоянных измерениях. Возможно это усреднение значений ???
  SetRange,
  SetResolution,
  SetFrequency
};

//*********************************************************************
struct motorStruct // Структура для локального управления и сбора данных по моторам
{
  int status;                // Передаются импульсы на мотор или нет в данный момент, вращается или нет
  int position;              // Текущая позиция в импульсах
  int destination;           // Цель назначение в позиции в импульсах
  int dir;                   // Направление вращения мотора 1 - по часовой 0 - против часовой
  GPIO_TypeDef *dir_port;    // Пин определяющий направление вращения
  uint16_t dir_pin;          // Пин определяющий направление вращения
  GPIO_TypeDef *step_port;   // Пин определяющий импульс
  uint16_t step_pin;         // Пин определяющий импульс
  GPIO_TypeDef *micric_port; // Пин определяющий концевик
  int micric_pin;            // Пин определяющий концевик
};
struct motorStruct motor[4]; // Все локальные данные по моторам

//*********************************************************************
struct SControlLaser
{
  uint32_t mode; // Текущий режим работы 0 - отключить датчики 1 - режим одновременного измерения
};
struct SControlMotor
{
  uint32_t mode;        // Текущий режим работы 0 - режим колибровки концевиков 1 - обычное управление моторами по углу
  float angle[4];       // Углы в которые нужно повернультя в локальной системе
  int32_t numPillar[4]; // Номер столба до которого измеряем расстояние
};

// Структура получаемых данных от Data к контроллеру Modul
struct Struct_Data2Modul
{
  uint32_t id;                       // Номер команды по порядку
  struct SControlMotor controlMotor; // Управление моторами
  struct SControlLaser controlLaser; // Управление лазерами
  uint32_t cheksum;                  // Контрольная сумма данных в структуре
};

struct Struct_Data2Modul Data2Modul_receive; // Экземпляр структуры получаемых данных

// const int size_structura_receive1 = sizeof(Data2Modul_receive); // Размер структуры с данными которые получаем

//*********************************************************************
// Структура по состоянию лидаров которая передается на верхний уровень

struct SFirmware
{
  uint8_t gen;   // Поколение
  uint8_t ver;   // Версия
  uint8_t laser; // Вариант использования лазеров
  uint8_t debug; // Вариант использования отладки для вывода в printf
  float motor;   // Вариант использования моторов сколько шагов на оборот 200 или 400 (1,8 градуса или 0,9 градусов на шаг)

};

struct SLaserSend
{
  uint32_t status;        // Статус датчика или ошибки
  float distance;         // Последнее измерение
  uint32_t signalQuality; // Качество сигнала
  float angle;            // Положение при последнем измерении
  uint32_t time;          // Время измерения от начала запуска программы
  float rate;             // Частота работы датчика
  int32_t numPillar;      // Номер столба до которого измерили расстояние
};
struct SLaserSend laser[4]; // Все локальные данные по лазерам

struct SMotorSend // Структура которая передается на верхний уровень
{
  int32_t status;    // Передаются импульсы на мотор или нет в данный момент, вращается или нет
  float position;    // Текущая позиция в градусах
  float destination; // Цель назначение в позиции в градусах
};

// Структура по обратной связи по обмену по шине SPI
struct SSpi
{
  uint32_t all;
  uint32_t bed;
};
struct SSpi spi; // Переменная где все данные по обмену

struct SXyz //
{
  float x;
  float y;
  float z;
};

struct SMpu // Структура с данными со всех датчиков, отправляем наверх
{
  int32_t status; // статус состояния
  float rate;     // частота работы датчика
  struct SXyz angleEuler;
  struct SXyz linear;
};

struct SMpu bno055; // Данные с датчика BNO055

// Структура в которой все главные переменные передаюся на высокий уровень от Modul к Data
struct Struct_Modul2Data
{
  uint32_t id; // id команды
  struct SFirmware firmware;  // Версия прошики и использованного оборудования
  uint32_t pinMotorEn;        // Стутус пина управления драйвером моторов, включен драйвер или нет
  struct SMotorSend motor[4]; // Структура по состоянию моторов
  struct SLaserSend laser[4]; // Структура по состоянию лазеров
  uint32_t micric[4];         // Структура по состоянию концевиков
  struct SMpu bno055;         // Данные с датчика BNO055
  struct SSpi spi;            // Структура по состоянию обмена по шине SPI

  uint32_t cheksum; // Контрольная сумма данных в структуре
};

struct Struct_Modul2Data Modul2Data_send; // Тут все переменные его характеризующие на низком уровне
// const uint16_t size_structura_send2 = sizeof(Modul2Data_send);                                                                       // Размер структуры с данными которые передаем
//const uint16_t max_size_stuct = (size_structura_receive < size_structura_send) ? size_structura_send : size_structura_receive; // Какая из структур больше

#endif
