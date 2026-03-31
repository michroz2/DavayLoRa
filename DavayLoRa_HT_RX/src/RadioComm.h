/**
 * @file RadioComm.h
 * @version 1.53
 * @brief Модуль радиосвязи SX1262 и протокол (RX)
 * Описание: Слушает эфир, фильтрует пакеты по адресу и транслирует команды в main.cpp.
 */
 #ifndef RADIOCOMM_H
 #define RADIOCOMM_H
 
 #include <Arduino.h>
 #include <RadioLib.h>
 #include <SPI.h>
 
 // --- Макросы команд (Протокол связи v1.53) ---
 #define CMD_SIGNAL         208  // Сигнал нажатия кнопки (Action!)
 #define CMD_SIGNAL_OK      209
 #define CMD_PING           212  // Пинг от пульта
 #define CMD_PING_OK        213
 #define CMD_SLEEP          214  // Приказ уснуть
 #define CMD_SLEEP_OK       215
 #define CMD_CONFIG         216  // Переход в режим настройки Wi-Fi
 #define CMD_CONFIG_OK      217
 #define CMD_NORMAL_MODE    218  // Возврат в рабочий режим
 #define CMD_NORMAL_MODE_OK 219
 #define CMD_SYNC_CONFIG    220  // Прием длинного пакета с настройками
 #define CMD_SYNC_CONFIG_OK 221
 #define CMD_REBOOT         222  // Перезагрузка
 #define CMD_EXEC_CONFIG    223  // Переход в режим выбора свето/вибро
 #define CMD_EXEC_CONFIG_OK 224
 #define CMD_CYCLE_EXEC     225  // Смена свето/вибро по кругу
 #define CMD_CYCLE_EXEC_OK  226
 
 #define WORK_FREQUENCY 434E6
 #define MAX_ADDRESS 20
 
 // --- Пины и объекты ---
 extern const int sckPin;
 extern const int misoPin;
 extern const int mosiPin;
 extern const int csPin;
 extern const int resetPin;
 extern const int irqPin;
 extern const int busyPin;
 
 extern SX1262 radio;
 
 // --- Переменные радиообмена ---
 extern byte rcvAddress;
 extern byte rcvCmd;
 extern byte rcvData;
 extern unsigned long workFrequency;
 extern unsigned long lastSendTime;
 extern volatile bool receivedFlag;
 extern unsigned long workingFrequency[MAX_ADDRESS];
 
 // Внешние переменные из main.cpp
 extern unsigned long pingTimeOutLastTime;
 
 // --- Прототипы ---
 void setFlag(void);
 void setLoRaParams();
 void transmitPacket(byte* payload, size_t size);
 void sendMessage(byte msgAddr, byte msgCmd, byte msgData);
 void checkReceive();
 void onReceive(byte* payload, int packetSize);
 
 #endif