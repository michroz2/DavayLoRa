/**
 * @file RadioComm.h
 * @version 1.53
 * @brief Модуль радиосвязи SX1262 и протокол (TX)
 * Описание: Отвечает за прием и передачу пакетов, содержит коды команд протокола.
 */
 #ifndef RADIOCOMM_H
 #define RADIOCOMM_H
 
 #include <Arduino.h>
 #include <RadioLib.h>
 #include <SPI.h>
 
 // --- Макросы команд (Протокол связи v1.53) ---
 #define CMD_SIGNAL         208  // Передача состояния главной кнопки
 #define CMD_SIGNAL_OK      209  // Подтверждение приема сигнала
 #define CMD_PING           212  // Контроль качества связи (Keep-alive)
 #define CMD_PING_OK        213  // Подтверждение пинга
 #define CMD_SLEEP          214  // Централизованная команда на засыпание
 #define CMD_SLEEP_OK       215  // Подтверждение сна
 #define CMD_CONFIG         216  // Синхронный переход в режим настройки (Web)
 #define CMD_CONFIG_OK      217  // Подтверждение режима настройки
 #define CMD_NORMAL_MODE    218  // Принудительный возврат в рабочий режим
 #define CMD_NORMAL_MODE_OK 219  // Подтверждение возврата
 #define CMD_SYNC_CONFIG    220  // Отправка структуры с настройками на RX
 #define CMD_SYNC_CONFIG_OK 221  // Успешная синхронизация настроек
 #define CMD_REBOOT         222  // Приказ на немедленную перезагрузку приемника
 #define CMD_EXEC_CONFIG    223  // Переход в режим изменения сигналов на лету
 #define CMD_EXEC_CONFIG_OK 224  // Подтверждение режима изменения сигналов
 #define CMD_CYCLE_EXEC     225  // Команда переключения исполнительных устройств (свет/вибро)
 #define CMD_CYCLE_EXEC_OK  226  // Подтверждение переключения
 
 #define WORK_FREQUENCY 434E6
 #define MAX_ADDRESS 20
 #define DEFAULT_TURNAROUND 300     
 #define WORK_COMM_ATTEMPTS 3       
 
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
 extern byte sndCmd;
 extern byte sndData;
 extern bool wasReceived;
 extern byte cmdExpected;
 extern byte rcvAddress;
 extern byte rcvCmd;
 extern byte rcvData;
 extern unsigned long workFrequency;
 
 extern long lastSendTime;
 extern int lastRSSI;
 extern float lastSNR;
 extern unsigned long lastTurnaround;
 extern long lastFrequencyError;
 
 extern volatile bool receivedFlag; 
 
 extern unsigned long workingFrequency[MAX_ADDRESS];
 
 // Внешние переменные из main.cpp
 extern unsigned long pingTimer;
 
 // --- Прототипы ---
 void setFlag(void);
 void transmitPacket(byte* payload, size_t size);
 void sendMessage(byte msgCmd, byte sndData);
 void checkReceive();
 bool commSession(byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes);
 bool syncConfigToRX();
 void setLoRaParams();
 void onReceive(byte* payload, int packetSize);
 
 #endif