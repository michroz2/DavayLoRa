/**
 * @file Config.h
 * @version 1.64
 * @brief Глобальные настройки и работа с энергонезависимой памятью NVS (RX)
 * Описание: Хранит параметры приемника и принимает новые конфигурации от пульта.
 */
 #ifndef CONFIG_H
 #define CONFIG_H
 
 #include <Arduino.h>
 #include <Preferences.h>
 
 // Структура пакета настроек, приходящая от TX при синхронизации (CMD_SYNC_CONFIG)
 #pragma pack(push, 1)
 struct ConfigPacket {
    byte workAddress;
    bool measurebattery;
    unsigned long batteryPeriod;
    unsigned long wakeUpHoldTime;
    unsigned long wakeUpReleaseWindow;
    unsigned long stuckSleepTime;
    unsigned long configTimeout;
    unsigned long sleepLedDuration;
    
    bool rxEnableBigLed;
    int rxPwmledBrightness;
    bool rxEnableBuzzer;
    int rxBuzzerVolume;
    unsigned long rxCutoffTime;
    unsigned long pingTimeoutRX;
    int maxPower; // Принимаем ограничение мощности от TX
 };
 #pragma pack(pop)
 
 // Глобальный объект Preferences (нужен в main.cpp для CMD_CYCLE_EXEC)
 extern Preferences preferences;
 
 // Локальные переменные RX (активные настройки)
 extern byte workAddress;                 
 extern int pwmledBrightness;            
 extern int buzzerVolume;               
 extern unsigned long cutoffTime;      
 extern unsigned long pingTimeout;     
 extern unsigned long stuckSleepTime; 
 extern bool measurebattery;           
 extern unsigned long batteryPeriod;    
 extern unsigned long sleepLedDuration;   
 extern unsigned long configTimeout;
 extern int maxPower; // Лимит мощности передачи
 
 extern unsigned long wakeUpHoldTime;     
 extern unsigned long wakeUpReleaseWindow; 
 
 // Флаги активных исполнительных устройств (Свет / Вибрация)
 extern bool enableBigLed;             
 extern bool enableBuzzer;            
 
 // Прототипы
 void loadConfig();
 void saveConfigFromPacket(ConfigPacket* p);
 
 #endif