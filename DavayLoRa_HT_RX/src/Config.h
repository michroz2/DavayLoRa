/**
 * @file Config.h
 * @version 1.53
 * @brief Глобальные настройки и работа с NVS (RX)
 */
 #ifndef CONFIG_H
 #define CONFIG_H
 
 #include <Arduino.h>
 #include <Preferences.h>
 
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
 };
 #pragma pack(pop)
 
 // Глобальный объект Preferences (нужен в main.cpp для CMD_CYCLE_EXEC)
 extern Preferences preferences;
 
 // Локальные переменные RX
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
 
 extern unsigned long wakeUpHoldTime;     
 extern unsigned long wakeUpReleaseWindow; 
 
 extern bool enableBigLed;             
 extern bool enableBuzzer;            
 
 // Прототипы
 void loadConfig();
 void saveConfigFromPacket(ConfigPacket* p);
 
 #endif