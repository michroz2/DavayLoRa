/**
 * @file Config.cpp
 * @version 1.58 (RX: Исправление NVS бага 16 символов для длительности прощального мигания)
 * @brief Реализация загрузки и сохранения настроек (RX)
 */
 #include "Config.h"

 // Локальные макросы отладки
 #define DEBUG_ENABLE // Логирование ВКЛЮЧЕНО
 #ifdef DEBUG_ENABLE
 #define DEBUG(x) Serial.print(x)
 #define DEBUGln(x) Serial.println(x)
 #else
 #define DEBUG(x)
 #define DEBUGln(x)
 #endif
 
 Preferences preferences;
 
 byte workAddress = 4;                 
 int pwmledBrightness = 35;            
 int buzzerVolume = 255;               
 unsigned long cutoffTime = 2000;      
 unsigned long pingTimeout = 9000;     
 unsigned long stuckSleepTime = 10000; 
 bool measurebattery = true;           
 unsigned long batteryPeriod = 300000;    
 unsigned long sleepLedDuration = 2000;   
 unsigned long configTimeout = 600000;
 
 unsigned long wakeUpHoldTime = 2000;     
 unsigned long wakeUpReleaseWindow = 2000; 
 
 // По умолчанию включен только свет
 bool enableBigLed = true;             
 bool enableBuzzer = false;            
 
 // Загрузка сохраненных настроек при включении приемника
 void loadConfig() {
    DEBUGln(F("--- Loading config from NVS ---"));
    preferences.begin("davaylora", false);
    
    workAddress = preferences.getUChar("workAddress", 4);
    measurebattery = preferences.getBool("measureBat", true);
    pwmledBrightness = preferences.getInt("bigLedBright", 35);
    buzzerVolume = preferences.getInt("buzzerVol", 255);
    cutoffTime = preferences.getULong("cutoffTime", 2000);
    pingTimeout = preferences.getULong("pingTimeout", 9000);
    stuckSleepTime = preferences.getULong("stuckSleep", 10000);
    configTimeout = preferences.getULong("confTo", 600000);
    
    // ИСПРАВЛЕНИЕ: Используем короткий ключ "sleepLedDur" (менее 15 символов)
    sleepLedDuration = preferences.getULong("sleepLedDur", 2000);
    
    enableBigLed = preferences.getBool("enBigLed", true);
    enableBuzzer = preferences.getBool("enBuzzer", false);
    
    batteryPeriod = preferences.getULong("batPeriod", 300000);
    wakeUpHoldTime = preferences.getULong("wkUpHold", 2000);
    wakeUpReleaseWindow = preferences.getULong("wkUpRel", 2000);
    
    preferences.end();
    DEBUGln(F("Config loaded."));
 } // конец функции loadConfig
 
 // Сохранение новых настроек, когда пульт присылает их пакет по воздуху
 void saveConfigFromPacket(ConfigPacket* p) {
    DEBUGln(F("--- Saving received config struct to NVS ---"));
    DEBUG(F("workAddress: ")); DEBUGln(p->workAddress);
    DEBUG(F("rxPwmledBrightness: ")); DEBUGln(p->rxPwmledBrightness);
    DEBUG(F("rxBuzzerVolume: ")); DEBUGln(p->rxBuzzerVolume);
    
    preferences.begin("davaylora", false);
    
    preferences.putUChar("workAddress", p->workAddress);
    preferences.putBool("measureBat", p->measurebattery);
    preferences.putULong("batPeriod", p->batteryPeriod);
    preferences.putULong("wkUpHold", p->wakeUpHoldTime);
    preferences.putULong("wkUpRel", p->wakeUpReleaseWindow);
    preferences.putULong("stuckSleep", p->stuckSleepTime);
    preferences.putULong("confTo", p->configTimeout);
    
    // ИСПРАВЛЕНИЕ: Используем короткий ключ "sleepLedDur"
    preferences.putULong("sleepLedDur", p->sleepLedDuration);
    
    preferences.putBool("enBigLed", p->rxEnableBigLed);
    preferences.putInt("bigLedBright", p->rxPwmledBrightness);
    preferences.putBool("enBuzzer", p->rxEnableBuzzer);
    preferences.putInt("buzzerVol", p->rxBuzzerVolume);
    preferences.putULong("cutoffTime", p->rxCutoffTime);
    preferences.putULong("pingTimeout", p->pingTimeoutRX);
    
    preferences.end();
    DEBUGln(F("Config saved."));
 } // конец функции saveConfigFromPacket