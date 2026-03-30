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

extern ConfigPacket rxSettings;
extern Preferences preferences;

// --- Системные и общие переменные ---
extern byte workAddress;                 
extern bool measurebattery;           
extern unsigned long batteryPeriod;    
extern unsigned long wakeUpHoldTime;     
extern unsigned long wakeUpReleaseWindow; 
extern unsigned long stuckSleepTime; 
extern unsigned long configTimeout; 
extern unsigned long sleepLedDuration;   

// --- Настройки пульта (TX) ---
extern int pwmledBrightness;            
extern int fbledBrightness;            
extern unsigned long pingTimeout;     
extern unsigned long bigTimeout;   
extern unsigned long execTimeout;    

// --- Настройки приемника (для синхронизации) ---
extern unsigned long pingTimeoutRX;   

// Прототипы
void loadConfig();
void saveConfig();

#endif