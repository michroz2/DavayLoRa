#include "Config.h"

// Локальные макросы отладки
#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.print(x)
#define DEBUGln(x) Serial.println(x)
#else
#define DEBUG(x)
#define DEBUGln(x)
#endif

ConfigPacket rxSettings;
Preferences preferences;

byte workAddress = 4;                 
bool measurebattery = true;           
unsigned long batteryPeriod = 300000;    
unsigned long wakeUpHoldTime = 2000;     
unsigned long wakeUpReleaseWindow = 2000; 
unsigned long stuckSleepTime = 10000; 
unsigned long configTimeout = 600000; 
unsigned long sleepLedDuration = 2000;   

int pwmledBrightness = 35;            
int fbledBrightness = 255;            
unsigned long pingTimeout = 3000;     
unsigned long bigTimeout = 3600000;   
unsigned long execTimeout = 30000;    

unsigned long pingTimeoutRX = 9000;   

void loadConfig() {
   DEBUGln(F("--- Loading config from NVS ---"));
   preferences.begin("davaylora", false); 
   
   workAddress = preferences.getUChar("workAddress", 4);
   measurebattery = preferences.getBool("measureBat", true);
   batteryPeriod = preferences.getULong("batPeriod", 300000);
   wakeUpHoldTime = preferences.getULong("wkUpHold", 2000);
   wakeUpReleaseWindow = preferences.getULong("wkUpRel", 2000);
   stuckSleepTime = preferences.getULong("stuckSleep", 10000);
   configTimeout = preferences.getULong("confTo", 600000);
   sleepLedDuration = preferences.getULong("sleepLedDur", 2000);
   
   pwmledBrightness = preferences.getInt("bigLedBright", 35);
   fbledBrightness = preferences.getInt("fbLedBright", 255);
   pingTimeout = preferences.getULong("pingTimeout", 3000);
   bigTimeout = preferences.getULong("bigTimeout", 3600000);
   execTimeout = preferences.getULong("execTo", 30000); 
   
   pingTimeoutRX = preferences.getULong("pingRx", 9000); 
   rxSettings.rxEnableBigLed = preferences.getBool("rxEnBigLed", true);
   rxSettings.rxPwmledBrightness = preferences.getInt("rxBigBright", 35);
   rxSettings.rxEnableBuzzer = preferences.getBool("rxEnBuzzer", false);
   rxSettings.rxBuzzerVolume = preferences.getInt("rxBuzVol", 255);
   rxSettings.rxCutoffTime = preferences.getULong("rxCutoff", 2000);

   preferences.end();
   DEBUGln(F("Config loaded."));
}

void saveConfig() {
   DEBUGln(F("--- Saving config to NVS ---"));
   preferences.begin("davaylora", false);
   
   preferences.putUChar("workAddress", workAddress);
   preferences.putBool("measureBat", measurebattery);
   preferences.putULong("batPeriod", batteryPeriod);
   preferences.putULong("wkUpHold", wakeUpHoldTime);
   preferences.putULong("wkUpRel", wakeUpReleaseWindow);
   preferences.putULong("stuckSleep", stuckSleepTime);
   preferences.putULong("confTo", configTimeout);
   preferences.putULong("sleepLedDur", sleepLedDuration);
   
   preferences.putInt("bigLedBright", pwmledBrightness);
   preferences.putInt("fbLedBright", fbledBrightness);
   preferences.putULong("pingTimeout", pingTimeout);
   preferences.putULong("bigTimeout", bigTimeout);
   preferences.putULong("execTo", execTimeout); 
   
   preferences.putULong("pingRx", pingTimeoutRX); 
   preferences.putBool("rxEnBigLed", rxSettings.rxEnableBigLed);
   preferences.putInt("rxBigBright", rxSettings.rxPwmledBrightness);
   preferences.putBool("rxEnBuzzer", rxSettings.rxEnableBuzzer);
   preferences.putInt("rxBuzzerVolume", rxSettings.rxBuzzerVolume);
   preferences.putULong("rxCutoff", rxSettings.rxCutoffTime);
   
   preferences.end();
   DEBUGln(F("Config saved."));
}