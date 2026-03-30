/**
 * @file main.cpp (RX)
 * @version 1.42 (RX: Синхронизация версий и форматирования)
 * @brief ПОЛНЫЙ ИСХОДНЫЙ КОД ПРИЁМНИКА (DavayLoRa)
 * Особенности: Добавлено строгое комментирование одиночных фигурных скобок для соответствия TX.
 */

 #include <Arduino.h>
 #include <esp_sleep.h>
 #include <driver/rtc_io.h>
 #include <SPI.h>
 #include <RadioLib.h>
 #include <Preferences.h>
 
 Preferences preferences;
 
 // ======================= ГЛОБАЛЬНЫЕ НАСТРОЙКИ (СИНХРОНИЗАЦИЯ) =======================
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
 
 // Локальные переменные RX
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
 
 bool enableBigLed = true;             
 bool enableBuzzer = false;            
 
 bool isBatteryConnected = false; 
 
 // Пороги батареи
 #define BATTERY_MIN_VOLTAGE 3.5
 #define BATTERY_VOLTAGE_1 3.6
 #define BATTERY_VOLTAGE_2 3.7
 #define BATTERY_VOLTAGE_3 3.8
 #define BATTERY_VOLTAGE_4 3.9
 #define BATTERY_VOLTAGE_5 4.0
 
 // ======================= АППАРАТНАЯ КОНФИГУРАЦИЯ =======================
 
 // Распиновка Heltec V3
 #define PIN_SIGNAL_LED      41     
 #define PIN_SIGNAL_BUZZERS  42     
 #define PIN_REED            7      // Магнитный переключатель (геркон)
 #define PIN_USER            0      
 #define PIN_STATUS_LED      35     
 #define PIN_BATTERY_LED     35     
 #define PIN_BATTERY_INTERNAL 1     
 #define PIN_VEXT 36                
 #define PIN_ADC_CTRL 37            
 #define HELTEC_BATTERY_MULTIPLIER 4.9 
 
 const int sckPin = 9;
 const int misoPin = 11;
 const int mosiPin = 10;
 const int csPin = 8;
 const int resetPin = 12;
 const int irqPin = 14;
 const int busyPin = 13;
 
 SX1262 radio = new Module(csPin, irqPin, resetPin, busyPin);
 
 // ======================= ПРОТОКОЛ И ПЕРЕМЕННЫЕ =======================
 
 #define WORK_FREQUENCY 434E6
 #define MAX_ADDRESS 20
 
 #define DEBUG_ENABLE
 #ifdef DEBUG_ENABLE
 #define DEBUG(x) Serial.print(x)
 #define DEBUGln(x) Serial.println(x)
 #else
 #define DEBUG(x)
 #define DEBUGln(x)
 #endif
 
 #define EVERY_MS(x) \
    static uint32_t tmr;\
    bool flg = millis() - tmr >= (x);\
    if (flg) { tmr = millis(); }\
    if (flg)
 
 // Команды протокола
 #define CMD_SIGNAL         208
 #define CMD_SIGNAL_OK      209
 #define CMD_PING           212
 #define CMD_PING_OK        213
 #define CMD_SLEEP          214
 #define CMD_SLEEP_OK       215
 #define CMD_CONFIG         216
 #define CMD_CONFIG_OK      217
 #define CMD_NORMAL_MODE    218
 #define CMD_NORMAL_MODE_OK 219
 #define CMD_SYNC_CONFIG    220
 #define CMD_SYNC_CONFIG_OK 221
 #define CMD_REBOOT         222
 #define CMD_EXEC_CONFIG    223
 #define CMD_EXEC_CONFIG_OK 224
 #define CMD_CYCLE_EXEC     225
 #define CMD_CYCLE_EXEC_OK  226
 
 byte rcvAddress = 0;
 byte rcvCmd = 0;
 byte rcvData = 0;
 bool signalStatus;
 unsigned long workFrequency = WORK_FREQUENCY;
 
 unsigned long lastSendTime = 0;
 unsigned long pingTimeOutLastTime; // Таймер Failsafe
 unsigned long cutoffTimer = 0;
 
 volatile bool receivedFlag = false;
 
 // Состояния RX
 enum SystemState { STATE_NORMAL, STATE_CONFIG, STATE_EXEC_CONFIG };
 SystemState currentState = STATE_NORMAL;
 
 bool configBlinkActive = false;
 unsigned long configBlinkStartTime = 0;
 
 bool execBlinkActive = false;
 unsigned long execBlinkStartTime = 0;
 
 unsigned long workingFrequency[MAX_ADDRESS] = {
    434000000, 434120000, 434240000, 433820000, 433700000, 433940000, 434030000,
    434150000, 434270000, 433850000, 433730000, 433970000, 434060000, 434180000,
    433880000, 433760000, 434090000, 434210000, 433910000, 433790000,
 };
 
 // --- ПРОТОТИПЫ ---
 void loadConfig();
 void saveConfigFromPacket(ConfigPacket* p);
 void enterDeepSleep();
 void runWakeUpProtection(uint8_t wakeupPin);
 void processTimeOut();
 void processCommand();
 void processSignal();
 void processConfigLed();
 void processExecConfigLed();
 void processCutoff();
 void processUserButton();
 void goToSleep();
 void updateStatusLed(bool ledStatus);
 void flashStatusLEDOnce();
 void flashStatusLed(byte times);
 void transmitPacket(byte* payload, size_t size);
 void sendMessage(byte msgAddr, byte msgCmd, byte msgData);
 void setLoRaParams();
 void checkReceive();
 void onReceive(byte* payload, int packetSize);
 bool testBattery();
 bool batteryVoltageOK(byte tries);
 float batteryVoltage();
 void processBattery();
 void showBatteryVoltage();
 void showNoBattery();
 void flashBatteryLEDOnce();
 void flashLedBattery(byte times);
 void stopWorking();
 
 // ======================= NVS (ПАМЯТЬ) =======================
 
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
    sleepLedDuration = preferences.getULong("sleepLedDuration", 2000);
    
    enableBigLed = preferences.getBool("enBigLed", true);
    enableBuzzer = preferences.getBool("enBuzzer", false);
    
    batteryPeriod = preferences.getULong("batPeriod", 300000);
    wakeUpHoldTime = preferences.getULong("wkUpHold", 2000);
    wakeUpReleaseWindow = preferences.getULong("wkUpRel", 2000);
    
    preferences.end();
 } // конец функции loadConfig
 
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
    preferences.putULong("sleepLedDuration", p->sleepLedDuration);
    
    preferences.putBool("enBigLed", p->rxEnableBigLed);
    preferences.putInt("bigLedBright", p->rxPwmledBrightness);
    preferences.putBool("enBuzzer", p->rxEnableBuzzer);
    preferences.putInt("buzzerVol", p->rxBuzzerVolume);
    preferences.putULong("cutoffTime", p->rxCutoffTime);
    preferences.putULong("pingTimeout", p->pingTimeoutRX);
    
    preferences.end();
 } // конец функции saveConfigFromPacket
 
 // ======================= РАДИООБМЕН =======================
 
 #if defined(ESP8266) || defined(ESP32)
    ICACHE_RAM_ATTR
 #endif
 void setFlag(void) { 
    receivedFlag = true; 
 } // конец функции прерывания setFlag
 
 void setLoRaParams() {
    DEBUGln("[RADIO] setLoRaParams()");
    radio.setOutputPower(20);                     
    radio.setBandwidth(125.0);                    
    radio.setSpreadingFactor(8);                  
    radio.setCodingRate(5);                       
    radio.setPreambleLength(8);                   
    radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE); 
 } // конец функции setLoRaParams
 
 void transmitPacket(byte* payload, size_t size) {
    DEBUG(F("[RADIO] >>> TX Packet [Size: ")); DEBUG(size); DEBUG(F("]: "));
    for (size_t i = 0; i < size; i++) { DEBUG(payload[i]); DEBUG(F(" ")); }
    DEBUGln();
 
    int state = radio.transmit(payload, size);
    if (state != RADIOLIB_ERR_NONE) {
      DEBUG(F("[RADIO] >>> Transmit failed, code: ")); DEBUGln(state);
    } // конец проверки ошибки передачи
    
    lastSendTime = millis();
    pingTimeOutLastTime = lastSendTime; 
    receivedFlag = false; 
    radio.startReceive(); 
 } // конец функции transmitPacket
 
 void sendMessage(byte msgAddr, byte msgCmd, byte msgData) {
    byte payload[3] = {msgAddr, msgCmd, msgData}; 
    transmitPacket(payload, 3);                         
 } // конец функции sendMessage
 
 void checkReceive() {
    if (receivedFlag) {
      receivedFlag = false;
      byte payload[256];
      int state = radio.readData(payload, sizeof(payload));
      if (state == RADIOLIB_ERR_NONE) {
        onReceive(payload, radio.getPacketLength());           
      } // конец проверки успешного приема
      radio.startReceive();                       
    } // конец проверки флага прерывания
 } // конец функции checkReceive
 
 void onReceive(byte* payload, int packetSize) {
    DEBUG(F("[RADIO] <<< RX Packet [Size: ")); DEBUG(packetSize); DEBUG(F("]: "));
    for (int i = 0; i < packetSize; i++) { DEBUG(payload[i]); DEBUG(F(" ")); }
    DEBUGln();
 
    rcvAddress = payload[0];
    if (rcvAddress != workAddress) {
      DEBUGln(F("\t[!] Ignored: Wrong address"));
      return;
    } // конец проверки адреса
 
    if (packetSize == 3) {
      rcvCmd = payload[1];
      rcvData = payload[2];
      
      if (rcvCmd == CMD_REBOOT) {
        DEBUGln(F("\n[ACTION] !!! CMD_REBOOT RECEIVED. Restarting in 500ms !!!"));
        delay(500);
        ESP.restart();
      } // конец обработки команды перезагрузки
    } 
    else if (packetSize == (sizeof(ConfigPacket) + 2) && payload[1] == CMD_SYNC_CONFIG) {
      DEBUGln(F("\n[RADIO] <<< Received Config Struct from TX!"));
      
      ConfigPacket newSettings;
      memcpy(&newSettings, &payload[2], sizeof(ConfigPacket));
      
      saveConfigFromPacket(&newSettings);
      
      DEBUGln(F("[RADIO] Replying with CMD_SYNC_CONFIG_OK"));
      sendMessage(workAddress, CMD_SYNC_CONFIG_OK, 1);
      
      rcvCmd = 0; 
    } 
    else {
      DEBUGln(F("\t[!] Invalid packet size or command!"));
    } // конец проверки длины пакета
 } // конец функции onReceive
 
 // ======================= БИЗНЕС-ЛОГИКА =======================
 
 void enterDeepSleep() {
    DEBUGln(F("[STATE] ---> ENTERING DEEP SLEEP"));
    radio.sleep();
    SPI.end();
    pinMode(csPin, INPUT); pinMode(mosiPin, INPUT); pinMode(misoPin, INPUT);
    pinMode(sckPin, INPUT); pinMode(resetPin, INPUT); pinMode(busyPin, INPUT);
    pinMode(irqPin, INPUT);
    
    pinMode(PIN_VEXT, OUTPUT); digitalWrite(PIN_VEXT, HIGH);
    pinMode(PIN_ADC_CTRL, OUTPUT); digitalWrite(PIN_ADC_CTRL, HIGH);
    
    pinMode(PIN_STATUS_LED, OUTPUT); digitalWrite(PIN_STATUS_LED, LOW);
    pinMode(PIN_SIGNAL_LED, OUTPUT); digitalWrite(PIN_SIGNAL_LED, LOW);
    pinMode(PIN_SIGNAL_BUZZERS, OUTPUT); digitalWrite(PIN_SIGNAL_BUZZERS, LOW);
    
    rtc_gpio_pullup_en((gpio_num_t)PIN_REED);
    rtc_gpio_pulldown_dis((gpio_num_t)PIN_REED);
    
    if (digitalRead(PIN_REED) == LOW) {
      DEBUGln(F("[STATE] Reed is STUCK. Sleeping with timer..."));
      esp_sleep_enable_timer_wakeup(stuckSleepTime * 1000ULL);
    } else {
      DEBUGln(F("[STATE] Normal sleep. Wakeup on EXT0 (Reed). Good night!"));
      esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_REED, 0);
    } // конец проверки залипания геркона
    esp_deep_sleep_start();
 } // конец функции enterDeepSleep
 
 void runWakeUpProtection(uint8_t wakeupPin) {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) enterDeepSleep(); 
    
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
      DEBUGln(F("[STATE] Woke up from Deep Sleep. Checking protection..."));
      updateStatusLed(true);
      unsigned long startHold = millis();
      while (millis() - startHold < wakeUpHoldTime) {
        if (digitalRead(wakeupPin) == HIGH) { 
          DEBUGln(F("[ACTION] Magnet released too early. Going back to sleep."));
          updateStatusLed(false); enterDeepSleep(); 
        } // конец проверки раннего отпускания
        delay(10);
      } // конец цикла удержания
      
      DEBUGln(F("[STATE] Waiting for magnet release in window..."));
      unsigned long startReleaseWindow = millis();
      bool releasedInWindow = false;
      while (millis() - startReleaseWindow < wakeUpReleaseWindow) {
        updateStatusLed((millis() % 200) < 100); 
        if (digitalRead(wakeupPin) == HIGH) { 
          DEBUGln(F("[ACTION] Magnet released! WAKE UP SUCCESS."));
          releasedInWindow = true; break; 
        } // конец проверки отпускания в окне
        delay(10);
      } // конец цикла окна отпускания
      
      if (!releasedInWindow) { 
        DEBUGln(F("[ACTION] Magnet held too long. Going back to sleep."));
        updateStatusLed(false); enterDeepSleep(); 
      } // конец условия ошибки окна отпускания
      updateStatusLed(false); 
    } // конец проверки причины просыпания EXT0
 } // конец функции runWakeUpProtection
 
 void goToSleep() {
    updateStatusLed(true); delay(sleepLedDuration); updateStatusLed(false);
    enterDeepSleep();
 } // конец функции goToSleep
 
 void processTimeOut() {
    if ((millis() - pingTimeOutLastTime) > pingTimeout) {
      signalStatus = false;
      pingTimeOutLastTime = millis();
      
      if (currentState == STATE_NORMAL) {
        DEBUGln(F("[RADIO] Ping Timeout! No connection."));
        flashStatusLed(2); 
      }
      else if (currentState == STATE_CONFIG) { 
        DEBUGln(F("[STATE] Config Timeout (No Heartbeat) -> STATE_NORMAL"));
        currentState = STATE_NORMAL; updateStatusLed(false); 
      }
      else if (currentState == STATE_EXEC_CONFIG) { 
        DEBUGln(F("[STATE] Exec Config Timeout (No Heartbeat) -> STATE_NORMAL"));
        currentState = STATE_NORMAL; updateStatusLed(false); 
      } // конец ветвления состояний при таймауте
    } // конец проверки интервала таймаута
 } // конец функции processTimeOut
 
 void processCommand() {
    DEBUG(F("[ACTION] Processing Command: ")); DEBUGln(rcvCmd);
    switch (rcvCmd) {
      case CMD_SIGNAL:
        signalStatus = rcvData; processSignal();
        if (signalStatus) sendMessage(rcvAddress, CMD_SIGNAL_OK, signalStatus); 
        break;
      case CMD_PING: {
        unsigned long flashStatus = millis();
        signalStatus = rcvData; processSignal();
        updateStatusLed(true);
        unsigned long restDelay = 100 - (millis() - flashStatus);
        if (restDelay > 0) delay(restDelay);
        updateStatusLed(false);
        sendMessage(rcvAddress, CMD_PING_OK, signalStatus);     
        break;
      } // конец обработки CMD_PING
      case CMD_SLEEP:
        sendMessage(rcvAddress, CMD_SLEEP_OK, 1); 
        delay(100); goToSleep();
        break;
      case CMD_CONFIG:
        DEBUGln(F("[STATE] ---> STATE_CONFIG"));
        currentState = STATE_CONFIG;
        pingTimeOutLastTime = millis();
        sendMessage(rcvAddress, CMD_CONFIG_OK, 1);
        configBlinkActive = true; configBlinkStartTime = millis(); 
        analogWrite(PIN_SIGNAL_LED, 0); analogWrite(PIN_SIGNAL_BUZZERS, 0);
        break;
      case CMD_EXEC_CONFIG:
        DEBUGln(F("[STATE] ---> STATE_EXEC_CONFIG"));
        currentState = STATE_EXEC_CONFIG;
        pingTimeOutLastTime = millis();
        sendMessage(rcvAddress, CMD_EXEC_CONFIG_OK, 1);
        execBlinkActive = true; execBlinkStartTime = millis(); 
        analogWrite(PIN_SIGNAL_LED, 0); analogWrite(PIN_SIGNAL_BUZZERS, 0);
        break;
      case CMD_CYCLE_EXEC: {
        DEBUGln(F("[ACTION] CMD_CYCLE_EXEC received. Cycling actuators."));
        byte state = (enableBuzzer ? 2 : 0) | (enableBigLed ? 1 : 0);
        state = (state + 1) % 4;
        
        enableBigLed = (state & 0x01);
        enableBuzzer = (state & 0x02) >> 1;
        
        preferences.begin("davaylora", false);
        preferences.putBool("enBigLed", enableBigLed);
        preferences.putBool("enBuzzer", enableBuzzer);
        preferences.end();
        
        DEBUG(F("[STATE] Actuators updated. LED: ")); DEBUG(enableBigLed);
        DEBUG(F(", BUZZER: ")); DEBUGln(enableBuzzer);
        
        sendMessage(rcvAddress, CMD_CYCLE_EXEC_OK, state);
        
        signalStatus = true; processSignal(); 
        delay(1000); 
        signalStatus = false; processSignal();
        break;
      } // конец обработки CMD_CYCLE_EXEC
      case CMD_NORMAL_MODE:
        DEBUGln(F("[STATE] ---> STATE_NORMAL"));
        currentState = STATE_NORMAL;
        pingTimeOutLastTime = millis();
        updateStatusLed(false);
        sendMessage(rcvAddress, CMD_NORMAL_MODE_OK, 1);
        break;
    } // конец switch
    rcvCmd = 0; 
 } // конец функции processCommand
 
 void processSignal() {
    cutoffTimer = millis(); 
    if (signalStatus && enableBigLed) analogWrite(PIN_SIGNAL_LED, pwmledBrightness);
    else analogWrite(PIN_SIGNAL_LED, 0);
    
    if (signalStatus && enableBuzzer) analogWrite(PIN_SIGNAL_BUZZERS, buzzerVolume);
    else analogWrite(PIN_SIGNAL_BUZZERS, 0);
    
    digitalWrite(PIN_STATUS_LED, signalStatus);
 } // конец функции processSignal
 
 void processConfigLed() {
    if (!configBlinkActive) return;
    unsigned long elapsed = millis() - configBlinkStartTime;
    if (elapsed < 100) updateStatusLed(true); 
    else if (elapsed < 200) updateStatusLed(false); 
    else if (elapsed < 300) updateStatusLed(true); 
    else if (elapsed < 400) updateStatusLed(false); 
    else if (elapsed < 500) updateStatusLed(true); 
    else { updateStatusLed(false); configBlinkActive = false; }
 } // конец функции processConfigLed
 
 void processExecConfigLed() {
    if (!execBlinkActive) return;
    unsigned long elapsed = millis() - execBlinkStartTime;
    if (elapsed < 100) updateStatusLed(true); 
    else if (elapsed < 200) updateStatusLed(false); 
    else if (elapsed < 300) updateStatusLed(true); 
    else { updateStatusLed(false); execBlinkActive = false; }
 } // конец функции processExecConfigLed
 
 void processCutoff() {
    if (signalStatus && (millis() - cutoffTimer > cutoffTime)) {
      DEBUGln(F("[ACTION] Signal Cutoff Triggered!"));
      signalStatus = 0;
      analogWrite(PIN_SIGNAL_LED, 0); analogWrite(PIN_SIGNAL_BUZZERS, 0);
      digitalWrite(PIN_STATUS_LED, 0);
    } // конец проверки таймаута отсечки
 } // конец функции processCutoff
 
 void processUserButton() {
    static unsigned long userButtonTimer = 0;
    if (digitalRead(PIN_USER) == LOW) { 
      if (userButtonTimer == 0) userButtonTimer = millis();
      else if (millis() - userButtonTimer > 5000) {
        DEBUGln(F("[ACTION] USER button held 5s -> Local Sleep"));
        goToSleep(); 
      } // конец проверки удержания USER
    } else userButtonTimer = 0; 
 } // конец функции processUserButton
 
 void updateStatusLed(bool ledStatus) { 
    digitalWrite(PIN_STATUS_LED, ledStatus); 
 } // конец функции updateStatusLed
 
 void flashStatusLEDOnce() { 
    digitalWrite(PIN_STATUS_LED, 1); delay(250); digitalWrite(PIN_STATUS_LED, 0); delay(250); 
 } // конец функции flashStatusLEDOnce
 
 void flashStatusLed(byte times) { 
    for (int i = 0; i < times; i++) flashStatusLEDOnce(); 
    delay(200); 
 } // конец функции flashStatusLed
 
 // ======================= БАТАРЕЯ =======================
 
 bool testBattery() { 
    if (batteryVoltageOK(5)) return true;
    return false;
 } // конец функции testBattery
 
 bool batteryVoltageOK(byte tries) {
    float minV = 5.0, maxV = 0.0;
    for (byte i = 0; i < tries; i++) {
      float currentVBat = batteryVoltage();
      if (currentVBat < minV) minV = currentVBat; 
      if (currentVBat > maxV) maxV = currentVBat; 
      if ((currentVBat > 4.5) || (currentVBat < 2.5)) return false;
      delay(150);
    } // конец цикла замеров
    if ((maxV - minV) > 0.05) return false;
    return true;
 } // конец функции batteryVoltageOK
 
 float batteryVoltage() {
    digitalWrite(PIN_ADC_CTRL, LOW); delay(10);                       
    float measuredvbat = analogRead(PIN_BATTERY_INTERNAL);
    digitalWrite(PIN_ADC_CTRL, HIGH); 
    measuredvbat *= 3.3; measuredvbat /= 4095.0; measuredvbat *= HELTEC_BATTERY_MULTIPLIER; 
    return measuredvbat;
 } // конец функции batteryVoltage
 
 void processBattery() { 
    if (batteryVoltage() < BATTERY_MIN_VOLTAGE) stopWorking(); 
 } // конец функции processBattery
 
 void showBatteryVoltage() {
    float voltage = batteryVoltage();
    if (voltage > BATTERY_VOLTAGE_1) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_2) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_3) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_4) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_5) flashBatteryLEDOnce(); 
 } // конец функции showBatteryVoltage
 
 void showNoBattery() { 
    digitalWrite(PIN_BATTERY_LED, 1); delay(2000); digitalWrite(PIN_BATTERY_LED, 0); delay(250); 
 } // конец функции showNoBattery
 
 void flashBatteryLEDOnce() { 
    digitalWrite(PIN_BATTERY_LED, 1); delay(250); digitalWrite(PIN_BATTERY_LED, 0); delay(250); 
 } // конец функции flashBatteryLEDOnce
 
 void flashLedBattery(byte times) { 
    for (int i = 0; i < times; i++) flashBatteryLEDOnce(); 
    delay(200); 
 } // конец функции flashLedBattery
 
 void stopWorking() { 
    DEBUGln(F("[STATE] BATTERY DEAD! Stopping..."));
    flashLedBattery(7); digitalWrite(PIN_BATTERY_LED, 0); delay(3000); 
    flashLedBattery(7); digitalWrite(PIN_BATTERY_LED, 0); enterDeepSleep(); 
 } // конец функции stopWorking
 
 // ======================= СТАРТ И ЦИКЛ =======================
 
 void setup() {
    pinMode(PIN_REED, INPUT_PULLUP);
    pinMode(PIN_USER, INPUT_PULLUP); 
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);
 
    loadConfig();
    runWakeUpProtection(PIN_REED);
    delay(2000);
 
 #ifdef DEBUG_ENABLE
    Serial.begin(115200); 
    while (!Serial); 
 #endif
 
    DEBUGln(F("================================"));
    DEBUGln(F("=========== START RX v1.42 ==========="));
    DEBUG(F("Work Channel/Address: ")); DEBUGln(workAddress);
    DEBUG(F("Battery Check Enabled: ")); DEBUGln(measurebattery ? "YES" : "NO");
    DEBUG(F("RX BIG LED Brightness: ")); DEBUGln(pwmledBrightness);
    DEBUG(F("RX Buzzer Volume: ")); DEBUGln(buzzerVolume);
    
    DEBUG(F("Bat. Period (ms): ")); DEBUGln(batteryPeriod);
    DEBUG(F("Wake Hold/Rel (ms): ")); DEBUG(wakeUpHoldTime); DEBUG(F("/")); DEBUGln(wakeUpReleaseWindow);
    DEBUG(F("Stuck Sleep (ms): ")); DEBUGln(stuckSleepTime);
    DEBUG(F("Config TO (ms): ")); DEBUGln(configTimeout);
    DEBUG(F("Sleep LED (ms): ")); DEBUGln(sleepLedDuration);
    DEBUG(F("RX En. Big LED: ")); DEBUGln(enableBigLed ? "YES" : "NO");
    DEBUG(F("RX En. Buzzer: ")); DEBUGln(enableBuzzer ? "YES" : "NO");
    DEBUG(F("RX Cutoff (ms): ")); DEBUGln(cutoffTime);
    DEBUG(F("RX Ping TO (ms): ")); DEBUGln(pingTimeout);
 
    DEBUGln(F("[STATE] ---> STATE_NORMAL (Boot)"));
 
    pinMode(PIN_VEXT, OUTPUT); digitalWrite(PIN_VEXT, HIGH);     
    pinMode(PIN_ADC_CTRL, OUTPUT); digitalWrite(PIN_ADC_CTRL, HIGH); 
    pinMode(PIN_SIGNAL_BUZZERS, OUTPUT); pinMode(PIN_SIGNAL_LED, OUTPUT);
    
    analogWrite(PIN_SIGNAL_LED, 0); analogWrite(PIN_SIGNAL_BUZZERS, 0);
    digitalWrite(PIN_BATTERY_LED, 0); delay(300);
 
    updateStatusLed(true);
    if (enableBigLed) analogWrite(PIN_SIGNAL_LED, pwmledBrightness);
    if (enableBuzzer) analogWrite(PIN_SIGNAL_BUZZERS, buzzerVolume);
    delay(1000);
    updateStatusLed(false);
    analogWrite(PIN_SIGNAL_LED, 0); analogWrite(PIN_SIGNAL_BUZZERS, 0);  
    delay(1000);
 
    if (measurebattery) {
      isBatteryConnected = testBattery(); 
    } // конец проверки подключения батареи
    
    if (measurebattery && isBatteryConnected) {
      processBattery(); delay(500); showBatteryVoltage(); delay(2000); showBatteryVoltage(); delay(500);
    } else if (measurebattery && !isBatteryConnected) { 
      showNoBattery(); delay(500); 
    } // конец ветвления индикации батареи
 
    SPI.begin(sckPin, misoPin, mosiPin, csPin);
    workFrequency = workingFrequency[workAddress % MAX_ADDRESS];
    DEBUG(F("[RADIO] LoRa Init on Frequency: ")); DEBUGln(workFrequency);
    
    int state = radio.begin(workFrequency / 1000000.0);
    if (state != RADIOLIB_ERR_NONE) {
      DEBUG(F("[RADIO] Init FAILED, code: ")); DEBUGln(state);
      while (true) { flashStatusLed(6); delay(4000); }
    } // конец проверки инициализации радио
 
    setLoRaParams();
    radio.setDio1Action(setFlag);
    radio.startReceive();
 
    pingTimeOutLastTime = millis();
    DEBUGln(F("[STATE] Setup complete"));
 } // конец функции setup
 
 void loop() {
    checkReceive(); 
 
    if (rcvCmd) processCommand(); 
    else processTimeOut(); 
 
    if (currentState == STATE_NORMAL) processCutoff();
    else if (currentState == STATE_CONFIG) processConfigLed();
    else if (currentState == STATE_EXEC_CONFIG) processExecConfigLed();
    
    processUserButton(); 
 
    EVERY_MS(batteryPeriod) {
      if (measurebattery && isBatteryConnected) processBattery();
    } // конец интервала проверки батареи
 } // конец функции loop