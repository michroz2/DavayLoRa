/**
 * @file main.cpp (RX)
 * @version 1.5 (Изменение: Засыпание системы по 5-секундному удержанию кнопки USER)
 * @brief Прошивка приёмника (Receiver) для проекта DavayLoRa на базе Heltec Wireless Stick Lite V3
 */

 #include <Arduino.h>
 #include <esp_sleep.h>
 #include <SPI.h>
 #include <RadioLib.h>
 #include <Preferences.h>
 
 Preferences preferences;
 
 // ======================= ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ (ИЗ ПАМЯТИ NVS) =======================
 byte workAddress = 4;                 
 int pwmledBrightness = 35;            
 int buzzerVolume = 255;               
 unsigned long cutoffTime = 2000;      
 bool measurebattery = true;           
 
 // Настройки активной периферии
 bool enableBigLed = true;             
 bool enableBuzzer = false;            
 
 // Настройки надежности связи (Константы)
 #define PING_TIMEOUT 5000             
 
 // Пороги индикации заряда батареи (в Вольтах)
 #define BATTERY_MIN_VOLTAGE 3.5
 #define BATTERY_VOLTAGE_1 3.6
 #define BATTERY_VOLTAGE_2 3.7
 #define BATTERY_VOLTAGE_3 3.8
 #define BATTERY_VOLTAGE_4 3.9
 #define BATTERY_VOLTAGE_5 4.0
 
 #define BATTERY_PERIOD 300000         // Периодичность проверки батареи (мс) = 5 минут
 
 // ======================= АППАРАТНАЯ КОНФИГУРАЦИЯ (HELTEC WSL V3) =======================
 
 // --- Исполнительные пины ---
 #define PIN_SIGNAL_LED      41     
 #define PIN_SIGNAL_BUZZERS  42     
 #define PIN_REED            7      
 #define PIN_USER            0      // Встроенная кнопка PRG/USER на плате Heltec
 
 // --- Индикация (Встроенный LED) ---
 #define PIN_STATUS_LED      35     
 #define PIN_BATTERY_LED     35     
 
 // --- Пины измерения батареи ---
 #define PIN_BATTERY_INTERNAL 1     
 #define PIN_VEXT 36                
 #define PIN_ADC_CTRL 37            
 #define HELTEC_BATTERY_MULTIPLIER 4.9 
 
 // --- Пины аппаратной шины SPI и радиомодуля SX1262 ---
 const int sckPin = 9;
 const int misoPin = 11;
 const int mosiPin = 10;
 const int csPin = 8;
 const int resetPin = 12;
 const int irqPin = 14;
 const int busyPin = 13;
 
 SX1262 radio = new Module(csPin, irqPin, resetPin, busyPin);
 
 // ======================= ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ И ПРОТОКОЛ =======================
 
 #define WORK_FREQUENCY 434E6
 
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
 
 #define MAX_ADDRESS 20
 
 #define CMD_SIGNAL         208
 #define CMD_SIGNAL_OK      209
 #define CMD_PING           212
 #define CMD_PING_OK        213
 #define CMD_SLEEP          214
 #define CMD_SLEEP_OK       215
 
 byte rcvAddress = 0;
 byte rcvCmd = 0;
 byte rcvData = 0;
 byte sndCmd = CMD_PING_OK;
 byte sndData;
 bool signalStatus;
 byte workChannel;
 unsigned long workFrequency = WORK_FREQUENCY;
 
 unsigned long lastSendTime = 0;
 int lastRSSI;
 float lastSNR;
 unsigned long lastTurnaround;
 float lastFrequencyError;
 
 unsigned long pingTimeOutLastTime;
 unsigned long cutoffTimer = 0;
 
 volatile bool receivedFlag = false;
 
 unsigned long workingFrequency[MAX_ADDRESS] = {
   434000000, 434120000, 434240000, 433820000, 433700000, 433940000, 434030000,
   434150000, 434270000, 433850000, 433730000, 433970000, 434060000, 434180000,
   433880000, 433760000, 434090000, 434210000, 433910000, 433790000,
 };
 
 // --- ПРОТОТИПЫ ФУНКЦИЙ ---
 void loadConfig();
 void saveConfig();
 void processTimeOut();
 void processCommand();
 void processSignal();
 void processCutoff();
 void processUserButton();
 void goToSleep();
 void updateStatusLed(bool ledStatus);
 void flashStatusLEDOnce();
 void flashStatusLed(byte times);
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
 
 // ======================= РАБОТА С ПАМЯТЬЮ NVS =======================
 
 void loadConfig() {
   DEBUGln(F("Loading config from NVS..."));
   preferences.begin("davaylora", false);
   
   workAddress = preferences.getUChar("workAddress", 4);
   measurebattery = preferences.getBool("measureBat", true);
   pwmledBrightness = preferences.getInt("bigLedBright", 35);
   buzzerVolume = preferences.getInt("buzzerVol", 255);
   cutoffTime = preferences.getULong("cutoffTime", 2000);
   
   enableBigLed = preferences.getBool("enBigLed", true);
   enableBuzzer = preferences.getBool("enBuzzer", false);
   
   preferences.end();
   DEBUGln(F("Config loaded."));
 } // end loadConfig
 
 void saveConfig() {
   DEBUGln(F("Saving config to NVS..."));
   preferences.begin("davaylora", false);
   
   preferences.putUChar("workAddress", workAddress);
   preferences.putBool("measureBat", measurebattery);
   preferences.putInt("bigLedBright", pwmledBrightness);
   preferences.putInt("buzzerVol", buzzerVolume);
   preferences.putULong("cutoffTime", cutoffTime);
   
   preferences.putBool("enBigLed", enableBigLed);
   preferences.putBool("enBuzzer", enableBuzzer);
   
   preferences.end();
   DEBUGln(F("Config saved."));
 } // end saveConfig
 
 // ======================= ЛОГИКА ПРЕРЫВАНИЙ И СЕТИ =======================
 
 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR
 #endif
 void setFlag(void) {
   receivedFlag = true;
 } // end setFlag
 
 void setLoRaParams() {
   DEBUGln("setLoRaParams()");
   radio.setOutputPower(20);                     
   radio.setBandwidth(125.0);                    
   radio.setSpreadingFactor(8);                  
   radio.setCodingRate(5);                       
   radio.setPreambleLength(8);                   
   radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE); 
 } // end setLoRaParams
 
 void sendMessage(byte msgAddr, byte msgCmd, byte msgData) {
   DEBUGln(F(">>>Sending Message"));
   
   byte payload[3] = {msgAddr, msgCmd, msgData}; 
   
   int state = radio.transmit(payload, 3);       
   
   if (state == RADIOLIB_ERR_NONE) {
     DEBUGln(("\tMessage sent: ") + String(msgAddr) + " " + String(msgCmd) + " " + String(msgData));
   } else {
     DEBUGln(("\tTransmit failed, code: ") + String(state));
   } // end if
   
   lastSendTime = millis();
   pingTimeOutLastTime = lastSendTime;
   radio.startReceive();                         
 } // end sendMessage
 
 void checkReceive() {
   if (receivedFlag) {
     receivedFlag = false;
     byte payload[256];
     int state = radio.readData(payload, sizeof(payload));
     
     if (state == RADIOLIB_ERR_NONE) {
       int packetSize = radio.getPacketLength();
       onReceive(payload, packetSize);           
     } // end if
     radio.startReceive();                       
   } // end if
 } // end checkReceive
 
 void onReceive(byte* payload, int packetSize) {
   DEBUGln(F("\n<<<Package Received"));
 
   rcvAddress = payload[0];
   if ((rcvAddress != workAddress) || (packetSize != 3)) {
     DEBUGln(F("Invalid package! "));
     return;
   } // end if
 
   rcvCmd = payload[1];
   rcvData = payload[2];
 
   lastFrequencyError = radio.getFrequencyError();
 
 #ifdef DEBUG_ENABLE
   lastRSSI = radio.getRSSI();
   lastSNR = radio.getSNR();
   DEBUGln("\tReceived Message:\t"  + String(rcvAddress) + " " + String(rcvCmd) + " " + String(rcvData));
   DEBUGln("\tRSSI:\t" + String(lastRSSI));
   DEBUGln("\tSnr:\t" + String(lastSNR));
   DEBUGln("\tFrequency Error:\t" + String(lastFrequencyError));
 #endif
 
   DEBUGln(F("=== onReceive done ==="));
 } // end onReceive
 
 // ======================= БИЗНЕС-ЛОГИКА (УПРАВЛЕНИЕ ПЕРИФЕРИЕЙ) =======================
 
 // Вынесенная логика ухода в сон
 void goToSleep() {
   flashStatusLed(3); 
   analogWrite(PIN_SIGNAL_LED, 0);
   analogWrite(PIN_SIGNAL_BUZZERS, 0);
   digitalWrite(PIN_STATUS_LED, 0);
   
   DEBUGln(F("Going to deep sleep..."));
   esp_deep_sleep_start();
 } // end goToSleep
 
 void processTimeOut() {
   if ((millis() - pingTimeOutLastTime) > PING_TIMEOUT) {
     DEBUGln(F("ZZZZZZZ"));
     signalStatus = false;
     pingTimeOutLastTime = millis();
     flashStatusLed(2); 
   } // end if
 } // end processTimeOut
 
 void processCommand() {
   switch (rcvCmd) {
     case CMD_SIGNAL: {
       DEBUGln(F("=== CMD_SIGNAL ==="));
       signalStatus = rcvData;
       processSignal();
       if (signalStatus) {
         sendMessage(rcvAddress, CMD_SIGNAL_OK, signalStatus); 
       } // end if
       break;
     } // end case CMD_SIGNAL
     
     case CMD_PING: {
       unsigned long flashStatus = millis();
       DEBUGln(F("=== CMD_PING ==="));
       signalStatus = rcvData;
       processSignal();
       
       updateStatusLed(true);
       unsigned long restDelay = 100 - (millis() - flashStatus);
       if (restDelay > 0) {
         delay(restDelay);
       } // end if
       updateStatusLed(false);
       
       sendMessage(rcvAddress, CMD_PING_OK, signalStatus);     
       break;
     } // end case CMD_PING
       
     case CMD_SLEEP: {
       DEBUGln(F("=== CMD_SLEEP ==="));
       sendMessage(rcvAddress, CMD_SLEEP_OK, 1); 
       delay(100); 
       
       goToSleep();
       break;
     } // end case CMD_SLEEP
   } // end switch
   rcvCmd = 0; 
 } // end processCommand
 
 void processSignal() {
   cutoffTimer = millis(); 
   
   if (signalStatus && enableBigLed) {
     analogWrite(PIN_SIGNAL_LED, pwmledBrightness);
   } else {
     analogWrite(PIN_SIGNAL_LED, 0);
   } // end if
 
   if (signalStatus && enableBuzzer) {
     analogWrite(PIN_SIGNAL_BUZZERS, buzzerVolume);
   } else {
     analogWrite(PIN_SIGNAL_BUZZERS, 0);
   } // end if
   
   digitalWrite(PIN_STATUS_LED, signalStatus);
 } // end processSignal
 
 void processCutoff() {
   if (signalStatus && (millis() - cutoffTimer > cutoffTime)) {
     signalStatus = 0;
     analogWrite(PIN_SIGNAL_LED, 0);
     analogWrite(PIN_SIGNAL_BUZZERS, 0);
     digitalWrite(PIN_STATUS_LED, 0);
   } // end if
 } // end processCutoff
 
 // Обработка встроенной кнопки USER
 void processUserButton() {
   static unsigned long userButtonTimer = 0;
   
   if (digitalRead(PIN_USER) == LOW) { // Кнопка зажата (замыкание на GND)
     if (userButtonTimer == 0) {
       userButtonTimer = millis();
     } else if (millis() - userButtonTimer > 5000) {
       DEBUGln(F("USER button held 5s -> LOCAL SLEEP"));
       goToSleep(); // Локально засыпаем
     } // end if
   } else {
     userButtonTimer = 0; // Кнопка отпущена
   } // end if
 } // end processUserButton
 
 // ======================= ИНДИКАЦИЯ =======================
 
 void updateStatusLed(bool ledStatus) {
   digitalWrite(PIN_STATUS_LED, ledStatus);
 } // end updateStatusLed
 
 void flashStatusLEDOnce() {
   digitalWrite(PIN_STATUS_LED, 1);
   delay(250);
   digitalWrite(PIN_STATUS_LED, 0);
   delay(250);
 } // end flashStatusLEDOnce
 
 void flashStatusLed(byte times) {
   for (int i = 0; i < times; i++) {
     flashStatusLEDOnce();
   } // end for
   delay(200);
 } // end flashStatusLed
 
 // ======================= УПРАВЛЕНИЕ БАТАРЕЕЙ =======================
 
 bool testBattery()   {
   DEBUGln(F("Проверка измерителя напряжения батареи Heltec WSL V3..."));
   if (batteryVoltageOK(5)) {
     DEBUGln(F("Батарея подключена и измеритель РАБОТАЕТ!"));
     return (true);
   } // end if
   DEBUGln(F("Батарея НЕ ПОДКЛЮЧЕНА (или сбой измерителя)!"));
   return (false);
 } // end testBattery
 
 bool batteryVoltageOK(byte tries) {
   DEBUGln(F("Is battery voltage OK?"));
   float minV = 5.0;
   float maxV = 0.0;
   
   for (byte i = 0; i < tries; i++) {
     float currentVBat = batteryVoltage();
     
     if (currentVBat < minV) {
       minV = currentVBat;
     } // end if
     if (currentVBat > maxV) {
       maxV = currentVBat;
     } // end if
     
     if ((currentVBat > 4.5) || (currentVBat < 2.5)) {
       DEBUGln(F("Voltage out of bounds (No battery or deeply discharged)!"));
       return (false);
     } // end if
     delay(150);
   } // end for
   
   DEBUG(F("Min V: ")); DEBUG(minV); DEBUG(F(", Max V: ")); DEBUGln(maxV);
   
   if ((maxV - minV) > 0.05) {
     DEBUGln(F("Voltage is unstable (Sawtooth ripple)! Батарея НЕ подключена!"));
     return (false);
   } // end if
   
   return (true);
 } // end batteryVoltageOK
 
 float batteryVoltage() {
   DEBUG(F("Battery Voltage: "));
 
   digitalWrite(PIN_ADC_CTRL, LOW); 
   delay(10);                       
 
   float measuredvbat = analogRead(PIN_BATTERY_INTERNAL);
 
   digitalWrite(PIN_ADC_CTRL, HIGH); 
 
   measuredvbat *= 3.3;
   measuredvbat /= 4095.0;           
   measuredvbat *= HELTEC_BATTERY_MULTIPLIER; 
 
   DEBUGln(measuredvbat);
   return measuredvbat;
 } // end batteryVoltage
 
 void processBattery() {
   if (batteryVoltage() < BATTERY_MIN_VOLTAGE) {
     stopWorking();
   } // end if
 } // end processBattery
 
 void showBatteryVoltage() {
   float voltage = batteryVoltage();
   if (voltage > BATTERY_VOLTAGE_1) { flashBatteryLEDOnce(); } // end if
   if (voltage > BATTERY_VOLTAGE_2) { flashBatteryLEDOnce(); } // end if
   if (voltage > BATTERY_VOLTAGE_3) { flashBatteryLEDOnce(); } // end if
   if (voltage > BATTERY_VOLTAGE_4) { flashBatteryLEDOnce(); } // end if
   if (voltage > BATTERY_VOLTAGE_5) { flashBatteryLEDOnce(); } // end if
 } // end showBatteryVoltage
 
 void showNoBattery() {
   digitalWrite(PIN_BATTERY_LED, 1);
   delay(2000); 
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(250);
 } // end showNoBattery
 
 void flashBatteryLEDOnce() {
   digitalWrite(PIN_BATTERY_LED, 1);
   delay(250);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(250);
 } // end flashBatteryLEDOnce
 
 void flashLedBattery(byte times) {
   for (int i = 0; i < times; i++) {
     flashBatteryLEDOnce();
   } // end for
   delay(200);
 } // end flashLedBattery
 
 void stopWorking() {
   flashLedBattery(7);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(3000);
   flashLedBattery(7);
   digitalWrite(PIN_BATTERY_LED, 0);
   
   esp_deep_sleep_start();
 } // end stopWorking
 
 // ======================= ОСНОВНЫЕ ФУНКЦИИ (SETUP & LOOP) =======================
 
 void setup() {
   delay(2000);
 
 #ifdef DEBUG_ENABLE
   Serial.begin(115200); 
   while (!Serial); // end while
 #endif
 
   DEBUGln(F("================================"));
   DEBUGln(F("=========== START RX ==========="));
 
   loadConfig();
 
   pinMode(PIN_VEXT, OUTPUT);
   digitalWrite(PIN_VEXT, HIGH);     
   pinMode(PIN_ADC_CTRL, OUTPUT);
   digitalWrite(PIN_ADC_CTRL, HIGH); 
 
   pinMode(PIN_STATUS_LED, OUTPUT);
   pinMode(PIN_SIGNAL_BUZZERS, OUTPUT);
   pinMode(PIN_SIGNAL_LED, OUTPUT);
   
   pinMode(PIN_REED, INPUT_PULLUP);
   pinMode(PIN_USER, INPUT_PULLUP); // Инициализация кнопки USER
   
   analogWrite(PIN_SIGNAL_LED, 0);
   analogWrite(PIN_SIGNAL_BUZZERS, 0);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(300);
 
   updateStatusLed(true);
   
   if (enableBigLed) {
     analogWrite(PIN_SIGNAL_LED, pwmledBrightness);
   } // end if
   
   if (enableBuzzer) {
     analogWrite(PIN_SIGNAL_BUZZERS, buzzerVolume);
   } // end if
   
   delay(1000);
   updateStatusLed(false);
   analogWrite(PIN_SIGNAL_LED, 0);
   analogWrite(PIN_SIGNAL_BUZZERS, 0);  
   delay(1000);
 
   DEBUGln(F("Battery Test"));
   if (measurebattery) {
     measurebattery = testBattery();
   } // end if
        
   if (measurebattery) {
     DEBUGln(F("-Measuring"));
     processBattery();
     delay(500);
     showBatteryVoltage();
     delay(2000);
     showBatteryVoltage();
     delay(500);
   } else {
     DEBUGln(F("-Cancelled"));
     showNoBattery();
     delay(500);
   } // end if
 
   SPI.begin(sckPin, misoPin, mosiPin, csPin);
   
   workFrequency = workingFrequency[workAddress % MAX_ADDRESS];
   DEBUG("LoRa begin on ");
   DEBUGln(workFrequency);
   
   int state = radio.begin(workFrequency / 1000000.0);
   if (state != RADIOLIB_ERR_NONE) {
     DEBUGln("LoRa init failed. Code: " + String(state));
     while (true) {
       flashStatusLed(6);    
       delay(4000);
     } // end while
   } // end if
 
   setLoRaParams();
 
   radio.setDio1Action(setFlag);
   radio.startReceive();
 
   pingTimeOutLastTime = millis();
 
   DEBUGln(F("DavayLoRa RX setup complete"));
 } // end setup
 
 void loop() {
   checkReceive(); 
 
   if (rcvCmd) {
     processCommand(); 
   } else {
     processTimeOut(); 
   } // end if
 
   processCutoff();
   
   processUserButton(); // Проверка 5-секундного удержания кнопки USER
 
   EVERY_MS(BATTERY_PERIOD) {
     if (measurebattery) {
       processBattery();
     } // end if
   } // end EVERY_MS
 } // end loop