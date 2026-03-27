/**
 * @file main.cpp (TX)
 * @version 1.3 (Изменение: Засыпание системы по 5-секундному удержанию кнопки USER)
 * @brief Прошивка передатчика (Transmitter) для проекта DavayLoRa на базе Heltec Wireless Stick Lite V3
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
 int fbledBrightness = 255;            
 unsigned long pingTimeout = 3000;     
 unsigned long bigTimeout = 3600000;   
 bool measurebattery = true;           
 
 // Пороги индикации заряда батареи (в Вольтах)
 #define BATTERY_MIN_VOLTAGE 3.5
 #define BATTERY_VOLTAGE_1 3.5
 #define BATTERY_VOLTAGE_2 3.6
 #define BATTERY_VOLTAGE_3 3.8
 #define BATTERY_VOLTAGE_4 3.9
 #define BATTERY_VOLTAGE_5 4.0
 
 #define BATTERY_PERIOD 300000         // 5 минут
 
 // ======================= АППАРАТНАЯ КОНФИГУРАЦИЯ (HELTEC WSL V3) =======================
 
 // --- Пины интерфейса ---
 #define PIN_BUTTON 45          
 #define PIN_FB_LED 35          
 #define PIN_BIG_LED 41         
 #define PIN_BATTERY_LED 35     
 #define PIN_USER 0             // Встроенная кнопка PRG/USER на плате Heltec
 
 // --- Пины измерения батареи ---
 #define PIN_BATTERY_INTERNAL 1 
 #define PIN_VEXT 36            
 #define PIN_ADC_CTRL 37        
 #define HELTEC_BATTERY_MULTIPLIER 4.9
 
 // --- Пины SPI и SX1262 ---
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
 
 // Настройки надежности радиообмена
 #define DEFAULT_TURNAROUND 300     
 #define WORK_COMM_ATTEMPTS 3       
 #define PING_FLASH 100             
 #define DEBOUNCE_TIME 100          
 
 // Команды протокола
 #define CMD_SIGNAL    208
 #define CMD_SIGNAL_OK 209
 #define CMD_PING      212
 #define CMD_PING_OK   213
 #define CMD_SLEEP     214
 #define CMD_SLEEP_OK  215
 
 // Переменные стейт-машины и сети
 byte sndCmd = CMD_PING;
 byte sndData;
 bool wasReceived = 0;
 byte cmdExpected = CMD_PING_OK;
 byte rcvAddress = 0;
 byte rcvCmd = 0;
 byte rcvData;
 byte workChannel;
 unsigned long workFrequency = WORK_FREQUENCY;
 
 long lastSendTime = 0;
 int lastRSSI;
 float lastSNR;
 unsigned long lastTurnaround = DEFAULT_TURNAROUND;
 long lastFrequencyError;
 unsigned long lastButtonTime;
 
 // Переменные состояния кнопки
 bool currButtonState;
 bool prevButtonState;
 bool buttonPressedFirstTime;
 
 // Таймеры пинга
 unsigned long pingTimer;
 unsigned long pingFlashTimer;
 bool pingFlash;
 
 volatile bool receivedFlag = false;
 
 // --- Стейт-машина пульта ---
 enum SystemState {
   STATE_NORMAL,
   STATE_PREPARATION
 };
 SystemState currentState = STATE_NORMAL;
 
 unsigned long buttonPressStartTime = 0; 
 unsigned long prepModeTimer = 0;        
 byte prepClickCount = 0;
 unsigned long lastPrepClickTime = 0;
 
 // --- ПРОТОТИПЫ ФУНКЦИЙ ---
 void loadConfig();
 void saveConfig();
 void processButton();
 void processPreparationMode();
 void processPing();
 void processUserButton();
 void sleepSystem();
 void updateStatusLed(bool ledStatus);
 void updateBIGLed(bool ledStatus);
 void flashStatusLed(byte times);
 void sendMessage(byte msgCmd, byte sndData);
 bool commSession(byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes);
 void setLoRaParams();
 void onReceive(byte* payload, int packetSize);
 void checkReceive();
 bool testBattery();
 bool batteryVoltageOK(byte tries);
 float batteryVoltage();
 void showBatteryVoltage();
 void showNoBattery();
 void flashBatteryLEDOnce();
 void flashLedBattery(byte times);
 void processBattery();
 void stopWorking();
 
 // ======================= РАБОТА С ПАМЯТЬЮ NVS =======================
 
 void loadConfig() {
   DEBUGln(F("Loading config from NVS..."));
   preferences.begin("davaylora", false); 
   
   workAddress = preferences.getUChar("workAddress", 4);
   measurebattery = preferences.getBool("measureBat", true);
   pwmledBrightness = preferences.getInt("bigLedBright", 35);
   fbledBrightness = preferences.getInt("fbLedBright", 255);
   pingTimeout = preferences.getULong("pingTimeout", 3000);
   bigTimeout = preferences.getULong("bigTimeout", 3600000);
   
   preferences.end();
   DEBUGln(F("Config loaded."));
 } // end loadConfig
 
 void saveConfig() {
   DEBUGln(F("Saving config to NVS..."));
   preferences.begin("davaylora", false);
   
   preferences.putUChar("workAddress", workAddress);
   preferences.putBool("measureBat", measurebattery);
   preferences.putInt("bigLedBright", pwmledBrightness);
   preferences.putInt("fbLedBright", fbledBrightness);
   preferences.putULong("pingTimeout", pingTimeout);
   preferences.putULong("bigTimeout", bigTimeout);
   
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
 
 void sendMessage(byte msgCmd, byte sndData) {
   DEBUGln(">>>sendMessage()");
   byte payload[3] = {workAddress, msgCmd, sndData};
   
   int state = radio.transmit(payload, 3);
   
   if (state == RADIOLIB_ERR_NONE) {
     DEBUGln(("\tMessage sent: ") + String(workAddress) + " " + String(msgCmd) + " " + String(sndData));
   } else {
     DEBUGln(("\tTransmit failed, code: ") + String(state));
   } // end if
   
   lastSendTime = millis();
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
 
 bool commSession(byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes) {
   DEBUGln(F("commSession()"));
   wasReceived = false;
   cmdExpected = expectedReply; 
   pingTimer = millis();
   
   do {
     checkReceive();
     EVERY_MS(waitMilliseconds) {
       sendMessage(msgCmd, sndData);
       doTimes--;
     } // end EVERY_MS
   } while ((doTimes > 0) && (!wasReceived)); // end while
   
   pingTimer = millis();
   return wasReceived; 
 } // end commSession
 
 unsigned long workingFrequency[MAX_ADDRESS] = {
   434000000, 434120000, 434240000, 433820000, 433700000, 433940000, 434030000,
   434150000, 434270000, 433850000, 433730000, 433970000, 434060000, 434180000,
   433880000, 433760000, 434090000, 434210000, 433910000, 433790000,
 };
 
 void setLoRaParams() {
   DEBUGln("setLoRaParams()");
   radio.setOutputPower(20);
   radio.setBandwidth(125.0);
   radio.setSpreadingFactor(8);
   radio.setCodingRate(5);
   radio.setPreambleLength(8);     
   radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE); 
 } // end setLoRaParams
 
 void onReceive(byte* payload, int packetSize) {
   DEBUGln("<<<PackageReceived");
 
   rcvAddress = payload[0];
   if ((rcvAddress != workAddress) || (packetSize != 3)) {
     delay(30);
     return;
   } // end if
 
   rcvCmd = payload[1];
   if (rcvCmd != cmdExpected) {
     DEBUGln("\tInvalid Reply: " + String(rcvCmd) + F(", Expected: ") + String(cmdExpected));
     delay(30);
     return;
   } // end if
   rcvData = payload[2];
 
   lastTurnaround = millis() - lastSendTime;
   lastFrequencyError = radio.getFrequencyError();
   lastRSSI = radio.getRSSI();
   lastSNR = radio.getSNR();
   
   DEBUGln("\tReceived Message: "  + String(rcvAddress) +" " + String( rcvCmd) + " " + String( rcvData));
   DEBUGln(("\tTurnaround: ") + String(lastTurnaround));
   DEBUGln(F("=== onReceive done ==="));
 
   wasReceived = true; 
 } // end onReceive
 
 // ======================= БИЗНЕС-ЛОГИКА (КНОПКА И ПИНГ) =======================
 
 // Вынесенная логика усыпления системы
 void sleepSystem() {
   if (commSession(CMD_SLEEP, 1, CMD_SLEEP_OK, 5 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
      DEBUGln(F("RX is sleeping, TX going to sleep too"));
      updateStatusLed(true);
      delay(2000);
      updateStatusLed(false);
      
      esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BUTTON, 0); 
      esp_deep_sleep_start();
   } else {
      DEBUGln(F("Failed to send SLEEP to RX"));
      flashStatusLed(2); 
   } // end if
 } // end sleepSystem
 
 void processButton() {
   prevButtonState = currButtonState;
   currButtonState = !digitalRead(PIN_BUTTON); 
   
   if (prevButtonState != currButtonState) {
     lastButtonTime = millis();
     DEBUGln("\nprocessButton(): " + String(currButtonState));
     prevButtonState = currButtonState;
     
     if (currButtonState) {
       if (currentState == STATE_NORMAL) {
         buttonPressStartTime = millis(); 
         pingTimer = millis();
         buttonPressedFirstTime = true;
         
         if (commSession(CMD_SIGNAL, 1, CMD_SIGNAL_OK, 2 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
           updateStatusLed(true); 
           updateBIGLed(true);    
         } else {
           updateBIGLed(false);
           flashStatusLed(2);     
         } // end if
       } else if (currentState == STATE_PREPARATION) {
         prepClickCount++;
         lastPrepClickTime = millis();
         prepModeTimer = millis(); 
       } // end if
     } else {
       if (currentState == STATE_NORMAL) {
         sendMessage(CMD_SIGNAL, false);
         updateStatusLed(false);
         updateBIGLed(false);
       } // end if
     } // end if
   } // end if
 } // end processButton
 
 void processPreparationMode() {
   EVERY_MS(166) {
     static bool prepLedState = false;
     prepLedState = !prepLedState;
     updateStatusLed(prepLedState);
   } // end EVERY_MS
 
   if (prepClickCount > 0 && (millis() - lastPrepClickTime > 600)) {
     if (prepClickCount == 2) {
       DEBUGln(F("2 clicks detected -> Go to SLEEP"));
       sleepSystem();
     } // end if
     prepClickCount = 0; 
   } // end if
 
   if (millis() - prepModeTimer > 10000) {
     DEBUGln(F("Exit Preparation Mode (Timeout)"));
     currentState = STATE_NORMAL;
     prepClickCount = 0; 
     updateStatusLed(false);
   } // end if
 } // end processPreparationMode
 
 void processPing() {
   if (!buttonPressedFirstTime) { return; } // end if
   
   if (pingFlash) {
     if ((millis() - pingFlashTimer) > PING_FLASH) {
       DEBUGln(F("\tPing LED OFF"));
       pingFlash = false;
       updateStatusLed(currButtonState);
     } // end if
   } else if ((millis() - pingTimer) > pingTimeout) {
     DEBUGln(F("\nStart Ping"));
     if (commSession(CMD_PING, currButtonState, CMD_PING_OK, 5 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
       DEBUGln(F("\tPing LED ON"));
       updateStatusLed(!currButtonState); 
       pingFlash = true;
       pingFlashTimer = millis();
       pingTimer = millis();
     } else {
       flashStatusLed(2); 
     } // end if
   } // end if
   
   if ((millis() - lastButtonTime) > bigTimeout) {
     flashStatusLed(3);
     buttonPressedFirstTime = false;
   } // end if
 } // end processPing
 
 // Обработка встроенной кнопки USER
 void processUserButton() {
   static unsigned long userButtonTimer = 0;
   
   if (digitalRead(PIN_USER) == LOW) { // Кнопка зажата (замыкание на GND)
     if (userButtonTimer == 0) {
       userButtonTimer = millis(); // Фиксируем начало нажатия
     } else if (millis() - userButtonTimer > 5000) {
       DEBUGln(F("USER button held 5s -> Centralized SLEEP"));
       sleepSystem(); // Используем ту же логику усыпления обеих плат
       userButtonTimer = 0; // Сброс таймера (если связь с RX прервалась и сон отменился)
     } // end if
   } else {
     userButtonTimer = 0; // Кнопка отпущена
   } // end if
 } // end processUserButton
 
 // ======================= ИНДИКАЦИЯ =======================
 
 void updateStatusLed(bool ledStatus) {
   digitalWrite(PIN_FB_LED, ledStatus);
 } // end updateStatusLed
 
 void updateBIGLed(bool ledStatus) {
   analogWrite(PIN_BIG_LED, ledStatus * pwmledBrightness);
 } // end updateBIGLed
 
 void flashStatusLed(byte times) {
   for (int i = 0; i < times; i++) {
     updateStatusLed(true);
     delay(100);
     updateStatusLed(false);
     delay(200);
   } // end for
 } // end flashStatusLed
 
 // ======================= УПРАВЛЕНИЕ БАТАРЕЕЙ =======================
 
 bool testBattery() {
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
     
     if (currentVBat < minV) { minV = currentVBat; } // end if
     if (currentVBat > maxV) { maxV = currentVBat; } // end if
     
     if ((currentVBat > 4.3) || (currentVBat < 2.5)) {
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
   measuredvbat *= HELTEC_BATTERY_MULTIPLIER;
   measuredvbat /= 4095.0;
 
   DEBUGln(measuredvbat);
   return measuredvbat;
 } // end batteryVoltage
 
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
 
 void processBattery() {
   if (batteryVoltage() < BATTERY_MIN_VOLTAGE) {
     stopWorking();
   } // end if
 } // end processBattery
 
 void stopWorking() {
   flashLedBattery(7);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(5000);
   flashLedBattery(7);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(5000);
   
   esp_deep_sleep_start();
 } // end stopWorking
 
 // ======================= ОСНОВНЫЕ ФУНКЦИИ (SETUP & LOOP) =======================
 
 void setup() {
   delay(2000);
 
 #ifdef DEBUG_ENABLE
   Serial.begin(115200);
   while (!Serial); // end while
 #endif
 
   DEBUGln("================================");
   DEBUGln("=========== START TX ===========");
 
   loadConfig();
 
   pinMode(PIN_VEXT, OUTPUT);
   digitalWrite(PIN_VEXT, HIGH);
   pinMode(PIN_ADC_CTRL, OUTPUT);
   digitalWrite(PIN_ADC_CTRL, HIGH);
 
   pinMode(PIN_BUTTON, INPUT_PULLUP);
   pinMode(PIN_USER, INPUT_PULLUP); // Инициализация кнопки USER
   pinMode(PIN_FB_LED, OUTPUT);
   pinMode(PIN_BIG_LED, OUTPUT);
   pinMode(PIN_BATTERY_LED, OUTPUT);
   
   digitalWrite(PIN_FB_LED, 0);
   analogWrite(PIN_BIG_LED, 0);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(300);
 
   updateStatusLed(true);
   analogWrite(PIN_BIG_LED, pwmledBrightness);
   digitalWrite(PIN_FB_LED, HIGH);
   digitalWrite(PIN_BATTERY_LED, HIGH);
   delay(1000);
   updateStatusLed(false);
   analogWrite(PIN_BIG_LED, 0);
   digitalWrite(PIN_FB_LED, LOW);
   digitalWrite(PIN_BATTERY_LED, LOW);
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
   delay(100);
 
   DEBUGln("DavayLoRa TX setup complete, waiting for 1-st buttonpress");
 } // end setup
 
 void loop() {
   checkReceive(); 
   
   if ((millis() - lastButtonTime) > DEBOUNCE_TIME) {
     processButton();
   } // end if
   
   processUserButton(); // Проверка 5-секундного удержания кнопки USER
   
   if (currentState == STATE_NORMAL && currButtonState) {
     if (millis() - buttonPressStartTime > 10000) {
       DEBUGln(F("Enter Preparation Mode!"));
       currentState = STATE_PREPARATION;
       prepModeTimer = millis(); 
       prepClickCount = 0;
       
       sendMessage(CMD_SIGNAL, false); 
       updateBIGLed(false);
       
       buttonPressedFirstTime = false; 
     } // end if
   } // end if
 
   if (currentState == STATE_NORMAL) {
     processPing();
   } else if (currentState == STATE_PREPARATION) {
     processPreparationMode();
   } // end if
   
   EVERY_MS(BATTERY_PERIOD) {
     if (measurebattery) {
       processBattery();
     } // end if
   } // end EVERY_MS
 } // end loop