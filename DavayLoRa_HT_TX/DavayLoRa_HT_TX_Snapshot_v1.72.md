# Project: DavayLoRa_HT_TX Snapshot V1.72

## File: .\src\Battery.cpp
```cpp
/**
 * @file Battery.cpp
 * @version 1.53
 * @brief Реализация подсистемы питания (TX)
 */
 #include "Battery.h"

 // Локальные макросы отладки
 #define DEBUG_ENABLE // Логирование выключено
 #ifdef DEBUG_ENABLE
 #define DEBUG(x) Serial.print(x)
 #define DEBUGln(x) Serial.println(x)
 #else
 #define DEBUG(x)
 #define DEBUGln(x)
 #endif
 
 bool isBatteryConnected = false;
 
 // Проверяет, подключена ли батарея, делая несколько замеров
 bool testBattery() { return batteryVoltageOK(5); } // конец функции testBattery
 
 bool batteryVoltageOK(byte tries) {
    float minV = 5.0, maxV = 0.0;
    for (byte i = 0; i < tries; i++) {
      float currentVBat = batteryVoltage();
      if (currentVBat < minV) minV = currentVBat; 
      if (currentVBat > maxV) maxV = currentVBat; 
      delay(150);
    } // конец цикла замеров
    return (maxV - minV) <= 0.05;
 } // конец функции batteryVoltageOK
 
 // Чтение реального напряжения через внутренний АЦП и делитель
 float batteryVoltage() {
    digitalWrite(PIN_ADC_CTRL, LOW); delay(10);
    float measuredvbat = analogRead(PIN_BATTERY_INTERNAL);
    digitalWrite(PIN_ADC_CTRL, HIGH);
    measuredvbat *= 3.3 * HELTEC_BATTERY_MULTIPLIER / 4095.0;
    return measuredvbat;
 } // конец функции batteryVoltage
 
 // Индикация заряда (от 1 до 5 вспышек в зависимости от напряжения)
 void showBatteryVoltage() {
    float voltage = batteryVoltage();
    if (voltage > BATTERY_VOLTAGE_1) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_2) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_3) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_4) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_5) flashBatteryLEDOnce(); 
 } // конец функции showBatteryVoltage
 
 // Индикация отсутствия батареи (длинная вспышка)
 void showNoBattery() { 
    digitalWrite(PIN_BATTERY_LED, 1); delay(2000); digitalWrite(PIN_BATTERY_LED, 0); delay(250); 
 } // конец функции showNoBattery
 
 void flashBatteryLEDOnce() { updateStatusLed(true); delay(250); updateStatusLed(false); delay(250); } // конец функции flashBatteryLEDOnce
 
 // Регулярная проверка. Если батарея села - уходим в сон
 void processBattery() { if (batteryVoltage() < BATTERY_MIN_VOLTAGE) stopWorking(); } // конец функции processBattery
 
 void stopWorking() { 
    DEBUGln(F("[STATE] BATTERY DEAD! Stopping..."));
    enterDeepSleep(); 
 } // конец функции stopWorking
```

---

## File: .\src\Battery.h
```cpp
/**
 * @file Battery.h
 * @version 1.53
 * @brief Подсистема питания и контроля батареи (TX)
 * Описание: Отвечает за проверку напряжения Li-Po аккумулятора и защиту от глубокого разряда.
 */
 #ifndef BATTERY_H
 #define BATTERY_H
 
 #include <Arduino.h>
 
 // --- Пороги напряжений ---
 #define BATTERY_MIN_VOLTAGE 3.5
 #define BATTERY_VOLTAGE_1 3.5
 #define BATTERY_VOLTAGE_2 3.6
 #define BATTERY_VOLTAGE_3 3.8
 #define BATTERY_VOLTAGE_4 3.9
 #define BATTERY_VOLTAGE_5 4.0
 
 // --- Пины и константы батареи ---
 #define PIN_BATTERY_LED 35     
 #define PIN_BATTERY_INTERNAL 1 
 #define PIN_ADC_CTRL 37        
 #define HELTEC_BATTERY_MULTIPLIER 4.9
 
 extern bool isBatteryConnected;
 
 // Внешние зависимости (функции, которые остались в main.cpp)
 extern void updateStatusLed(bool ledStatus);
 extern void enterDeepSleep();
 
 // Прототипы функций
 bool testBattery();
 bool batteryVoltageOK(byte tries);
 float batteryVoltage();
 void showBatteryVoltage();
 void showNoBattery();
 void flashBatteryLEDOnce();
 void processBattery();
 void stopWorking();
 
 #endif
```

---

## File: .\src\Config.cpp
```cpp
/**
 * @file Config.cpp
 * @version 1.64
 * @brief Реализация загрузки и сохранения настроек (TX)
 */
 #include "Config.h"

 // Локальные макросы отладки
 #define DEBUG_ENABLE // Логирование выключено
 #ifdef DEBUG_ENABLE
 #define DEBUG(x) Serial.print(x)
 #define DEBUGln(x) Serial.println(x)
 #else
 #define DEBUG(x)
 #define DEBUGln(x)
 #endif
 
 ConfigPacket rxSettings;
 Preferences preferences; // Объект для работы с NVS (сохранение настроек)
 
 byte workAddress = 0;                 
 bool measurebattery = true;           
 unsigned long batteryPeriod = 300000;    
 unsigned long wakeUpHoldTime = 2000;     
 unsigned long wakeUpReleaseWindow = 2000; 
 unsigned long stuckSleepTime = 10000; 
 unsigned long configTimeout = 600000; 
 unsigned long sleepLedDuration = 2000;   
 int maxPower = 20; // Максимальная мощность по умолчанию
 
 int pwmledBrightness = 35;            
 int fbledBrightness = 255;            
 unsigned long pingTimeout = 3000;     
 unsigned long bigTimeout = 3600000;   
 unsigned long execTimeout = 30000;    
 bool dynamicPower = true; // Адаптивная мощность включена
 int minPower = -9;        // Аппаратный минимум SX1262
 int servicePower = 18;    // Комфортная мощность для ближней связи
 
 unsigned long pingTimeoutRX = 9000;   
 
 // Чтение настроек из памяти при загрузке системы
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
    maxPower = preferences.getInt("maxPwr", 20);
    
    pwmledBrightness = preferences.getInt("bigLedBright", 35);
    fbledBrightness = preferences.getInt("fbLedBright", 255);
    pingTimeout = preferences.getULong("pingTimeout", 3000);
    bigTimeout = preferences.getULong("bigTimeout", 3600000);
    execTimeout = preferences.getULong("execTo", 30000); 
    dynamicPower = preferences.getBool("dynPwr", true);
    minPower = preferences.getInt("minPwr", -9);
    servicePower = preferences.getInt("srvPwr", 18);
    
    pingTimeoutRX = preferences.getULong("pingRx", 9000); 
    rxSettings.rxEnableBigLed = preferences.getBool("rxEnBigLed", true);
    rxSettings.rxPwmledBrightness = preferences.getInt("rxBigBright", 35);
    rxSettings.rxEnableBuzzer = preferences.getBool("rxEnBuzzer", false);
    rxSettings.rxBuzzerVolume = preferences.getInt("rxBuzVol", 255);
    rxSettings.rxCutoffTime = preferences.getULong("rxCutoff", 2000);
    rxSettings.maxPower = maxPower; // Синхронизация структуры
 
    preferences.end();
    DEBUGln(F("Config loaded."));
 } // конец функции loadConfig
 
 // Сохранение новых настроек после изменения их через Wi-Fi портал
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
    preferences.putInt("maxPwr", maxPower);
    
    preferences.putInt("bigLedBright", pwmledBrightness);
    preferences.putInt("fbLedBright", fbledBrightness);
    preferences.putULong("pingTimeout", pingTimeout);
    preferences.putULong("bigTimeout", bigTimeout);
    preferences.putULong("execTo", execTimeout); 
    preferences.putBool("dynPwr", dynamicPower);
    preferences.putInt("minPwr", minPower);
    preferences.putInt("srvPwr", servicePower);
    
    preferences.putULong("pingRx", pingTimeoutRX); 
    preferences.putBool("rxEnBigLed", rxSettings.rxEnableBigLed);
    preferences.putInt("rxBigBright", rxSettings.rxPwmledBrightness);
    preferences.putBool("rxEnBuzzer", rxSettings.rxEnableBuzzer);
    preferences.putInt("rxBuzVol", rxSettings.rxBuzzerVolume);
    preferences.putULong("rxCutoff", rxSettings.rxCutoffTime);
    
    preferences.end();
    
    // Синхронизируем структуру перед возможной отправкой
    rxSettings.maxPower = maxPower;
    
    DEBUGln(F("Config saved."));
 } // конец функции saveConfig
```

---

## File: .\src\Config.h
```cpp
/**
 * @file Config.h
 * @version 1.64
 * @brief Глобальные настройки и работа с энергонезависимой памятью NVS (TX)
 * Описание: Хранит все пользовательские настройки (таймауты, яркости, адреса, настройки мощности).
 */
 #ifndef CONFIG_H
 #define CONFIG_H
 
 #include <Arduino.h>
 #include <Preferences.h>
 
 // Структура пакета настроек для передачи на приемник (RX)
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
    int maxPower; // Ограничение максимальной мощности (0-22 дБм)
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
 extern int maxPower; // Максимальная мощность (для обеих плат)
 
 // --- Настройки пульта (TX) ---
 extern int pwmledBrightness;            
 extern int fbledBrightness;            
 extern unsigned long pingTimeout;     
 extern unsigned long bigTimeout;   
 extern unsigned long execTimeout;    
 extern bool dynamicPower; // Включение адаптивной мощности
 extern int minPower;      // Минимальный порог мощности для адаптации (-9 - 0)
 extern int servicePower;  // Мощность для сервисных команд (0 - 22)
 
 // --- Настройки приемника (для синхронизации) ---
 extern unsigned long pingTimeoutRX;   
 
 // Прототипы
 void loadConfig();
 void saveConfig();
 
 #endif
```

---

## File: .\src\main.cpp
```cpp
/**
 * @file main.cpp (TX) (TEST)
 * @version 1.72 Внедрена вычисляемая рабочая частота и число каналов 81
 * @brief Прошивка передатчика (Transmitter) для проекта DavayLoRa на базе Heltec Wireless Stick Lite V3
 * Описание: Ядро стейт-машины, логика переключения режимов и опроса кнопок.
 */

 #include <Arduino.h>
 #include <esp_sleep.h>
 #include <driver/rtc_io.h>
 #include <driver/gpio.h> // Добавлено для функций заморозки пинов (gpio_hold)
 #include <WiFi.h>        // Добавлено для принудительного отключения модема на старте
 
 #include "Battery.h" 
 #include "Config.h"  
 #include "WebPortal.h" 
 #include "RadioComm.h" 
 
 // ======================= АППАРАТНАЯ КОНФИГУРАЦИЯ =======================
 #define PIN_BUTTON 7           
 #define PIN_FB_LED 35          
 #define PIN_BIG_LED 41         
 #define PIN_USER 0             
 #define PIN_VEXT 36            
 
 // ======================= ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ И МАКРОСЫ =======================
 
 // Локальные макросы отладки
 #define DEBUG_ENABLE // Логирование ВКЛЮЧЕНО
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
 
 #define PING_FLASH 100             
 #define DEBOUNCE_TIME 100          
 
 unsigned long lastButtonTime;
 
 bool currButtonState;
 bool prevButtonState;
 bool buttonPressedFirstTime;
 
 unsigned long pingTimer;
 unsigned long pingFlashTimer;
 bool pingFlash;
 
 // --- ПЕРЕМЕННЫЕ АДАПТАЦИИ МОЩНОСТИ ---
 int currentTxPower = 22;      // Текущая мощность (перезаписывается при старте из Config)
 const int targetRssi = -95;   // Целевой уровень сигнала на стороне RX
 // --------------------------------------
 
 enum SystemState {
    STATE_NORMAL,          // Рабочий режим (передача сигналов)
    STATE_PREPARATION,     // Режим подготовки (ожидание серии кликов)
    STATE_CONFIG_STANDBY,  // Ожидание запуска Wi-Fi
    STATE_EXEC_CONFIG      // Режим переключения типа сигнала на лету
 };
 SystemState currentState = STATE_NORMAL;
 
 unsigned long buttonPressStartTime = 0; 
 unsigned long prepModeTimer = 0;        
 byte prepClickCount = 0;
 unsigned long lastPrepClickTime = 0;
 
 byte configClickCount = 0;
 unsigned long lastConfigClickTime = 0;
 
 bool configBlinkActive = false;
 unsigned long configBlinkStartTime = 0; 
 
 byte execClickCount = 0;
 unsigned long lastExecClickTime = 0;
 
 bool execBlinkActive = false;
 unsigned long execBlinkStartTime = 0; 
 
 unsigned long execModeTimer = 0; 
 
 // --- ПРОТОТИПЫ ---
 void updateStatusLed(bool ledStatus);
 void updateBIGLed(bool ledStatus);
 void flashStatusLed(byte times);
 void processConfigLed();
 void processExecConfigLed();
 void enterDeepSleep();
 void runWakeUpProtection(uint8_t wakeupPin);
 void sleepSystem();
 void processButton();
 void processUserButton();
 void processPreparationMode();
 void processConfigStandby();
 void processExecConfigStandby();
 void processPing();
 
 // ======================= ИНДИКАЦИЯ (LED) =======================
 
 void updateStatusLed(bool ledStatus) { 
    static int lastBrightness = -1;
    int newBrightness = ledStatus ? fbledBrightness : 0;
    
    if (lastBrightness != newBrightness) {
      analogWrite(PIN_FB_LED, newBrightness);
      lastBrightness = newBrightness;
    } // конец защиты от спама analogWrite
 } // конец функции updateStatusLed
 
 void updateBIGLed(bool ledStatus) { 
    static int lastBigBrightness = -1;
    int newBrightness = ledStatus * pwmledBrightness;
    
    if (lastBigBrightness != newBrightness) {
      analogWrite(PIN_BIG_LED, newBrightness); 
      lastBigBrightness = newBrightness;
    } // конец защиты от спама analogWrite
 } // конец функции updateBIGLed
 
 void flashStatusLed(byte times) {
    for (int i = 0; i < times; i++) {
      updateStatusLed(true); delay(100); updateStatusLed(false); delay(200);
    } // конец цикла вспышек
 } // конец функции flashStatusLed
 
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
 
 // ======================= УПРАВЛЕНИЕ ПИТАНИЕМ И СНОМ =======================
 
 void enterDeepSleep() {
    DEBUGln(F("[STATE] ---> ENTERING DEEP SLEEP"));
    if (isWifiActive) stopWiFiPortal(); 
    
    radio.sleep();
    SPI.end();
    
    // Симметричное отключение пинов SPI, как в RX, для устранения паразитных утечек тока
    pinMode(csPin, INPUT); pinMode(mosiPin, INPUT); pinMode(misoPin, INPUT);
    pinMode(sckPin, INPUT); pinMode(resetPin, INPUT); pinMode(busyPin, INPUT);
    pinMode(irqPin, INPUT);
    
    pinMode(PIN_VEXT, OUTPUT); digitalWrite(PIN_VEXT, HIGH); 
    pinMode(PIN_ADC_CTRL, OUTPUT); digitalWrite(PIN_ADC_CTRL, HIGH);
    
    // --- ИДЕАЛЬНЫЙ СОН: Заморозка пинов для исключения плавающих уровней ---
    gpio_hold_en((gpio_num_t)PIN_VEXT);
    gpio_hold_en((gpio_num_t)PIN_ADC_CTRL);
    gpio_deep_sleep_hold_en();
    // -----------------------------------------------------------------------
 
    pinMode(PIN_FB_LED, OUTPUT); digitalWrite(PIN_FB_LED, LOW);
    pinMode(PIN_BIG_LED, OUTPUT); digitalWrite(PIN_BIG_LED, LOW);
    pinMode(PIN_BATTERY_LED, OUTPUT); digitalWrite(PIN_BATTERY_LED, LOW);
    
    rtc_gpio_pullup_en((gpio_num_t)PIN_BUTTON);
    rtc_gpio_pulldown_dis((gpio_num_t)PIN_BUTTON);
    
    // --- ИСПРАВЛЕНИЕ БАГА: Защита от бесконечного просыпания при зажатой кнопке ---
    if (digitalRead(PIN_BUTTON) == LOW) {
      DEBUGln(F("[STATE] Button is STUCK. Sleeping with timer..."));
      esp_sleep_enable_timer_wakeup(stuckSleepTime * 1000ULL);
    } else {
      DEBUGln(F("[STATE] Normal sleep. Wakeup on EXT0 (Button). Good night!"));
      esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BUTTON, 0);
    }
    // ------------------------------------------------------------------------------
 
    esp_deep_sleep_start();
 } // конец функции enterDeepSleep
 
 void runWakeUpProtection(uint8_t wakeupPin) {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) enterDeepSleep(); 
    
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
      DEBUGln(F("[STATE] Woke up. Checking protection..."));
      updateStatusLed(true);
      unsigned long startHold = millis();
      while (millis() - startHold < wakeUpHoldTime) {
        if (digitalRead(wakeupPin) == HIGH) { 
          updateStatusLed(false); enterDeepSleep(); 
        } // конец проверки отпускания
        delay(10);
      } // конец цикла удержания
      
      unsigned long startReleaseWindow = millis();
      bool releasedInWindow = false;
      while (millis() - startReleaseWindow < wakeUpReleaseWindow) {
        updateStatusLed((millis() % 200) < 100); 
        if (digitalRead(wakeupPin) == HIGH) { 
          releasedInWindow = true; break; 
        } // конец проверки отпускания в окне
        delay(10);
      } // конец цикла окна отпускания
      
      if (!releasedInWindow) { 
        updateStatusLed(false); enterDeepSleep(); 
      } // конец условия ошибки просыпания
      updateStatusLed(false); 
    } // конец условия просыпания от кнопки
 } // конец функции runWakeUpProtection
 
 void sleepSystem() {
    DEBUGln(F("[ACTION] Sending Centralized SLEEP Command"));
    currentTxPower = servicePower; 
    radio.setOutputPower(currentTxPower);
    DEBUG(F("[RADIO] Send SLEEP (Power: ")); DEBUG(currentTxPower); DEBUGln(F(" dBm)"));
    commSession(CMD_SLEEP, 1, CMD_SLEEP_OK, 500, WORK_COMM_ATTEMPTS);
    
    updateStatusLed(true); delay(sleepLedDuration); updateStatusLed(false);
    enterDeepSleep();
 } // конец функции sleepSystem
 
 // ======================= ОБРАБОТКА ВВОДА (КНОПКИ) =======================
 
 void processButton() {
    prevButtonState = currButtonState;
    currButtonState = !digitalRead(PIN_BUTTON); 
    
    if (prevButtonState != currButtonState) {
      lastButtonTime = millis();
      
      if (currButtonState) { // Нажата
        DEBUGln(F("[ACTION] Main Button PRESSED"));
        if (currentState == STATE_NORMAL) {
          buttonPressStartTime = millis(); pingTimer = millis(); buttonPressedFirstTime = true;
          
          // Передача сигнала "ACTION!" на максимальной мощности
          currentTxPower = maxPower; 
          radio.setOutputPower(currentTxPower);
          DEBUG(F("[RADIO] Send ACTION (Power: ")); DEBUG(currentTxPower); DEBUGln(F(" dBm)"));
          if (commSession(CMD_SIGNAL, 1, CMD_SIGNAL_OK, 300, WORK_COMM_ATTEMPTS)) {
            updateStatusLed(true); updateBIGLed(true);    
            DEBUG(F("[RADIO] RX RSSI: ")); DEBUGln(radio.getRSSI());
          } else { updateBIGLed(false); flashStatusLed(2); }
        } else if (currentState == STATE_PREPARATION) {
          prepClickCount++; lastPrepClickTime = millis(); prepModeTimer = millis(); 
        } else if (currentState == STATE_CONFIG_STANDBY) {
          configClickCount++; lastConfigClickTime = millis();
        } else if (currentState == STATE_EXEC_CONFIG) {
          execClickCount++; lastExecClickTime = millis(); execModeTimer = millis(); 
        } // конец ветвления по состояниям
      } else { // Отпущена
        DEBUGln(F("[ACTION] Main Button RELEASED"));
        if (currentState == STATE_NORMAL) {
          // Отмена сигнала "ACTION!" на максимальной мощности
          currentTxPower = maxPower;
          radio.setOutputPower(currentTxPower);
          sendMessage(CMD_SIGNAL, false); updateStatusLed(false); updateBIGLed(false);
        } // конец условия для нормального режима
      } // конец условия проверки нажатия
    } // конец условия изменения состояния
 } // конец функции processButton
 
 void processUserButton() {
    static unsigned long userButtonTimer = 0;
    static bool prevUserState = HIGH;
    bool currUserState = digitalRead(PIN_USER);
    
    // Отслеживание физического нажатия и отпускания кнопки
    if (currUserState != prevUserState) {
      if (currUserState == LOW) {
        DEBUGln(F("[ACTION] USER Button PRESSED"));
        userButtonTimer = millis();
      } else {
        DEBUGln(F("[ACTION] USER Button RELEASED"));
        userButtonTimer = 0;
      }
      prevUserState = currUserState;
    } // конец отслеживания состояний
    
    // Таймер долгого удержания (5 секунд)
    if (currUserState == LOW && userButtonTimer > 0) {
      if (millis() - userButtonTimer > 5000) { 
        DEBUGln(F("[ACTION] USER button held 5s -> Local Sleep"));
        sleepSystem(); 
        userButtonTimer = 0; // Сброс таймера для предотвращения спама
      } // конец условия зажатия 5 сек
    } // конец проверки зажатия
 } // конец функции processUserButton
 
 // ======================= СТЕЙТ-МАШИНА И БИЗНЕС-ЛОГИКА =======================
 
 // Режим подготовки. Ожидает определенное количество кликов для перехода в другие режимы.
 void processPreparationMode() {
    EVERY_MS(166) {
      static bool prepLedState = false;
      prepLedState = !prepLedState;
      updateStatusLed(prepLedState);
    } // конец интервала мигания
 
    // Проверка завершения ввода серии кликов
    if (prepClickCount > 0 && (millis() - lastPrepClickTime > 600)) {
      currentTxPower = servicePower;
      radio.setOutputPower(currentTxPower);
      if (prepClickCount == 2) { 
        // 2 клика: Выключение приборов (Сон)
        sleepSystem(); 
      } else if (prepClickCount == 3) {
        // 3 клика: Смена типа сигнала актеру (свет/вибро)
        if (commSession(CMD_EXEC_CONFIG, 1, CMD_EXEC_CONFIG_OK, 500, WORK_COMM_ATTEMPTS)) {
          DEBUGln(F("[STATE] ---> STATE_EXEC_CONFIG"));
          currentState = STATE_EXEC_CONFIG; execModeTimer = millis(); 
          execBlinkActive = true; execBlinkStartTime = millis(); execClickCount = 0;
        }
      } else if (prepClickCount == 4) {
        // 4 клика: Полная настройка через телефон (Wi-Fi Портал)
        if (commSession(CMD_CONFIG, 1, CMD_CONFIG_OK, 500, WORK_COMM_ATTEMPTS)) {
          DEBUGln(F("[STATE] ---> STATE_CONFIG_STANDBY"));
          currentState = STATE_CONFIG_STANDBY; pingTimer = millis();
          configBlinkActive = true; configBlinkStartTime = millis(); configClickCount = 0;
        }
      } else {
        DEBUGln(F("[STATE] ---> STATE_NORMAL (Invalid clicks)"));
        currentState = STATE_NORMAL; updateStatusLed(false);
      } // конец разбора количества кликов
      prepClickCount = 0; 
    } // конец условия окончания кликов
 
    // Если пользователь ничего не нажал в течение 10 секунд - возврат в рабочий режим
    if (millis() - prepModeTimer > 10000 && currentState == STATE_PREPARATION) {
      DEBUGln(F("[STATE] Prep mode timeout -> Exit to STATE_NORMAL"));
      currentState = STATE_NORMAL; updateStatusLed(false);
    } // конец условия таймаута режима подготовки
 } // конец функции processPreparationMode
 
 // Ожидание в режиме настройки (до включения Wi-Fi)
 void processConfigStandby() {
    processConfigLed();
 
    if (configClickCount > 0 && (millis() - lastConfigClickTime > 600)) {
      if (configClickCount == 2) {
        // 2 клика: Возврат в рабочий режим
        if (isWifiActive) stopWiFiPortal();
        currentTxPower = servicePower;
        radio.setOutputPower(currentTxPower);
        commSession(CMD_NORMAL_MODE, 1, CMD_NORMAL_MODE_OK, 500, WORK_COMM_ATTEMPTS);
        DEBUGln(F("[STATE] ---> STATE_NORMAL (Exited Config via button)"));
        currentState = STATE_NORMAL; updateStatusLed(false);
      } else if (configClickCount == 1) { 
        // 1 клик: Активация точки доступа Wi-Fi
        if (!isWifiActive) startWiFiPortal(); 
      } // конец разбора кликов настройки
      configClickCount = 0;
    } // конец условия обработки кликов
 
    // Keep-alive для удержания приемника в режиме настройки
    if ((millis() - pingTimer) > pingTimeout) {
      currentTxPower = servicePower;
      radio.setOutputPower(currentTxPower);
      if (commSession(CMD_CONFIG, 1, CMD_CONFIG_OK, 500, WORK_COMM_ATTEMPTS)) {
        configBlinkActive = true; configBlinkStartTime = millis(); 
      } else {
        DEBUGln(F("[ERROR] Config Keepalive Failed! -> Exit to STATE_NORMAL"));
        if (isWifiActive) stopWiFiPortal(); 
        currentState = STATE_NORMAL; updateStatusLed(false); flashStatusLed(2); 
      } // конец условия отсутствия ответа от RX
      pingTimer = millis(); 
    } // конец проверки интервала пинга
 } // конец функции processConfigStandby
 
 // Ожидание в режиме переключения исполнительных устройств
 void processExecConfigStandby() {
    processExecConfigLed();
 
    if (millis() - execModeTimer > execTimeout) {
      DEBUGln(F("[STATE] Exec Config Inactivity Timeout -> Exit to STATE_NORMAL"));
      currentTxPower = servicePower;
      radio.setOutputPower(currentTxPower);
      commSession(CMD_NORMAL_MODE, 1, CMD_NORMAL_MODE_OK, 500, WORK_COMM_ATTEMPTS);
      currentState = STATE_NORMAL; updateStatusLed(false); execClickCount = 0;
    } // конец условия таймаута бездействия пользователя
 
    if (execClickCount > 0 && (millis() - lastExecClickTime > 600)) {
      currentTxPower = servicePower;
      radio.setOutputPower(currentTxPower);
      if (execClickCount == 2) {
        // 2 клика: Выход и сохранение
        commSession(CMD_NORMAL_MODE, 1, CMD_NORMAL_MODE_OK, 500, WORK_COMM_ATTEMPTS);
        DEBUGln(F("[STATE] ---> STATE_NORMAL (Exited Exec Config via button)"));
        currentState = STATE_NORMAL; updateStatusLed(false);
      } else if (execClickCount == 1) {
        // 1 клик: Цикличное переключение режимов света/вибро на приемнике
        if (commSession(CMD_CYCLE_EXEC, 1, CMD_CYCLE_EXEC_OK, 500, WORK_COMM_ATTEMPTS)) {
           rxSettings.rxEnableBigLed = (rcvData & 0x01);
           rxSettings.rxEnableBuzzer = (rcvData & 0x02) >> 1;
           saveConfig();
        } else { flashStatusLed(2); } // конец условия ошибки цикла
      } // конец разбора кликов режима управления
      execClickCount = 0;
    } // конец условия обработки кликов
 
    // Keep-alive
    if ((millis() - pingTimer) > pingTimeout) {
      currentTxPower = servicePower;
      radio.setOutputPower(currentTxPower);
      if (commSession(CMD_EXEC_CONFIG, 1, CMD_EXEC_CONFIG_OK, 500, WORK_COMM_ATTEMPTS)) {
        execBlinkActive = true; execBlinkStartTime = millis(); 
      } else {
        DEBUGln(F("[ERROR] Exec Config Keepalive Failed! -> Exit to STATE_NORMAL"));
        currentState = STATE_NORMAL; updateStatusLed(false); flashStatusLed(2); 
      } // конец условия ошибки связи
      pingTimer = millis(); 
    } // конец проверки интервала пинга
 } // конец функции processExecConfigStandby
 
 // Постоянный контроль качества связи (Heartbeat) в рабочем режиме
 void processPing() {
    if (!buttonPressedFirstTime) return; // Пинг начинается только после первого клика
    
    if (pingFlash) {
      if ((millis() - pingFlashTimer) > PING_FLASH) {
        pingFlash = false; updateStatusLed(currButtonState);
      } // конец условия гашения вспышки
    } else if ((millis() - pingTimer) > pingTimeout) {
      
      // Если динамическая мощность выключена - форсируем максимум
      if (!dynamicPower) {
        currentTxPower = maxPower;
      }
 
      // Упаковка мощности (currentTxPower + 10) и состояния кнопки (бит 7)
      byte safePower = (byte)(constrain(currentTxPower, -9, 22) + 10);
      byte packedData = (currButtonState << 7) | (safePower & 0x7F);
      
      radio.setOutputPower(currentTxPower);
      DEBUG(F("[PING] Tx Power: ")); DEBUG(currentTxPower); DEBUGln(F(" dBm"));
      
      if (commSession(CMD_PING, packedData, CMD_PING_OK, 500, WORK_COMM_ATTEMPTS)) {
        updateStatusLed(!currButtonState); pingFlash = true;
        pingFlashTimer = millis(); pingTimer = millis();
        
        if (dynamicPower) {
            // Распаковка RSSI из ответа
            byte rssiRaw = rcvData & 0x7F;
            int rxRssi = -(int)(rssiRaw + 30);
            
            // Изменение: Разделен лог RSSI на локальный и удаленный для удобства диагностики
            DEBUG(F("[PING] Response RSSI: ")); DEBUGln(radio.getRSSI());
            DEBUG(F("[PING] Loopback RSSI: ")); DEBUGln(rxRssi);
            
            // Вычисление Дельты и Адаптация мощности
            int pathLoss = currentTxPower - rxRssi;
            int idealPower = targetRssi + pathLoss;
            
            if (idealPower > currentTxPower) {
              currentTxPower = idealPower; // Fast UP
            } else if (idealPower < currentTxPower) {
              // Безопасный спуск по алгоритму половины пути
              int safeTarget = idealPower;
              if (safeTarget < minPower) safeTarget = minPower; 
              int nextPower = (currentTxPower + safeTarget) / 2;
              if (currentTxPower - nextPower < 1) nextPower = currentTxPower - 1;
              currentTxPower = nextPower;
            } // конец проверки направления мощности
        } else {
            DEBUGln(F("[PING] Dynamic Power OFF. Using maxPower."));
            currentTxPower = maxPower;
        } // конец проверки флага dynamicPower
        
        currentTxPower = constrain(currentTxPower, minPower, maxPower);
        DEBUG(F("[PING] Adaptive Next Power: ")); DEBUG(currentTxPower); DEBUGln(F(" dBm"));
        
      } else { 
        flashStatusLed(2); 
        currentTxPower = maxPower; // Panic Mode: возврат на максимум при потере связи
        DEBUGln(F("[PING] Link Lost! Reset to maxPower"));
      } // Ошибка связи: двойная вспышка
    } // конец проверки таймера пинга
    
    if ((millis() - lastButtonTime) > bigTimeout) {
      flashStatusLed(3); buttonPressedFirstTime = false;
    } // конец условия таймаута неактивности
 } // конец функции processPing
 
 // ======================= ОСНОВНЫЕ ФУНКЦИИ (SETUP & LOOP) =======================
 
 void setup() {
    // --- СНЯТИЕ ЗАМОРОЗКИ ПИНОВ ПОСЛЕ СНА ---
    gpio_hold_dis((gpio_num_t)PIN_VEXT);
    gpio_hold_dis((gpio_num_t)PIN_ADC_CTRL);
    gpio_deep_sleep_hold_dis();
    // ----------------------------------------
 
    // --- ОТКЛЮЧЕНИЕ RF-МОДЕМА НА СТАРТЕ ДЛЯ ЭКОНОМИИ ЭНЕРГИИ ---
    WiFi.mode(WIFI_OFF);
    // -----------------------------------------------------------
 
 #ifdef DEBUG_ENABLE
    Serial.begin(115200);
    while (!Serial); 
 #endif
 
    // Изменение: Форматирование стартового лога по правилам (10 символов '=', название, модуль, версия, макросы времени)
    DEBUGln(F("========== DavayLoRa TX v1.68 =========="));
    DEBUG(F("[INFO] Compiled: ")); DEBUG(__DATE__); DEBUG(F(" ")); DEBUGln(__TIME__);
    
    DEBUGln(F("[STATE] Initializing GPIO pins..."));
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_USER, INPUT_PULLUP); 
    pinMode(PIN_FB_LED, OUTPUT);
    updateStatusLed(false);
 
    DEBUGln(F("[STATE] Loading NVS config..."));
    loadConfig();
    
    // Синхронизация стартовой мощности с лимитом из конфигурации
    currentTxPower = maxPower;
 
    DEBUGln(F("[STATE] Running wake-up protection..."));
    runWakeUpProtection(PIN_BUTTON);
    delay(2000);
 
    DEBUG(F("Work Channel/Address: ")); DEBUGln(workAddress);
    DEBUG(F("TX BIG Brightness: ")); DEBUGln(pwmledBrightness);
    DEBUG(F("TX FB Brightness: ")); DEBUGln(fbledBrightness);
    DEBUG(F("Dynamic Power: ")); DEBUGln(dynamicPower ? "ON" : "OFF");
    DEBUG(F("Max Power Limit: ")); DEBUGln(maxPower);
    DEBUG(F("Min Power Limit: ")); DEBUGln(minPower);
    DEBUG(F("Service Power: ")); DEBUGln(servicePower);
    
    DEBUGln(F("[STATE] ---> STATE_NORMAL (Boot)"));
 
    DEBUGln(F("[STATE] Powering up peripherals (VEXT/ADC)..."));
    pinMode(PIN_VEXT, OUTPUT); digitalWrite(PIN_VEXT, HIGH);
    pinMode(PIN_ADC_CTRL, OUTPUT); digitalWrite(PIN_ADC_CTRL, HIGH);
    pinMode(PIN_BIG_LED, OUTPUT); analogWrite(PIN_BIG_LED, 0);
    pinMode(PIN_BATTERY_LED, OUTPUT); digitalWrite(PIN_BATTERY_LED, LOW);
 
    DEBUGln(F("[ACTION] Testing LEDs..."));
    updateStatusLed(true); analogWrite(PIN_BIG_LED, pwmledBrightness); digitalWrite(PIN_BATTERY_LED, HIGH); delay(1000);
    updateStatusLed(false); analogWrite(PIN_BIG_LED, 0); digitalWrite(PIN_BATTERY_LED, LOW); delay(1000);
 
    DEBUGln(F("[ACTION] Checking battery status..."));
    if (measurebattery) {
      isBatteryConnected = testBattery(); 
      if (isBatteryConnected) { 
        DEBUGln(F("[ACTION] Battery connected. Showing voltage (2 times)."));
        processBattery(); delay(500); showBatteryVoltage(); delay(2000); showBatteryVoltage(); delay(500);
      } else {
        DEBUGln(F("[ACTION] No battery detected."));
        showNoBattery(); delay(500); 
      } // конец условия обработки подключенной батареи
    } else {
      DEBUGln(F("[ACTION] Battery measurement disabled in config."));
    } // конец условия проверки включения замеров батареи
 
    DEBUGln(F("[STATE] Initializing LoRa radio..."));
    SPI.begin(sckPin, misoPin, mosiPin, csPin);
    workFrequency = 434000000 + ( (workAddress % MAX_ADDRESS) * 200000 );
    int state = radio.begin(workFrequency / 1000000.0);
    if (state != RADIOLIB_ERR_NONE) while (true) { flashStatusLed(6); delay(4000); }
    setLoRaParams();
    
    // Максимальная чувствительность приемника
    radio.setRxBoostedGainMode(true);
    
    radio.setDio1Action(setFlag);
    radio.startReceive();
    
    // --- СНИЖЕНИЕ ЧАСТОТЫ ПРОЦЕССОРА ТОЛЬКО ПОСЛЕ ПОЛНОЙ ИНИЦИАЛИЗАЦИИ ---
    // Спасает стек ipc1 от переполнения на старте!
    setCpuFrequencyMhz(80);
    // ---------------------------------------------------------------------
    
    // Изменение: Финальный лог setup по правилам
    DEBUGln(F("[ACTION] SETUP COMPLETE "));
 } // конец функции setup
 
 void loop() {
    checkReceive(); 
    
    if ((millis() - lastButtonTime) > DEBOUNCE_TIME) processButton();
    processUserButton(); 
    
    // Удержание кнопки 10 секунд для входа в режим подготовки
    if (currentState == STATE_NORMAL && currButtonState) {
      if (millis() - buttonPressStartTime > 10000) {
        DEBUGln(F("[STATE] ---> STATE_PREPARATION"));
        currentState = STATE_PREPARATION; prepModeTimer = millis(); prepClickCount = 0;
        updateBIGLed(false); buttonPressedFirstTime = false; 
      } // конец условия перехода в подготовку
    } // конец условия зажатия кнопки
 
    // Маршрутизация логики в зависимости от текущего состояния
    if (currentState == STATE_NORMAL) processPing();
    else if (currentState == STATE_PREPARATION) processPreparationMode();
    else if (currentState == STATE_CONFIG_STANDBY) processConfigStandby();
    else if (currentState == STATE_EXEC_CONFIG) processExecConfigStandby();
    
    // Обработка Wi-Fi портала, если он запущен
    if (isWifiActive) {
      dnsServer.processNextRequest();
      server.handleClient();
      
      if (exitConfigRequested || (millis() - wifiStartTime > configTimeout)) {
        if (millis() - wifiStartTime > configTimeout) DEBUGln(F("[STATE] WiFi Portal Timeout -> Exit to STATE_NORMAL"));
        else DEBUGln(F("[STATE] ---> STATE_NORMAL (Exited via Web UI)"));
        stopWiFiPortal();
        currentState = STATE_NORMAL; updateStatusLed(false);
      } // конец условия выхода из WiFi
    } // конец условия работы WiFi
    
    EVERY_MS(batteryPeriod) { 
      if (measurebattery && isBatteryConnected) processBattery(); 
    } // конец интервала проверки батареи
 
    // --- FREERTOS YIELD: Передача управления ОС для охлаждения процессора и экономии батареи ---
    delay(1); 
 } // конец функции loop
```

---

## File: .\src\RadioComm.cpp
```cpp
/**
 * @file RadioComm.cpp
 * @version 1.53
 * @brief Реализация радиосвязи SX1262 (TX)
 */
 #include "RadioComm.h"
 #include "Config.h"
 
 // Локальные макросы отладки
 #define DEBUG_ENABLE // Логирование выключено
 #ifdef DEBUG_ENABLE
 #define DEBUG(x) Serial.print(x)
 #define DEBUGln(x) Serial.println(x)
 #else
 #define DEBUG(x)
 #define DEBUGln(x)
 #endif
 
 extern const int sckPin = 9;
 extern const int misoPin = 11;
 extern const int mosiPin = 10;
 extern const int csPin = 8;
 extern const int resetPin = 12;
 extern const int irqPin = 14;
 extern const int busyPin = 13;
 
 SX1262 radio = new Module(csPin, irqPin, resetPin, busyPin);
 
 byte sndCmd = CMD_PING;
 byte sndData;
 bool wasReceived = false;
 byte cmdExpected = CMD_PING_OK;
 byte rcvAddress = 0;
 byte rcvCmd = 0;
 byte rcvData;
 unsigned long workFrequency = WORK_FREQUENCY;
 
 long lastSendTime = 0;
 int lastRSSI;
 float lastSNR;
 unsigned long lastTurnaround = DEFAULT_TURNAROUND;
 long lastFrequencyError;
 
 volatile bool receivedFlag = false; 
 
//  // Список рабочих частот. Выбор частоты зависит от адреса устройства.
//  unsigned long workingFrequency[MAX_ADDRESS] = {
//     434000000, 434120000, 434240000, 433820000, 433700000, 433940000, 434030000,
//     434150000, 434270000, 433850000, 433730000, 433970000, 434060000, 434180000,
//     433880000, 433760000, 434090000, 434210000, 433910000, 433790000,
//  };
// теперь будет вычисляться по формуле: 434000000 + (адрес * 20000)

 #if defined(ESP8266) || defined(ESP32)
    ICACHE_RAM_ATTR
 #endif
 void setFlag(void) {
    receivedFlag = true;
 } // конец функции прерывания setFlag
 
 void transmitPacket(byte* payload, size_t size) {
    DEBUG(F("[RADIO] >>> TX Packet [Size: ")); DEBUG(size); DEBUG(F("]: "));
    for (size_t i = 0; i < size; i++) { DEBUG(payload[i]); DEBUG(F(" ")); }
    DEBUGln();
 
    int state = radio.transmit(payload, size);
    if (state != RADIOLIB_ERR_NONE) {
      DEBUG(F("[RADIO] >>> Transmit failed, code: ")); DEBUGln(state);
    } // конец проверки ошибки передачи
    
    lastSendTime = millis();
    receivedFlag = false; 
    radio.startReceive(); // Сразу переходим обратно в режим приема
 } // конец функции transmitPacket
 
 // Стандартная отправка 3-байтового пакета (Адрес, Команда, Данные)
 void sendMessage(byte msgCmd, byte sndData) {
    byte payload[3] = {workAddress, msgCmd, sndData};
    transmitPacket(payload, 3);
 } // конец функции sendMessage
 
 void checkReceive() {
    if (receivedFlag) {
      receivedFlag = false;
      byte payload[256];
      int state = radio.readData(payload, sizeof(payload));
      
      if (state == RADIOLIB_ERR_NONE) {
        int packetSize = radio.getPacketLength();
        onReceive(payload, packetSize);
      } // конец проверки успешного приема
      radio.startReceive();
    } // конец проверки флага прерывания
 } // конец функции checkReceive
 
 // Сессия связи: отправка команды и ожидание подтверждения (с повторными попытками)
 bool commSession(byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes) {
    DEBUG(F("[RADIO] Starting CommSession for CMD: ")); DEBUGln(msgCmd);
    wasReceived = false;
    cmdExpected = expectedReply; 
    pingTimer = millis();
    unsigned long lastSend = millis() - waitMilliseconds; 
    
    do {
      checkReceive();
      if (millis() - lastSend >= waitMilliseconds) {
        sendMessage(msgCmd, sndData);
        lastSend = millis();
        doTimes--;
      } // конец интервальной отправки
    } while ((doTimes > 0) && (!wasReceived)); // конец цикла попыток
    
    pingTimer = millis();
    if (wasReceived) { DEBUGln(F("[RADIO] CommSession SUCCESS")); }
    else { DEBUGln(F("[RADIO] CommSession FAILED (Timeout/No Reply)")); }
    return wasReceived; 
 } // конец функции commSession
 
 // Передача структуры настроек на приемник
 bool syncConfigToRX() {
    DEBUGln(F("[RADIO] --- Syncing Config Struct to RX ---"));
    DEBUG(F("workAddress: ")); DEBUGln(rxSettings.workAddress);
    DEBUG(F("rxPwmledBrightness: ")); DEBUGln(rxSettings.rxPwmledBrightness);
    DEBUG(F("rxBuzzerVolume: ")); DEBUGln(rxSettings.rxBuzzerVolume);
 
    wasReceived = false;
    cmdExpected = CMD_SYNC_CONFIG_OK;
    
    byte txBuffer[sizeof(ConfigPacket) + 2];
    txBuffer[0] = workAddress; 
    txBuffer[1] = CMD_SYNC_CONFIG;
    memcpy(&txBuffer[2], &rxSettings, sizeof(ConfigPacket));
 
    for (int i=0; i < 3; i++) {
      transmitPacket(txBuffer, sizeof(txBuffer));
 
      unsigned long startWait = millis();
      while (millis() - startWait < 500) { 
        checkReceive();
        if (wasReceived) {
          DEBUGln(F("[RADIO] --- RX Confirmed Sync! ---"));
          return true;
        } // конец условия приема подтверждения
      } // конец цикла ожидания ответа
    } // конец цикла попыток синхронизации
    DEBUGln(F("[RADIO] --- RX Sync Timeout! ---"));
    return false;
 } // конец функции syncConfigToRX
 
 // Базовые параметры LoRa: Максимальная мощность и дальнобойные настройки
 void setLoRaParams() {
    DEBUGln("[RADIO] setLoRaParams()");
    radio.setOutputPower(20);
    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(8);
    radio.setCodingRate(5);
    radio.setPreambleLength(8);     
    radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE); 
 } // конец функции setLoRaParams
 
 void onReceive(byte* payload, int packetSize) {
    DEBUG(F("[RADIO] <<< RX Packet [Size: ")); DEBUG(packetSize); DEBUG(F("]: "));
    for (int i = 0; i < packetSize; i++) { DEBUG(payload[i]); DEBUG(F(" ")); }
    DEBUGln();
 
    if (packetSize != 3) {
      DEBUGln(F("\t[!] Invalid packet size! Expected 3 bytes."));
      return;
    } // конец проверки длины пакета
 
    rcvAddress = payload[0];
    if ((rcvAddress != workAddress) && rcvAddress != 255) { 
      DEBUGln(F("\t[!] Ignored: Wrong address"));
      return;
    } // конец проверки адреса
 
    rcvCmd = payload[1];
    if (rcvCmd != cmdExpected) {
      DEBUGln("\t[!] Invalid Reply: " + String(rcvCmd) + F(", Expected: ") + String(cmdExpected));
      return;
    } // конец проверки команды
    rcvData = payload[2];
 
    lastTurnaround = millis() - lastSendTime;
    lastRSSI = radio.getRSSI();
    lastSNR = radio.getSNR();
    
    DEBUGln(("\tTurnaround: ") + String(lastTurnaround));
    wasReceived = true; 
 } // конец функции onReceive
```

---

## File: .\src\RadioComm.h
```cpp
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
 #define MAX_ADDRESS 81
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
 
 //extern unsigned long workingFrequency[MAX_ADDRESS];
 
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
```

---

## File: .\src\webpage.h
```cpp
/**
 * @file webpage.h
 * @version 1.71 (Добавлен справочный вывод рабочей частоты %FREQ_MHZ%)
 * @brief HTML-интерфейс для Captive Portal (TX)
 */

 #ifndef WEBPAGE_H
 #define WEBPAGE_H
 
 #include <Arduino.h>
 
 const char index_html[] PROGMEM = R"rawliteral(
 <!DOCTYPE HTML>
 <html lang="ru">
 <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>DavayLoRa Config</title>
    <style>
      body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #121212; color: #ffffff; padding: 15px; max-width: 600px; margin: 0 auto; }
      h2 { text-align: center; color: #4CAF50; margin-bottom: 10px; font-size: 24px;}
      .timer { text-align: center; color: #ff9800; font-size: 18px; margin-bottom: 20px; font-weight: bold; }
      fieldset { border: 1px solid #4CAF50; border-radius: 8px; margin-bottom: 25px; padding: 20px; background: #1e1e1e; }
      legend { color: #4CAF50; font-weight: bold; font-size: 18px; padding: 0 10px; }
      label { display: block; margin-top: 15px; margin-bottom: 5px; color: #cccccc; font-size: 14px; }
      input[type="number"] { width: 100%; padding: 12px; border-radius: 6px; border: 1px solid #444; background: #2a2a2a; color: #fff; box-sizing: border-box; font-size: 16px; transition: border 0.3s; }
      input[type="number"]:focus { border-color: #4CAF50; outline: none; }
      .checkbox-container { display: flex; align-items: center; margin-top: 15px; background: #2a2a2a; padding: 12px; border-radius: 6px; }
      input[type="checkbox"] { transform: scale(1.5); margin: 0 10px 0 5px; accent-color: #4CAF50; }
      .checkbox-container span { font-size: 15px; color: #fff; }
      .buttons-container { display: flex; flex-direction: column; gap: 15px; margin-bottom: 30px; margin-top: 10px; }
      input[type="submit"] { background-color: #4CAF50; color: white; padding: 16px; border: none; border-radius: 6px; cursor: pointer; width: 100%; font-size: 18px; font-weight: bold; transition: background 0.3s; box-shadow: 0 4px 6px rgba(0,0,0,0.3); box-sizing: border-box; }
      input[type="submit"]:hover { background-color: #45a049; }
      .cancel-btn { background-color: #f44336; color: white; text-decoration: none; padding: 16px; display: flex; align-items: center; justify-content: center; border-radius: 6px; width: 100%; font-size: 18px; font-weight: bold; transition: background 0.3s; box-shadow: 0 4px 6px rgba(0,0,0,0.3); box-sizing: border-box; }
      .cancel-btn:hover { background-color: #d32f2f; }
    </style>
    <script>
      let timeLeft = %TIME_LEFT%;
      function updateTimer() {
        if (timeLeft <= 0) {
          document.getElementById('timeDisplay').innerText = "00:00";
          alert("⏳ Время конфигурации истекло!");
          window.location.reload();
          return;
        }
        let m = Math.floor(timeLeft / 60).toString().padStart(2, '0');
        let s = (timeLeft % 60).toString().padStart(2, '0');
        document.getElementById('timeDisplay').innerText = m + ":" + s;
        timeLeft--;
        setTimeout(updateTimer, 1000);
      }
      window.onload = updateTimer;
    </script>
 </head>
 <body>
    <h2>⚙️ Настройка DavayLoRa</h2>
    <div class="timer">⏳ До автовыхода: <span id="timeDisplay">--:--</span></div>
    <div style="text-align: center; color: #4CAF50; font-size: 16px; margin-bottom: 20px; font-weight: bold;">📡 Рабочая частота: %FREQ_MHZ% МГц</div>

    <form action="/save" method="POST">
      <fieldset>
        <legend>🌍 Общие настройки</legend>
        <label title="434-450 МГц, шаг 0.2">Рабочий канал (0-80):</label>
        <input type="number" name="workAddress" value="%ADDR%" min="0" max="80" required>

        <label title="По умолчанию: 20 dBm">Максимальная мощность (0-22 дБм):</label>
        <input type="number" name="maxPower" value="%MAX_PWR%" min="0" max="22" required>
        
        <div class="checkbox-container">
          <input type="checkbox" name="measurebattery" value="1" %BAT_CHK% title="По умолчанию: Включено">
          <span>Включить проверку батареи</span>
        </div>
        <label title="По умолчанию: 300000 мс (5 минут)">Период проверки батареи (мс):</label>
        <input type="number" name="batteryPeriod" value="%BAT_PER%" min="10000" step="1000" required>
        <label title="По умолчанию: 2000 мс">Время удержания при вкл. (мс):</label>
        <input type="number" name="wakeUpHoldTime" value="%WK_HOLD%" min="100" step="100" required>
        <label title="По умолчанию: 2000 мс">Окно отпускания при вкл. (мс):</label>
        <input type="number" name="wakeUpReleaseWindow" value="%WK_REL%" min="100" step="100" required>
        <label title="По умолчанию: 10000 мс (10 сек)">Таймаут заклинивания вкл. (мс):</label>
        <input type="number" name="stuckSleepTime" value="%STUCK_SL%" min="1000" step="1000" required>
        <label title="По умолчанию: 600000 мс (10 минут)">Таймаут настроек (мс):</label>
        <input type="number" name="configTimeout" value="%CONF_TO%" min="60000" step="10000" required>
        <label title="По умолчанию: 2000 мс">Индикация выключения (мс):</label>
        <input type="number" name="sleepLedDuration" value="%SLP_LED%" min="100" step="100" required>
      </fieldset>
 
      <fieldset>
        <legend>🔘 Пульт (TX)</legend>
        <div class="checkbox-container">
          <input type="checkbox" name="dynamicPower" value="1" %DYN_PWR% title="По умолчанию: Включено">
          <span>Переменная мощность (Адаптация)</span>
        </div>
        <label title="По умолчанию: -9 дБм">Минимальная мощность (-9...0 дБм):</label>
        <input type="number" name="minPower" value="%MIN_PWR%" min="-9" max="0" required>
        <label title="По умолчанию: 18 дБм">Мощность настройки (0-22 дБм):</label>
        <input type="number" name="servicePower" value="%SRV_PWR%" min="0" max="22" required>
        
        <label title="По умолчанию: 35">Яркость LED (0-255):</label>
        <input type="number" name="pwmledBrightness" value="%TX_BIG_LED%" min="0" max="255" required>
        <label title="По умолчанию: 255">Яркость кнопки (0-255):</label>
        <input type="number" name="fbledBrightness" value="%TX_FB_LED%" min="0" max="255" required>
        <label title="По умолчанию: 3000 мс">Таймаут Пинга (мс):</label>
        <input type="number" name="pingTimeout" value="%TX_PING%" min="1000" step="100" required>
        <label title="По умолчанию: 3600000 мс (1 час)">Таймаут отключения Пинга (мс):</label>
        <input type="number" name="bigTimeout" value="%TX_BIG_TO%" min="10000" step="1000" required>
        <label title="По умолчанию: 30000 мс (30 сек)">Таймаут настроек сигнала (мс):</label>
        <input type="number" name="execTimeout" value="%TX_EXEC_TO%" min="10000" step="1000" required>
      </fieldset>
 
      <fieldset>
        <legend>💡🔔 Приёмник (RX)</legend>
        <div class="checkbox-container">
          <input type="checkbox" name="rxEnableBigLed" value="1" %RX_BIG_EN% title="По умолчанию: Включено">
          <span>Включить светодиод</span>
        </div>
        <label title="По умолчанию: 35">Яркость светодиода (0-255):</label>
        <input type="number" name="rxPwmledBrightness" value="%RX_BIG_LED%" min="0" max="255" required>
        <div class="checkbox-container">
          <input type="checkbox" name="rxEnableBuzzer" value="1" %RX_BUZ_EN% title="По умолчанию: Выключено">
          <span>Включить вибратор</span>
        </div>
        <label title="По умолчанию: 255">Громкость вибратора (0-255):</label>
        <input type="number" name="rxBuzzerVolume" value="%RX_BUZ_VOL%" min="0" max="255" required>
        <label title="По умолчанию: 2000 мс">Отсечка постоянного нажатия (мс):</label>
        <input type="number" name="rxCutoffTime" value="%RX_CUTOFF%" min="100" step="100" required>
        <label title="По умолчанию: 9000 мс (9 сек)">Таймаут потери Пинга RX (мс):</label>
        <input type="number" name="pingTimeoutRX" value="%RX_PING%" min="1000" step="100" required>
      </fieldset>
 
      <div class="buttons-container">
        <input type="submit" value="💾 Сохранить и Перезагрузить">
        <a href="/cancel" class="cancel-btn">❌ Отмена</a>
      </div>
    </form>
 </body>
 </html>
 )rawliteral";
 
 #endif
```

---

## File: .\src\WebPortal.cpp
```cpp
/**
 * @file WebPortal.cpp
 * @version 1.71 (Добавлен справочный вывод рабочей частоты %FREQ_MHZ%)
 * @brief Реализация Captive Portal (TX) с поддержкой настроек мощности
 */
 #include "WebPortal.h"
 #include "Config.h"
 #include "webpage.h" 
 
 // Локальные макросы отладки
 #define DEBUG_ENABLE // Логирование ВКЛЮЧЕНО
 #ifdef DEBUG_ENABLE
 #define DEBUG(x) Serial.print(x)
 #define DEBUGln(x) Serial.println(x)
 #else
 #define DEBUG(x)
 #define DEBUGln(x)
 #endif
 
 // Константа протокола, нужная для перезагрузки
 #define CMD_REBOOT 222
 
 const byte DNS_PORT = 53;
 WebServer server(80);
 DNSServer dnsServer;
 
 bool isWifiActive = false;
 unsigned long wifiStartTime = 0;
 bool exitConfigRequested = false;
 // Внешние переменные из main.cpp
 extern unsigned long workFrequency;

 // Шаблон для вывода крупных сообщений на мобильном экране
 const String MSG_HEADER = F("<html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><style>body{font-family:sans-serif;text-align:center;margin-top:30%;font-size:1.5rem;background-color:#f4f4f9;color:#333;}</style></head><body>");
 const String MSG_FOOTER = F("</body></html>");
 
 // Отдача главной страницы настроек
 void handleRoot() {
    DEBUGln(F("[WIFI] Client requested root page"));
    String html = String(index_html);
    
    unsigned long elapsed = millis() - wifiStartTime;
    long remainingSec = (configTimeout > elapsed) ? (configTimeout - elapsed) / 1000 : 0;
    html.replace("%TIME_LEFT%", String(remainingSec));
 
   // Вычисление и вывод справочной рабочей частоты в МГц (1 знак после запятой)
   float freqMHz = workFrequency / 1000000.0;
   // Подстановка текущих значений в HTML-шаблон
   html.replace("%FREQ_MHZ%", String(freqMHz, 1));    
    html.replace("%ADDR%", String(workAddress));
    html.replace("%BAT_CHK%", measurebattery ? "checked" : "");
    html.replace("%BAT_PER%", String(batteryPeriod));
    html.replace("%WK_HOLD%", String(wakeUpHoldTime));
    html.replace("%WK_REL%", String(wakeUpReleaseWindow));
    html.replace("%STUCK_SL%", String(stuckSleepTime));
    html.replace("%CONF_TO%", String(configTimeout));
    html.replace("%SLP_LED%", String(sleepLedDuration));
 
    html.replace("%TX_BIG_LED%", String(pwmledBrightness));
    html.replace("%TX_FB_LED%", String(fbledBrightness));
    html.replace("%TX_PING%", String(pingTimeout));
    html.replace("%TX_BIG_TO%", String(bigTimeout));
    html.replace("%TX_EXEC_TO%", String(execTimeout));
    
    // Новые параметры мощности
    html.replace("%DYN_PWR%", dynamicPower ? "checked" : "");
    html.replace("%MAX_PWR%", String(maxPower));
    html.replace("%MIN_PWR%", String(minPower));
    html.replace("%SRV_PWR%", String(servicePower));
 
    html.replace("%RX_BIG_EN%", rxSettings.rxEnableBigLed ? "checked" : "");
    html.replace("%RX_BIG_LED%", String(rxSettings.rxPwmledBrightness));
    html.replace("%RX_BUZ_EN%", rxSettings.rxEnableBuzzer ? "checked" : "");
    html.replace("%RX_BUZ_VOL%", String(rxSettings.rxBuzzerVolume));
    html.replace("%RX_CUTOFF%", String(rxSettings.rxCutoffTime));
    html.replace("%RX_PING%", String(pingTimeoutRX));
 
    server.send(200, "text/html", html);
 } // конец функции handleRoot
 
 // Обработка кнопки "Сохранить" на веб-странице
 void handleSave() {
    DEBUGln(F("[WIFI] === Web UI: Save Requested ==="));
    
    rxSettings.workAddress = server.arg("workAddress").toInt();
    rxSettings.measurebattery = server.hasArg("measurebattery");
    rxSettings.batteryPeriod = server.arg("batteryPeriod").toInt();
    rxSettings.wakeUpHoldTime = server.arg("wakeUpHoldTime").toInt();
    rxSettings.wakeUpReleaseWindow = server.arg("wakeUpReleaseWindow").toInt();
    rxSettings.stuckSleepTime = server.arg("stuckSleepTime").toInt();
    rxSettings.configTimeout = server.arg("configTimeout").toInt();
    rxSettings.sleepLedDuration = server.arg("sleepLedDuration").toInt();
    
    rxSettings.rxEnableBigLed = server.hasArg("rxEnableBigLed");
    rxSettings.rxPwmledBrightness = server.arg("rxPwmledBrightness").toInt();
    rxSettings.rxEnableBuzzer = server.hasArg("rxEnableBuzzer");
    rxSettings.rxBuzzerVolume = server.arg("rxBuzzerVolume").toInt();
    rxSettings.rxCutoffTime = server.arg("rxCutoffTime").toInt();
    rxSettings.pingTimeoutRX = server.arg("pingTimeoutRX").toInt();
    
    // Захватываем maxPower, чтобы передать его на приемник при синхронизации
    rxSettings.maxPower = server.arg("maxPower").toInt();
 
    // Синхронизация с RX перед сохранением
    if (syncConfigToRX()) {
      DEBUGln(F("[WIFI] === RX Confirmed. Proceeding with Reboot ==="));
      
      sendMessage(CMD_REBOOT, 1);
      delay(500); 
 
      workAddress = rxSettings.workAddress;
      measurebattery = rxSettings.measurebattery;
      batteryPeriod = rxSettings.batteryPeriod;
      wakeUpHoldTime = rxSettings.wakeUpHoldTime;
      wakeUpReleaseWindow = rxSettings.wakeUpReleaseWindow;
      stuckSleepTime = rxSettings.stuckSleepTime;
      configTimeout = rxSettings.configTimeout;
      sleepLedDuration = rxSettings.sleepLedDuration;
      maxPower = rxSettings.maxPower; // Сохраняем лимит мощности
      
      pwmledBrightness = server.arg("pwmledBrightness").toInt();
      fbledBrightness = server.arg("fbledBrightness").toInt();
      pingTimeout = server.arg("pingTimeout").toInt();
      bigTimeout = server.arg("bigTimeout").toInt();
      execTimeout = server.arg("execTimeout").toInt();
      
      // Захватываем новые параметры мощности пульта
      dynamicPower = server.hasArg("dynamicPower");
      minPower = server.arg("minPower").toInt();
      servicePower = server.arg("servicePower").toInt();
      
      pingTimeoutRX = rxSettings.pingTimeoutRX;
 
      saveConfig();
      
      server.send(200, "text/html", MSG_HEADER + "<h2>✅ SUCCESS!<br>Rebooting...</h2>" + MSG_FOOTER);
      
      DEBUGln(F("[STATE] TX Rebooting now..."));
      delay(1500);
      ESP.restart(); 
    } else {
      server.send(200, "text/html", MSG_HEADER + "<h2>❌ ERROR:<br>RX not responding!</h2>" + MSG_FOOTER);
    } // конец проверки успешной синхронизации
 } // конец функции handleSave
 
 // Обработка кнопки "Отмена" на веб-странице
 void handleCancel() {
    DEBUGln(F("[WIFI] Received Cancel Request from browser"));
    server.send(200, "text/html", MSG_HEADER + "<h2>🚪 Cancelled.<br>Returning to Normal Mode.</h2>" + MSG_FOOTER);
    exitConfigRequested = true; 
 } // конец функции handleCancel
 
 void startWiFiPortal() {
    // --- ВОЗВРАТ ЧАСТОТЫ ДЛЯ СТАБИЛЬНОЙ РАБОТЫ WI-FI И ВЕБ-СЕРВЕРА ---
    setCpuFrequencyMhz(240);
    DEBUGln(F("[STATE] CPU Frequency boosted to 240MHz for WiFi Operations"));
    
    DEBUGln(F("[WIFI] Starting WiFi AP (Captive Portal)..."));
    WiFi.mode(WIFI_AP);
    
    String ssidName = "DavayLoRa_" + String(workAddress);
    WiFi.softAP(ssidName.c_str());
    
    delay(100);
    
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/cancel", HTTP_GET, handleCancel);
    
    // ФУНКЦИЯ, КОТОРУЮ НЕЛЬЗЯ УДАЛЯТЬ (Captive Portal Redirect)
    // Перенаправляет все неизвестные запросы на главную страницу портала
    server.onNotFound([]() {
      server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
      server.send(302, "text/plain", "");
    }); // конец обработчика onNotFound
    
    server.begin();
    isWifiActive = true;
    wifiStartTime = millis();
    DEBUGln(F("[WIFI] WebServer started."));
 } // конец функции startWiFiPortal
 
 void stopWiFiPortal() {
    DEBUGln(F("[WIFI] Stopping WiFi..."));
    server.stop();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    isWifiActive = false;
    DEBUGln(F("[WIFI] WiFi Stopped."));
 
    // --- СНИЖЕНИЕ ЧАСТОТЫ ПОСЛЕ ОТКЛЮЧЕНИЯ WI-FI ДЛЯ ЭКОНОМИИ БАТАРЕИ ---
    setCpuFrequencyMhz(80);
    DEBUGln(F("[STATE] CPU Frequency reduced back to 80MHz"));
 } // конец функции stopWiFiPortal
```

---

## File: .\src\WebPortal.h
```cpp
/**
 * @file WebPortal.h
 * @version 1.53
 * @brief Веб-интерфейс (Captive Portal) для настройки параметров (TX)
 * Описание: Позволяет настраивать параметры через смартфон без установки приложений.
 */
 #ifndef WEBPORTAL_H
 #define WEBPORTAL_H
 
 #include <Arduino.h>
 #include <WiFi.h>
 #include <WebServer.h>
 #include <DNSServer.h>
 
 // Глобальные объекты и флаги WiFi, нужные в main.cpp
 extern WebServer server;
 extern DNSServer dnsServer;
 extern bool isWifiActive;
 extern unsigned long wifiStartTime;
 extern bool exitConfigRequested;
 
 // Внешние зависимости (функции радио из main.cpp)
 extern void sendMessage(byte msgCmd, byte sndData);
 extern bool syncConfigToRX();
 
 // Прототипы функций веб-портала
 void startWiFiPortal();
 void stopWiFiPortal();
 void handleRoot();
 void handleSave();
 void handleCancel();
 
 #endif
```

---

