# Project: DavayLoRa_HT_RX Snapshot V1.72

## File: .\src\Battery.cpp
```cpp
/**
 * @file Battery.cpp
 * @version 1.53
 * @brief Реализация подсистемы питания (RX)
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
 
 // Проверка наличия подключенной батареи при запуске
 bool testBattery() { 
    if (batteryVoltageOK(5)) return true;
    return false;
 } // конец функции testBattery
 
 // Серия замеров напряжения для фильтрации шумов и проверки стабильности
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
 
 // Считывание напряжения через АЦП микроконтроллера
 float batteryVoltage() {
    digitalWrite(PIN_ADC_CTRL, LOW); delay(10); // Включение цепи делителя напряжения                      
    float measuredvbat = analogRead(PIN_BATTERY_INTERNAL);
    digitalWrite(PIN_ADC_CTRL, HIGH); // Отключение делителя для экономии энергии
    measuredvbat *= 3.3; measuredvbat /= 4095.0; measuredvbat *= HELTEC_BATTERY_MULTIPLIER; 
    return measuredvbat;
 } // конец функции batteryVoltage
 
 // Периодическая проверка. Если разряд критический — выключение устройства
 void processBattery() { 
    if (batteryVoltage() < BATTERY_MIN_VOLTAGE) stopWorking(); 
 } // конец функции processBattery
 
 // Индикация заряда статусной лампой (до 5 вспышек)
 void showBatteryVoltage() {
    float voltage = batteryVoltage();
    if (voltage > BATTERY_VOLTAGE_1) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_2) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_3) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_4) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_5) flashBatteryLEDOnce(); 
 } // конец функции showBatteryVoltage
 
 // Индикация отсутствия батареи (например, питание от USB)
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
 
 // Принудительное засыпание приемника для защиты аккумулятора
 void stopWorking() { 
    DEBUGln(F("[STATE] BATTERY DEAD! Stopping..."));
    flashLedBattery(7); digitalWrite(PIN_BATTERY_LED, 0); delay(3000); 
    flashLedBattery(7); digitalWrite(PIN_BATTERY_LED, 0); enterDeepSleep(); 
 } // конец функции stopWorking
```

---

## File: .\src\Battery.h
```cpp
/**
 * @file Battery.h
 * @version 1.53
 * @brief Подсистема питания и контроля батареи (RX)
 * Описание: Отвечает за замеры напряжения аккумулятора и защиту от глубокого разряда.
 */
 #ifndef BATTERY_H
 #define BATTERY_H
 
 #include <Arduino.h>
 
 // --- Пороги батареи ---
 #define BATTERY_MIN_VOLTAGE 3.5
 #define BATTERY_VOLTAGE_1 3.6
 #define BATTERY_VOLTAGE_2 3.7
 #define BATTERY_VOLTAGE_3 3.8
 #define BATTERY_VOLTAGE_4 3.9
 #define BATTERY_VOLTAGE_5 4.0
 
 // --- Пины и константы батареи ---
 #define PIN_BATTERY_LED 35     
 #define PIN_BATTERY_INTERNAL 1     
 #define PIN_ADC_CTRL 37            
 #define HELTEC_BATTERY_MULTIPLIER 4.9 
 
 extern bool isBatteryConnected;
 
 // Внешние зависимости (из main.cpp)
 extern void enterDeepSleep();
 
 // Прототипы функций
 bool testBattery();
 bool batteryVoltageOK(byte tries);
 float batteryVoltage();
 void processBattery();
 void showBatteryVoltage();
 void showNoBattery();
 void flashBatteryLEDOnce();
 void flashLedBattery(byte times);
 void stopWorking();
 
 #endif
```

---

## File: .\src\Config.cpp
```cpp
/**
 * @file Config.cpp
 * @version 1.64 (RX: Добавлен прием ограничения максимальной мощности)
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
 
 byte workAddress = 0;                 
 int pwmledBrightness = 35;            
 int buzzerVolume = 255;               
 unsigned long cutoffTime = 2000;      
 unsigned long pingTimeout = 9000;     
 unsigned long stuckSleepTime = 10000; 
 bool measurebattery = true;           
 unsigned long batteryPeriod = 300000;    
 unsigned long sleepLedDuration = 2000;   
 unsigned long configTimeout = 600000;
 int maxPower = 20; // Инициализация лимита мощности
 
 unsigned long wakeUpHoldTime = 2000;     
 unsigned long wakeUpReleaseWindow = 2000; 
 
 // По умолчанию включен только свет
 bool enableBigLed = true;             
 bool enableBuzzer = false;            
 
 // Загрузка сохраненных настроек при включении приемника
 void loadConfig() {
    DEBUGln(F("--- Loading config from NVS ---"));
    preferences.begin("davaylora", false);
    
    workAddress = preferences.getUChar("workAddress", 0);
    measurebattery = preferences.getBool("measureBat", true);
    pwmledBrightness = preferences.getInt("bigLedBright", 35);
    buzzerVolume = preferences.getInt("buzzerVol", 255);
    cutoffTime = preferences.getULong("cutoffTime", 2000);
    pingTimeout = preferences.getULong("pingTimeout", 9000);
    stuckSleepTime = preferences.getULong("stuckSleep", 10000);
    configTimeout = preferences.getULong("confTo", 600000);
    maxPower = preferences.getInt("maxPwr", 20);
    
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
    DEBUG(F("maxPower: ")); DEBUGln(p->maxPower);
    
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
    preferences.putInt("maxPwr", p->maxPower);
    
    // Обновляем глобальную переменную в памяти (важно для работы прямо сейчас)
    maxPower = p->maxPower;
    
    preferences.end();
    DEBUGln(F("Config saved."));
 } // конец функции saveConfigFromPacket
```

---

## File: .\src\Config.h
```cpp
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
```

---

## File: .\src\main.cpp
```cpp
/** Gemini
 * @file main.cpp (RX) 
 * @version 1.72 Внедрена вычисляемая рабочая частота и число каналов 81
 * @brief ПОЛНЫЙ ИСХОДНЫЙ КОД ПРИЁМНИКА (DavayLoRa)
 * Описание: Ядро стейт-машины, логика переключения режимов и обработка геркона/кнопки.
 */

 #include <Arduino.h>
 #include <esp_sleep.h>
 #include <driver/rtc_io.h>
 #include <driver/gpio.h> // Добавлено для функций заморозки пинов (gpio_hold)
 #include <WiFi.h>        // Добавлено для принудительного отключения модема на старте
 
 #include "Battery.h" 
 #include "Config.h"   
 #include "RadioComm.h" 
 
 // ======================= АППАРАТНАЯ КОНФИГУРАЦИЯ =======================
 
 // Распиновка Heltec V3
 #define PIN_SIGNAL_LED      41     // Сигнальный свет для актера
 #define PIN_SIGNAL_BUZZERS  42     // Вибрация / Зуммер
 #define PIN_REED            7      // Магнитный переключатель (геркон) для включения
 #define PIN_USER            0      // Кнопка PRG на плате
 #define PIN_STATUS_LED      35     // Мелкий светодиод обратной связи
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
 
 bool signalStatus; // Активно ли удержание кнопки на пульте в данный момент
 
 unsigned long pingTimeOutLastTime; // Таймер Failsafe для отключения при обрыве связи
 unsigned long cutoffTimer = 0;     // Таймер защиты от слишком долгого удержания сигнала
 
 // Состояния RX: Рабочий режим, Настройка (ожидание Wi-Fi), Настройка сигнала (Свет/Вибро)
 enum SystemState { STATE_NORMAL, STATE_CONFIG, STATE_EXEC_CONFIG };
 SystemState currentState = STATE_NORMAL;
 
 bool configBlinkActive = false;
 unsigned long configBlinkStartTime = 0;
 
 bool execBlinkActive = false;
 unsigned long execBlinkStartTime = 0;
 
 // --- ПРОТОТИПЫ ---
 void updateStatusLed(bool ledStatus);
 void flashStatusLEDOnce();
 void flashStatusLed(byte times);
 void processConfigLed();
 void processExecConfigLed();
 void enterDeepSleep();
 void runWakeUpProtection(uint8_t wakeupPin);
 void goToSleep();
 void processUserButton();
 void processCommand();
 void processSignal();
 void processCutoff();
 void processTimeOut();
 
 // ======================= ИНДИКАЦИЯ (LED & BUZZER) =======================
 
 void updateStatusLed(bool ledStatus) { 
    static int lastLedState = -1;
    int newState = ledStatus ? HIGH : LOW;
    if (lastLedState != newState) {
      digitalWrite(PIN_STATUS_LED, newState);
      lastLedState = newState;
    } // конец защиты от аппаратного спама
 } // конец функции updateStatusLed
 
 void flashStatusLEDOnce() { 
    digitalWrite(PIN_STATUS_LED, 1); delay(250); digitalWrite(PIN_STATUS_LED, 0); delay(250); 
 } // конец функции flashStatusLEDOnce
 
 void flashStatusLed(byte times) { 
    for (int i = 0; i < times; i++) flashStatusLEDOnce(); 
    delay(200); 
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
 
 // Оптимизация Deep Sleep: Перевод всей периферии в режим минимального энергопотребления
 void enterDeepSleep() {
    DEBUGln(F("[STATE] ---> ENTERING DEEP SLEEP"));
    radio.sleep();
    SPI.end(); // Выключение аппаратной шины SPI
    
    // Блокировка паразитных токов на пинах LoRa путем перевода их в режим высокого сопротивления (INPUT)
    pinMode(csPin, INPUT); pinMode(mosiPin, INPUT); pinMode(misoPin, INPUT);
    pinMode(sckPin, INPUT); pinMode(resetPin, INPUT); pinMode(busyPin, INPUT);
    pinMode(irqPin, INPUT);
    
    // Обесточивание периферии и делителя батареи
    pinMode(PIN_VEXT, OUTPUT); digitalWrite(PIN_VEXT, HIGH);
    pinMode(PIN_ADC_CTRL, OUTPUT); digitalWrite(PIN_ADC_CTRL, HIGH);
    
    // --- ИДЕАЛЬНЫЙ СОН: Заморозка пинов для исключения плавающих уровней ---
    gpio_hold_en((gpio_num_t)PIN_VEXT);
    gpio_hold_en((gpio_num_t)PIN_ADC_CTRL);
    gpio_deep_sleep_hold_en();
    // -----------------------------------------------------------------------
 
    pinMode(PIN_STATUS_LED, OUTPUT); digitalWrite(PIN_STATUS_LED, LOW);
    pinMode(PIN_SIGNAL_LED, OUTPUT); digitalWrite(PIN_SIGNAL_LED, LOW);
    pinMode(PIN_SIGNAL_BUZZERS, OUTPUT); digitalWrite(PIN_SIGNAL_BUZZERS, LOW);
    
    rtc_gpio_pullup_en((gpio_num_t)PIN_REED);
    rtc_gpio_pulldown_dis((gpio_num_t)PIN_REED);
    
    // Защита от "залипания" магнита: если он не был убран, уходим в сон по таймеру, а не по прерыванию
    if (digitalRead(PIN_REED) == LOW) {
      DEBUGln(F("[STATE] Reed is STUCK. Sleeping with timer..."));
      esp_sleep_enable_timer_wakeup(stuckSleepTime * 1000ULL);
    } else {
      DEBUGln(F("[STATE] Normal sleep. Wakeup on EXT0 (Reed). Good night!"));
      esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_REED, 0);
    } // конец проверки залипания геркона
    esp_deep_sleep_start();
 } // конец функции enterDeepSleep
 
 // Защита от дребезга и случайного касания магнита при пробуждении
 void runWakeUpProtection(uint8_t wakeupPin) {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) enterDeepSleep(); 
    
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
      DEBUGln(F("[STATE] Woke up from Deep Sleep. Checking protection..."));
      updateStatusLed(true);
      unsigned long startHold = millis();
      // Требование удержания магнита не менее wakeUpHoldTime (2 сек)
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
      // Требование убрать магнит в течение окна wakeUpReleaseWindow
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
 
 // ======================= ОБРАБОТКА ВВОДА (КНОПКИ) =======================
 
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
        goToSleep(); 
        userButtonTimer = 0; // Сброс таймера для предотвращения спама
      } // конец проверки удержания USER
    } // конец проверки зажатия
 } // конец функции processUserButton
 
 // ======================= РАДИООБМЕН =======================
 
 // Исполнение команд, принятых из эфира
 void processCommand() {
    int currentRssi = (int)radio.getRSSI();
    DEBUG(F("[RADIO] RX Cmd: ")); DEBUG(rcvCmd); DEBUG(F(" | RSSI: ")); DEBUGln(currentRssi);
 
    switch (rcvCmd) {
      case CMD_SIGNAL:
        // Главный рабочий сигнал (Вкл/Выкл свет или вибро у актера)
        signalStatus = rcvData; processSignal();
        radio.setOutputPower(maxPower); // Ответы на Action всегда на максимум из конфига
        if (signalStatus) sendMessage(rcvAddress, CMD_SIGNAL_OK, signalStatus); 
        break;
      case CMD_PING: {
        // Синхронное моргание для подтверждения качества связи и АДАПТИВНАЯ МОЩНОСТЬ
        // Распаковка требуемой мощности от TX
        int reqPower = (int)(rcvData & 0x7F) - 10;
        signalStatus = (rcvData >> 7) & 0x01;
        
        processSignal();
        updateStatusLed(true);
        
        // Безопасная упаковка RSSI (защита от отрицательного переполнения)
        int safeRssi = abs(currentRssi);
        if (safeRssi < 30) safeRssi = 30; // Лимит SX1262
        byte packedRssi = (byte)(safeRssi - 30);
        byte replyByte = (signalStatus << 7) | (packedRssi & 0x7F);
        
        radio.setOutputPower(reqPower);
        DEBUG(F("[PING] Reply Power: ")); DEBUG(reqPower); DEBUGln(F(" dBm"));
        sendMessage(rcvAddress, CMD_PING_OK, replyByte);     
        
        delay(50); updateStatusLed(false);
        radio.setOutputPower(maxPower); // Возврат на maxPower для готовности к Action
        break;
      } // конец обработки CMD_PING
      case CMD_SLEEP:
        radio.setOutputPower(maxPower);
        sendMessage(rcvAddress, CMD_SLEEP_OK, 1); 
        delay(100); goToSleep();
        break;
      case CMD_CONFIG:
        DEBUGln(F("[STATE] ---> STATE_CONFIG"));
        currentState = STATE_CONFIG;
        pingTimeOutLastTime = millis();
        radio.setOutputPower(maxPower);
        sendMessage(rcvAddress, CMD_CONFIG_OK, 1);
        configBlinkActive = true; configBlinkStartTime = millis(); 
        analogWrite(PIN_SIGNAL_LED, 0); analogWrite(PIN_SIGNAL_BUZZERS, 0);
        break;
      case CMD_EXEC_CONFIG:
        DEBUGln(F("[STATE] ---> STATE_EXEC_CONFIG"));
        currentState = STATE_EXEC_CONFIG;
        pingTimeOutLastTime = millis();
        radio.setOutputPower(maxPower);
        sendMessage(rcvAddress, CMD_EXEC_CONFIG_OK, 1);
        execBlinkActive = true; execBlinkStartTime = millis(); 
        analogWrite(PIN_SIGNAL_LED, 0); analogWrite(PIN_SIGNAL_BUZZERS, 0);
        break;
      case CMD_CYCLE_EXEC: {
        // Смена исполнительных устройств (свет -> вибро -> свет+вибро -> ничего) на лету
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
        
        radio.setOutputPower(maxPower);
        sendMessage(rcvAddress, CMD_CYCLE_EXEC_OK, state);
        
        // Демонстрация актеру выбранного режима (длится 1 сек)
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
        radio.setOutputPower(maxPower);
        sendMessage(rcvAddress, CMD_NORMAL_MODE_OK, 1);
        break;
    } // конец switch
    rcvCmd = 0; 
 } // конец функции processCommand
 
 // Физическое включение света и вибрации (с учетом настроек яркости и громкости)
 void processSignal() {
    cutoffTimer = millis(); 
    if (signalStatus && enableBigLed) analogWrite(PIN_SIGNAL_LED, pwmledBrightness);
    else analogWrite(PIN_SIGNAL_LED, 0);
    
    if (signalStatus && enableBuzzer) analogWrite(PIN_SIGNAL_BUZZERS, buzzerVolume);
    else analogWrite(PIN_SIGNAL_BUZZERS, 0);
    
    digitalWrite(PIN_STATUS_LED, signalStatus);
 } // конец функции processSignal
 
 // Защитная отсечка: если сигнал подается слишком долго, он отключается для экономии батареи
 void processCutoff() {
    if (signalStatus && (millis() - cutoffTimer > cutoffTime)) {
      DEBUGln(F("[ACTION] Signal Cutoff Triggered!"));
      signalStatus = 0;
      analogWrite(PIN_SIGNAL_LED, 0); analogWrite(PIN_SIGNAL_BUZZERS, 0);
      digitalWrite(PIN_STATUS_LED, 0);
    } // конец проверки таймаута отсечки
 } // конец функции processCutoff
 
 // Проверка потери связи (если пинги от пульта перестали приходить)
 void processTimeOut() {
    if ((millis() - pingTimeOutLastTime) > pingTimeout) {
      signalStatus = false;
      pingTimeOutLastTime = millis();
      
      if (currentState == STATE_NORMAL) {
        DEBUGln(F("[RADIO] Ping Timeout! No connection."));
        flashStatusLed(2); // Двойная вспышка - индикация потери связи
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
 
    DEBUGln(F("================================"));
    DEBUGln(F("=========== START RX v1.67 ==========="));
    
    DEBUGln(F("[STATE] Initializing GPIO pins..."));
    pinMode(PIN_REED, INPUT_PULLUP);
    pinMode(PIN_USER, INPUT_PULLUP); 
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);
 
    DEBUGln(F("[STATE] Loading NVS config..."));
    loadConfig();
 
    DEBUGln(F("[STATE] Running wake-up protection..."));
    runWakeUpProtection(PIN_REED);
    delay(2000);
 
    DEBUG(F("Work Channel/Address: ")); DEBUGln(workAddress);
    DEBUG(F("Battery Check Enabled: ")); DEBUGln(measurebattery ? "YES" : "NO");
    DEBUG(F("RX BIG LED Brightness: ")); DEBUGln(pwmledBrightness);
    DEBUG(F("RX Buzzer Volume: ")); DEBUGln(buzzerVolume);
    
    // Вывод параметров NVS
    DEBUG(F("Bat. Period (ms): ")); DEBUGln(batteryPeriod);
    DEBUG(F("Wake Hold/Rel (ms): ")); DEBUG(wakeUpHoldTime); DEBUG(F("/")); DEBUGln(wakeUpReleaseWindow);
    DEBUG(F("Stuck Sleep (ms): ")); DEBUGln(stuckSleepTime);
    DEBUG(F("Config TO (ms): ")); DEBUGln(configTimeout);
    DEBUG(F("Sleep LED (ms): ")); DEBUGln(sleepLedDuration);
    DEBUG(F("RX En. Big LED: ")); DEBUGln(enableBigLed ? "YES" : "NO");
    DEBUG(F("RX En. Buzzer: ")); DEBUGln(enableBuzzer ? "YES" : "NO");
    DEBUG(F("RX Cutoff (ms): ")); DEBUGln(cutoffTime);
    DEBUG(F("RX Ping TO (ms): ")); DEBUGln(pingTimeout);
    DEBUG(F("Max Power Limit: ")); DEBUGln(maxPower);
 
    DEBUGln(F("[STATE] ---> STATE_NORMAL (Boot)"));
 
    DEBUGln(F("[STATE] Powering up peripherals (VEXT/ADC)..."));
    pinMode(PIN_VEXT, OUTPUT); digitalWrite(PIN_VEXT, HIGH);     
    pinMode(PIN_ADC_CTRL, OUTPUT); digitalWrite(PIN_ADC_CTRL, HIGH); 
    pinMode(PIN_SIGNAL_BUZZERS, OUTPUT); pinMode(PIN_SIGNAL_LED, OUTPUT);
    pinMode(PIN_BATTERY_LED, OUTPUT); digitalWrite(PIN_BATTERY_LED, 0);
    
    analogWrite(PIN_SIGNAL_LED, 0); analogWrite(PIN_SIGNAL_BUZZERS, 0);
    delay(300);
 
    DEBUGln(F("[ACTION] Testing LEDs and Buzzer..."));
    updateStatusLed(true);
    if (enableBigLed) analogWrite(PIN_SIGNAL_LED, pwmledBrightness);
    if (enableBuzzer) analogWrite(PIN_SIGNAL_BUZZERS, buzzerVolume);
    delay(1000);
    updateStatusLed(false);
    analogWrite(PIN_SIGNAL_LED, 0); analogWrite(PIN_SIGNAL_BUZZERS, 0);  
    delay(1000);
 
    DEBUGln(F("[ACTION] Checking battery status..."));
    if (measurebattery) {
      isBatteryConnected = testBattery(); 
      if (isBatteryConnected) {
        DEBUGln(F("[ACTION] Battery connected. Showing voltage (2 times)."));
        processBattery(); 
        delay(500); 
        showBatteryVoltage(); 
        delay(2000); 
        showBatteryVoltage(); 
        delay(500);
      } else { 
        DEBUGln(F("[ACTION] No battery detected."));
        showNoBattery(); 
        delay(500); 
      } // конец ветвления индикации подключенной батареи
    } else {
      DEBUGln(F("[ACTION] Battery measurement disabled in config."));
    } // конец ветвления включения замеров батареи
 
    DEBUGln(F("[STATE] Initializing LoRa radio..."));
    SPI.begin(sckPin, misoPin, mosiPin, csPin);
    workFrequency = WORK_FREQUENCY + ( (workAddress % MAX_ADDRESS) * 200000 );
    DEBUG(F("[RADIO] LoRa Init on Frequency: ")); DEBUGln(workFrequency);
    
    int state = radio.begin(workFrequency / 1000000.0);
    if (state != RADIOLIB_ERR_NONE) {
      DEBUG(F("[RADIO] Init FAILED, code: ")); DEBUGln(state);
      while (true) { flashStatusLed(6); delay(4000); }
    } // конец проверки инициализации радио
 
    setLoRaParams();
    
    // Максимальная чувствительность приемника
    radio.setRxBoostedGainMode(true);
    
    radio.setDio1Action(setFlag);
    radio.startReceive();
 
    pingTimeOutLastTime = millis();
    
    // --- СНИЖЕНИЕ ЧАСТОТЫ ПРОЦЕССОРА ТОЛЬКО ПОСЛЕ ПОЛНОЙ ИНИЦИАЛИЗАЦИИ ---
    // Спасает стек ipc1 от переполнения на старте!
    setCpuFrequencyMhz(80);
    // ---------------------------------------------------------------------
    
    DEBUGln(F("[STATE] Setup complete"));
 } // конец функции setup
 
 void loop() {
    // Проверка, пришел ли пакет по воздуху
    checkReceive(); 
 
    // Исполнение пришедшей команды либо проверка таймаута соединения
    if (rcvCmd) processCommand(); 
    else processTimeOut(); 
 
    // Маршрутизация логики в зависимости от режима
    if (currentState == STATE_NORMAL) processCutoff();
    else if (currentState == STATE_CONFIG) processConfigLed();
    else if (currentState == STATE_EXEC_CONFIG) processExecConfigLed();
    
    processUserButton(); 
 
    // Периодический мониторинг заряда батареи
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
 * @brief Реализация радиосвязи SX1262 (RX)
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
 
 byte rcvAddress = 0;
 byte rcvCmd = 0;
 byte rcvData = 0;
 unsigned long workFrequency = WORK_FREQUENCY;
 
 unsigned long lastSendTime = 0;
 volatile bool receivedFlag = false;
 
//  // Список рабочих частот (каналов связи)
//  unsigned long workingFrequency[MAX_ADDRESS] = {
//     434000000, 434120000, 434240000, 433820000, 433700000, 433940000, 434030000,
//     434150000, 434270000, 433850000, 433730000, 433970000, 434060000, 434180000,
//     433880000, 433760000, 434090000, 434210000, 433910000, 433790000,
//  };
// теперь будет вычисляться по формуле: 434000000 + (адрес * 200000)
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
 
 // Отправка пакета-ответа пульту
 void transmitPacket(byte* payload, size_t size) {
    DEBUG(F("[RADIO] >>> TX Packet [Size: ")); DEBUG(size); DEBUG(F("]: "));
    for (size_t i = 0; i < size; i++) { DEBUG(payload[i]); DEBUG(F(" ")); }
    DEBUGln();
 
    int state = radio.transmit(payload, size);
    if (state != RADIOLIB_ERR_NONE) {
      DEBUG(F("[RADIO] >>> Transmit failed, code: ")); DEBUGln(state);
    } // конец проверки ошибки передачи
    
    lastSendTime = millis();
    pingTimeOutLastTime = lastSendTime; // Сброс таймера отсутствия связи
    receivedFlag = false; 
    radio.startReceive(); // Немедленный возврат к прослушиванию эфира
 } // конец функции transmitPacket
 
 void sendMessage(byte msgAddr, byte msgCmd, byte msgData) {
    byte payload[3] = {msgAddr, msgCmd, msgData}; 
    transmitPacket(payload, 3);                         
 } // конец функции sendMessage
 
 // Проверка прерывания: пришел ли пакет из эфира
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
 
 // Обработка входящего пакета и передача команды в стейт-машину
 void onReceive(byte* payload, int packetSize) {
    DEBUG(F("[RADIO] <<< RX Packet [Size: ")); DEBUG(packetSize); DEBUG(F("]: "));
    for (int i = 0; i < packetSize; i++) { DEBUG(payload[i]); DEBUG(F(" ")); }
    DEBUGln();
 
    // Адресный фильтр: игнорируем чужие пульты
    rcvAddress = payload[0];
    if (rcvAddress != workAddress) {
      DEBUGln(F("\t[!] Ignored: Wrong address"));
      return;
    } // конец проверки адреса
 
    // Стандартный пакет управления
    if (packetSize == 3) {
      rcvCmd = payload[1];
      rcvData = payload[2];
      
      if (rcvCmd == CMD_REBOOT) {
        DEBUGln(F("[ACTION] !!! CMD_REBOOT RECEIVED. Restarting in 500ms !!!"));
        delay(500);
        ESP.restart();
      } // конец обработки команды перезагрузки
    } 
    // Длинный пакет: Синхронизация структуры настроек по воздуху
    else if (packetSize == (sizeof(ConfigPacket) + 2) && payload[1] == CMD_SYNC_CONFIG) {
      DEBUGln(F("[RADIO] <<< Received Config Struct from TX!"));
      
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
```

---

## File: .\src\RadioComm.h
```cpp
/**
 * @file RadioComm.h
 * @version 1.53
 * @brief Модуль радиосвязи SX1262 и протокол (RX)
 * Описание: Слушает эфир, фильтрует пакеты по адресу и транслирует команды в main.cpp.
 */
 #ifndef RADIOCOMM_H
 #define RADIOCOMM_H
 
 #include <Arduino.h>
 #include <RadioLib.h>
 #include <SPI.h>
 
 // --- Макросы команд (Протокол связи v1.53) ---
 #define CMD_SIGNAL         208  // Сигнал нажатия кнопки (Action!)
 #define CMD_SIGNAL_OK      209
 #define CMD_PING           212  // Пинг от пульта
 #define CMD_PING_OK        213
 #define CMD_SLEEP          214  // Приказ уснуть
 #define CMD_SLEEP_OK       215
 #define CMD_CONFIG         216  // Переход в режим настройки Wi-Fi
 #define CMD_CONFIG_OK      217
 #define CMD_NORMAL_MODE    218  // Возврат в рабочий режим
 #define CMD_NORMAL_MODE_OK 219
 #define CMD_SYNC_CONFIG    220  // Прием длинного пакета с настройками
 #define CMD_SYNC_CONFIG_OK 221
 #define CMD_REBOOT         222  // Перезагрузка
 #define CMD_EXEC_CONFIG    223  // Переход в режим выбора свето/вибро
 #define CMD_EXEC_CONFIG_OK 224
 #define CMD_CYCLE_EXEC     225  // Смена свето/вибро по кругу
 #define CMD_CYCLE_EXEC_OK  226
 
 #define WORK_FREQUENCY 434E6
 #define MAX_ADDRESS 81
 
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
 extern byte rcvAddress;
 extern byte rcvCmd;
 extern byte rcvData;
 extern unsigned long workFrequency;
 extern unsigned long lastSendTime;
 extern volatile bool receivedFlag;
 //extern unsigned long workingFrequency[MAX_ADDRESS];
 
 // Внешние переменные из main.cpp
 extern unsigned long pingTimeOutLastTime;
 
 // --- Прототипы ---
 void setFlag(void);
 void setLoRaParams();
 void transmitPacket(byte* payload, size_t size);
 void sendMessage(byte msgAddr, byte msgCmd, byte msgData);
 void checkReceive();
 void onReceive(byte* payload, int packetSize);
 
 #endif
```

---

