/**
 * @file main.cpp (TX)
 * @version 1.53
 * @brief Прошивка передатчика (Transmitter) для проекта DavayLoRa на базе Heltec Wireless Stick Lite V3
 * Описание: Ядро стейт-машины, логика переключения режимов и опроса кнопок.
 */

 #include <Arduino.h>
 #include <esp_sleep.h>
 #include <driver/rtc_io.h>
 
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
 // #define DEBUG_ENABLE // Логирование выключено
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
    
    pinMode(PIN_FB_LED, OUTPUT); digitalWrite(PIN_FB_LED, LOW);
    pinMode(PIN_BIG_LED, OUTPUT); digitalWrite(PIN_BIG_LED, LOW);
    pinMode(PIN_BATTERY_LED, OUTPUT); digitalWrite(PIN_BATTERY_LED, LOW);
    
    rtc_gpio_pullup_en((gpio_num_t)PIN_BUTTON);
    rtc_gpio_pulldown_dis((gpio_num_t)PIN_BUTTON);
    
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BUTTON, 0);
    esp_deep_sleep_start();
 } // конец функции enterDeepSleep
 
 void runWakeUpProtection(uint8_t wakeupPin) {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
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
          // Передача сигнала "ACTION!"
          if (commSession(CMD_SIGNAL, 1, CMD_SIGNAL_OK, 300, WORK_COMM_ATTEMPTS)) {
            updateStatusLed(true); updateBIGLed(true);    
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
          // Отмена сигнала "ACTION!"
          sendMessage(CMD_SIGNAL, false); updateStatusLed(false); updateBIGLed(false);
        } // конец условия для нормального режима
      } // конец условия проверки нажатия
    } // конец условия изменения состояния
 } // конец функции processButton
 
 void processUserButton() {
    static unsigned long userButtonTimer = 0;
    if (digitalRead(PIN_USER) == LOW) { 
      if (userButtonTimer == 0) userButtonTimer = millis();
      else if (millis() - userButtonTimer > 5000) { 
        sleepSystem(); userButtonTimer = 0; 
      } // конец условия зажатия 5 сек
    } else { userButtonTimer = 0; } // конец условия отпускания кнопки
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
      if (prepClickCount == 2) { 
        // 2 клика: Выключение приборов (Сон)
        sleepSystem(); 
      } else if (prepClickCount == 3) {
        // 3 клика: Смена типа сигнала актеру (свет/вибро)
        commSession(CMD_EXEC_CONFIG, 1, CMD_EXEC_CONFIG_OK, 500, WORK_COMM_ATTEMPTS);
        DEBUGln(F("[STATE] ---> STATE_EXEC_CONFIG"));
        currentState = STATE_EXEC_CONFIG; execModeTimer = millis(); 
        execBlinkActive = true; execBlinkStartTime = millis(); execClickCount = 0;
      } else if (prepClickCount == 4) {
        // 4 клика: Полная настройка через телефон (Wi-Fi Портал)
        commSession(CMD_CONFIG, 1, CMD_CONFIG_OK, 500, WORK_COMM_ATTEMPTS);
        DEBUGln(F("[STATE] ---> STATE_CONFIG_STANDBY"));
        currentState = STATE_CONFIG_STANDBY; pingTimer = millis();
        configBlinkActive = true; configBlinkStartTime = millis(); configClickCount = 0;
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
      commSession(CMD_NORMAL_MODE, 1, CMD_NORMAL_MODE_OK, 500, WORK_COMM_ATTEMPTS);
      currentState = STATE_NORMAL; updateStatusLed(false); execClickCount = 0;
    } // конец условия таймаута бездействия пользователя
 
    if (execClickCount > 0 && (millis() - lastExecClickTime > 600)) {
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
      if (commSession(CMD_PING, currButtonState, CMD_PING_OK, 500, WORK_COMM_ATTEMPTS)) {
        updateStatusLed(!currButtonState); pingFlash = true;
        pingFlashTimer = millis(); pingTimer = millis();
      } else { flashStatusLed(2); } // Ошибка связи: двойная вспышка
    } // конец проверки таймера пинга
    
    if ((millis() - lastButtonTime) > bigTimeout) {
      flashStatusLed(3); buttonPressedFirstTime = false;
    } // конец условия таймаута неактивности
 } // конец функции processPing
 
 // ======================= ОСНОВНЫЕ ФУНКЦИИ (SETUP & LOOP) =======================
 
 void setup() {
 #ifdef DEBUG_ENABLE
    Serial.begin(115200);
    while (!Serial); 
 #endif
 
    DEBUGln(F("================================"));
    DEBUGln(F("=========== START TX v1.53 ==========="));
    
    DEBUGln(F("[STATE] Initializing GPIO pins..."));
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_USER, INPUT_PULLUP); 
    pinMode(PIN_FB_LED, OUTPUT);
    updateStatusLed(false);
 
    DEBUGln(F("[STATE] Loading NVS config..."));
    loadConfig();
 
    DEBUGln(F("[STATE] Running wake-up protection..."));
    runWakeUpProtection(PIN_BUTTON);
    delay(2000);
 
    DEBUG(F("Work Channel/Address: ")); DEBUGln(workAddress);
    DEBUG(F("TX BIG Brightness: ")); DEBUGln(pwmledBrightness);
    DEBUG(F("TX FB Brightness: ")); DEBUGln(fbledBrightness);
    
    // Вывод всех параметров из NVS
    DEBUG(F("Bat. Period (ms): ")); DEBUGln(batteryPeriod);
    DEBUG(F("Wake Hold/Rel (ms): ")); DEBUG(wakeUpHoldTime); DEBUG(F("/")); DEBUGln(wakeUpReleaseWindow);
    DEBUG(F("Stuck Sleep (ms): ")); DEBUGln(stuckSleepTime);
    DEBUG(F("Config TO (ms): ")); DEBUGln(configTimeout);
    DEBUG(F("Sleep LED (ms): ")); DEBUGln(sleepLedDuration);
    DEBUG(F("TX Ping TO (ms): ")); DEBUGln(pingTimeout);
    DEBUG(F("TX Big TO (ms): ")); DEBUGln(bigTimeout);
    DEBUG(F("TX Exec TO (ms): ")); DEBUGln(execTimeout);
    DEBUG(F("RX Ping TO (ms): ")); DEBUGln(pingTimeoutRX);
    DEBUG(F("RX En. LED/Buz: ")); DEBUG(rxSettings.rxEnableBigLed); DEBUG(F("/")); DEBUGln(rxSettings.rxEnableBuzzer);
    
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
      } // конец условия обработки подключенной батареи
    } else {
      DEBUGln(F("[ACTION] Battery measurement disabled in config."));
    } // конец условия проверки включения замеров батареи
 
    DEBUGln(F("[STATE] Initializing LoRa radio..."));
    SPI.begin(sckPin, misoPin, mosiPin, csPin);
    workFrequency = workingFrequency[workAddress % MAX_ADDRESS];
    int state = radio.begin(workFrequency / 1000000.0);
    if (state != RADIOLIB_ERR_NONE) while (true) { flashStatusLed(6); delay(4000); }
    setLoRaParams();
    radio.setDio1Action(setFlag);
    radio.startReceive();
    
    DEBUGln(F("[STATE] Setup complete"));
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
 } // конец функции loop