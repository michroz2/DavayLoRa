/**
 * @file main.cpp (RX)
 * @version 1.55 (RX: Оптимизация Active Mode - отключение Wi-Fi на старте и FreeRTOS Yield)
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
    if (digitalRead(PIN_USER) == LOW) { 
      if (userButtonTimer == 0) userButtonTimer = millis();
      else if (millis() - userButtonTimer > 5000) {
        DEBUGln(F("[ACTION] USER button held 5s -> Local Sleep"));
        goToSleep(); 
      } // конец проверки удержания USER
    } else userButtonTimer = 0; 
 } // конец функции processUserButton
 
 // ======================= СТЕЙТ-МАШИНА И БИЗНЕС-ЛОГИКА =======================
 
 // Исполнение команд, принятых из эфира
 void processCommand() {
    DEBUG(F("[ACTION] Processing Command: ")); DEBUGln(rcvCmd);
    switch (rcvCmd) {
      case CMD_SIGNAL:
        // Главный рабочий сигнал (Вкл/Выкл свет или вибро у актера)
        signalStatus = rcvData; processSignal();
        if (signalStatus) sendMessage(rcvAddress, CMD_SIGNAL_OK, signalStatus); 
        break;
      case CMD_PING: {
        // Синхронное моргание для подтверждения качества связи
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
    DEBUGln(F("=========== START RX v1.55 ==========="));
    
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