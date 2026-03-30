/**
 * @file main.cpp (TX)
 * @version 1.42 (TX: Финализация execTimeout, строгий аудит)
 * @brief Прошивка передатчика (Transmitter) для проекта DavayLoRa на базе Heltec Wireless Stick Lite V3
 */

 #include <Arduino.h>
 #include <esp_sleep.h>
 #include <driver/rtc_io.h>
 #include <SPI.h>
 #include <RadioLib.h>
 #include <Preferences.h>
 #include <WiFi.h>
 #include <WebServer.h>
 #include <DNSServer.h>
 
 #include "webpage.h" 
 
 Preferences preferences;
 
 // ======================= ГЛОБАЛЬНЫЕ НАСТРОЙКИ =======================
 // Выравнивание структуры на 1 байт для корректной передачи по радиоканалу
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
 
 // Экземпляр структуры для хранения настроек приемника перед отправкой
 ConfigPacket rxSettings;
 
 // --- Системные и общие переменные ---
 byte workAddress = 4;                 
 bool measurebattery = true;           
 unsigned long batteryPeriod = 300000;    
 unsigned long wakeUpHoldTime = 2000;     
 unsigned long wakeUpReleaseWindow = 2000; 
 unsigned long stuckSleepTime = 10000; 
 unsigned long configTimeout = 600000; 
 unsigned long sleepLedDuration = 2000;   
 
 // --- Настройки пульта (TX) ---
 int pwmledBrightness = 35;            
 int fbledBrightness = 255;            
 unsigned long pingTimeout = 3000;     
 unsigned long bigTimeout = 3600000;   
 unsigned long execTimeout = 30000;    
 
 // --- Настройки приемника (для локального хранения/синхронизации) ---
 unsigned long pingTimeoutRX = 9000;   
 
 bool isBatteryConnected = false; 
 
 // --- Пороги напряжений для индикации заряда батареи ---
 #define BATTERY_MIN_VOLTAGE 3.5
 #define BATTERY_VOLTAGE_1 3.5
 #define BATTERY_VOLTAGE_2 3.6
 #define BATTERY_VOLTAGE_3 3.8
 #define BATTERY_VOLTAGE_4 3.9
 #define BATTERY_VOLTAGE_5 4.0
 
 // ======================= АППАРАТНАЯ КОНФИГУРАЦИЯ =======================
 
 // Распиновка Heltec V3
 #define PIN_BUTTON 7           
 #define PIN_FB_LED 35          
 #define PIN_BIG_LED 41         
 #define PIN_BATTERY_LED 35     
 #define PIN_USER 0             
 
 #define PIN_BATTERY_INTERNAL 1 
 #define PIN_VEXT 36            
 #define PIN_ADC_CTRL 37        
 #define HELTEC_BATTERY_MULTIPLIER 4.9
 
 // Распиновка встроенного модуля LoRa (SX1262)
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
 
 // Макросы для вывода отладочной информации в Serial
 #define DEBUG_ENABLE
 #ifdef DEBUG_ENABLE
 #define DEBUG(x) Serial.print(x)
 #define DEBUGln(x) Serial.println(x)
 #else
 #define DEBUG(x)
 #define DEBUGln(x)
 #endif
 
 // Макрос для выполнения кода с заданным интервалом без блокировки (non-blocking delay)
 #define EVERY_MS(x) \
    static uint32_t tmr;\
    bool flg = millis() - tmr >= (x);\
    if (flg) { tmr = millis(); }\
    if (flg)
 
 #define MAX_ADDRESS 20
 
 #define DEFAULT_TURNAROUND 300     
 #define WORK_COMM_ATTEMPTS 3       
 #define PING_FLASH 100             
 #define DEBOUNCE_TIME 100          
 
 // --- Кодовая таблица команд ---
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
 
 // Переменные для обработки радиопакетов
 byte sndCmd = CMD_PING;
 byte sndData;
 bool wasReceived = false;
 byte cmdExpected = CMD_PING_OK;
 byte rcvAddress = 0;
 byte rcvCmd = 0;
 byte rcvData;
 byte workChannel;
 unsigned long workFrequency = WORK_FREQUENCY;
 
 // Статистика радиосвязи
 long lastSendTime = 0;
 int lastRSSI;
 float lastSNR;
 unsigned long lastTurnaround = DEFAULT_TURNAROUND;
 long lastFrequencyError;
 unsigned long lastButtonTime;
 
 // Состояния кнопки
 bool currButtonState;
 bool prevButtonState;
 bool buttonPressedFirstTime;
 
 unsigned long pingTimer;
 unsigned long pingFlashTimer;
 bool pingFlash;
 
 volatile bool receivedFlag = false; // Флаг аппаратного прерывания от SX1262
 
 // --- Стейт-машина системы ---
 enum SystemState {
    STATE_NORMAL,          // Обычный рабочий режим
    STATE_PREPARATION,     // Режим подготовки (зажата кнопка на 10 сек)
    STATE_CONFIG_STANDBY,  // Режим ожидания/настройки (после 4 кликов)
    STATE_EXEC_CONFIG      // Режим быстрой настройки исполнительных элементов (после 3 кликов)
 };
 SystemState currentState = STATE_NORMAL;
 
 // Таймеры и счетчики для интерфейса управления (клики кнопки)
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
 
 // ======================= ПЕРЕМЕННЫЕ WIFI =======================
 WebServer server(80);
 const byte DNS_PORT = 53;
 DNSServer dnsServer;
 
 bool isWifiActive = false;
 unsigned long wifiStartTime = 0;
 bool exitConfigRequested = false;
 
 // --- ПРОТОТИПЫ ---
 void loadConfig();
 void saveConfig();
 void enterDeepSleep();
 void runWakeUpProtection(uint8_t wakeupPin);
 void processButton();
 void processPreparationMode();
 void processConfigStandby();
 void processConfigLed();
 void processExecConfigStandby();
 void processExecConfigLed();
 void processPing();
 void processUserButton();
 void sleepSystem();
 void updateStatusLed(bool ledStatus);
 void updateBIGLed(bool ledStatus);
 void flashStatusLed(byte times);
 void transmitPacket(byte* payload, size_t size);
 void sendMessage(byte msgCmd, byte sndData);
 bool commSession(byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes);
 bool syncConfigToRX();
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
 void startWiFiPortal();
 void stopWiFiPortal();
 void handleRoot();
 void handleSave();
 void handleCancel();
 
 // ======================= РАБОТА С ПАМЯТЬЮ NVS =======================
 
 /**
  * Чтение всех настроек из энергонезависимой памяти (NVS).
  */
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
 } // конец функции loadConfig
 
 /**
  * Сохранение локальных настроек TX в энергонезависимую память.
  */
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
 } // конец функции saveConfig
 
 // ======================= РАДИООБМЕН =======================
 
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
    radio.startReceive(); 
 } // конец функции transmitPacket
 
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
 
 bool commSession(byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes) {
    DEBUG(F("[RADIO] Starting CommSession for CMD: ")); DEBUGln(msgCmd);
    wasReceived = false;
    cmdExpected = expectedReply; 
    pingTimer = millis();
    
    do {
      checkReceive();
      EVERY_MS(waitMilliseconds) {
        sendMessage(msgCmd, sndData);
        doTimes--;
      } // конец интервальной отправки
    } while ((doTimes > 0) && (!wasReceived)); // конец цикла попыток
    
    pingTimer = millis();
    if (wasReceived) { DEBUGln(F("[RADIO] CommSession SUCCESS")); }
    else { DEBUGln(F("[RADIO] CommSession FAILED (Timeout/No Reply)")); }
    return wasReceived; 
 } // конец функции commSession
 
 bool syncConfigToRX() {
    DEBUGln(F("\n[RADIO] --- Syncing Config Struct to RX ---"));
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
 
 unsigned long workingFrequency[MAX_ADDRESS] = {
    434000000, 434120000, 434240000, 433820000, 433700000, 433940000, 434030000,
    434150000, 434270000, 433850000, 433730000, 433970000, 434060000, 434180000,
    433880000, 433760000, 434090000, 434210000, 433910000, 433790000,
 };
 
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
 
 // ======================= WIFI & CAPTIVE PORTAL =======================
 
 void handleRoot() {
    DEBUGln(F("[WIFI] Client requested root page"));
    String html = String(index_html);
    
    unsigned long elapsed = millis() - wifiStartTime;
    long remainingSec = (configTimeout > elapsed) ? (configTimeout - elapsed) / 1000 : 0;
    html.replace("%TIME_LEFT%", String(remainingSec));
 
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
 
    html.replace("%RX_BIG_EN%", rxSettings.rxEnableBigLed ? "checked" : "");
    html.replace("%RX_BIG_LED%", String(rxSettings.rxPwmledBrightness));
    html.replace("%RX_BUZ_EN%", rxSettings.rxEnableBuzzer ? "checked" : "");
    html.replace("%RX_BUZ_VOL%", String(rxSettings.rxBuzzerVolume));
    html.replace("%RX_CUTOFF%", String(rxSettings.rxCutoffTime));
    html.replace("%RX_PING%", String(pingTimeoutRX));
 
    server.send(200, "text/html", html);
 } // конец функции handleRoot
 
 void handleSave() {
    DEBUGln(F("\n[WIFI] === Web UI: Save Requested ==="));
    
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
      
      pwmledBrightness = server.arg("pwmledBrightness").toInt();
      fbledBrightness = server.arg("fbledBrightness").toInt();
      pingTimeout = server.arg("pingTimeout").toInt();
      bigTimeout = server.arg("bigTimeout").toInt();
      execTimeout = server.arg("execTimeout").toInt();
      pingTimeoutRX = rxSettings.pingTimeoutRX;
 
      saveConfig();
      
      server.send(200, "text/html", "<h2>✅ SUCCESS! Rebooting...</h2>");
      
      DEBUGln(F("[STATE] TX Rebooting now..."));
      delay(500);
      ESP.restart(); 
    } else {
      server.send(200, "text/html", "<h2>❌ ERROR: RX not responding!</h2>");
    } // конец проверки успешной синхронизации
 } // конец функции handleSave
 
 void handleCancel() {
    DEBUGln(F("[WIFI] Received Cancel Request from browser"));
    server.send(200, "text/html", "<h2>🚪 Cancelled. Returning to Normal Mode.</h2>");
    exitConfigRequested = true; 
 } // конец функции handleCancel
 
 void startWiFiPortal() {
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
 } // конец функции stopWiFiPortal
 
 // ======================= БИЗНЕС-ЛОГИКА (СОН, КНОПКА И ПИНГ) =======================
 
 void enterDeepSleep() {
    DEBUGln(F("[STATE] ---> ENTERING DEEP SLEEP"));
    if (isWifiActive) stopWiFiPortal(); 
    
    radio.sleep();
    SPI.end();
    
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
 
 void processButton() {
    prevButtonState = currButtonState;
    currButtonState = !digitalRead(PIN_BUTTON); 
    
    if (prevButtonState != currButtonState) {
      lastButtonTime = millis();
      
      if (currButtonState) { // Нажата
        DEBUGln(F("\n[ACTION] Main Button PRESSED"));
        if (currentState == STATE_NORMAL) {
          buttonPressStartTime = millis(); pingTimer = millis(); buttonPressedFirstTime = true;
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
        DEBUGln(F("\n[ACTION] Main Button RELEASED"));
        if (currentState == STATE_NORMAL) {
          sendMessage(CMD_SIGNAL, false); updateStatusLed(false); updateBIGLed(false);
        } // конец условия для нормального режима
      } // конец условия проверки нажатия
    } // конец условия изменения состояния
 } // конец функции processButton
 
 void updateStatusLed(bool ledStatus) { 
    analogWrite(PIN_FB_LED, ledStatus ? fbledBrightness : 0); 
 } // конец функции updateStatusLed
 
 void updateBIGLed(bool ledStatus) { 
    analogWrite(PIN_BIG_LED, ledStatus * pwmledBrightness); 
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
 
 void processPreparationMode() {
    EVERY_MS(166) {
      static bool prepLedState = false;
      prepLedState = !prepLedState;
      updateStatusLed(prepLedState);
    } // конец интервала мигания
 
    if (prepClickCount > 0 && (millis() - lastPrepClickTime > 600)) {
      if (prepClickCount == 2) { 
        sleepSystem(); 
      } else if (prepClickCount == 3) {
        commSession(CMD_EXEC_CONFIG, 1, CMD_EXEC_CONFIG_OK, 500, WORK_COMM_ATTEMPTS);
        currentState = STATE_EXEC_CONFIG; execModeTimer = millis(); 
        execBlinkActive = true; execBlinkStartTime = millis(); execClickCount = 0;
      } else if (prepClickCount == 4) {
        commSession(CMD_CONFIG, 1, CMD_CONFIG_OK, 500, WORK_COMM_ATTEMPTS);
        currentState = STATE_CONFIG_STANDBY; pingTimer = millis();
        configBlinkActive = true; configBlinkStartTime = millis(); configClickCount = 0;
      } else {
        currentState = STATE_NORMAL; updateStatusLed(false);
      } // конец разбора количества кликов
      prepClickCount = 0; 
    } // конец условия окончания кликов
 
    if (millis() - prepModeTimer > 10000 && currentState == STATE_PREPARATION) {
      DEBUGln(F("[STATE] Prep mode timeout -> Exit to STATE_NORMAL"));
      currentState = STATE_NORMAL; updateStatusLed(false);
    } // конец условия таймаута режима подготовки
 } // конец функции processPreparationMode
 
 void processConfigStandby() {
    processConfigLed();
 
    if (configClickCount > 0 && (millis() - lastConfigClickTime > 600)) {
      if (configClickCount == 2) {
        if (isWifiActive) stopWiFiPortal();
        commSession(CMD_NORMAL_MODE, 1, CMD_NORMAL_MODE_OK, 500, WORK_COMM_ATTEMPTS);
        currentState = STATE_NORMAL; updateStatusLed(false);
      } else if (configClickCount == 1) { 
        if (!isWifiActive) startWiFiPortal(); 
      } // конец разбора кликов настройки
      configClickCount = 0;
    } // конец условия обработки кликов
 
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
 
 void processExecConfigStandby() {
    processExecConfigLed();
 
    if (millis() - execModeTimer > execTimeout) {
      DEBUGln(F("[STATE] Exec Config Inactivity Timeout -> Exit to STATE_NORMAL"));
      commSession(CMD_NORMAL_MODE, 1, CMD_NORMAL_MODE_OK, 500, WORK_COMM_ATTEMPTS);
      currentState = STATE_NORMAL; updateStatusLed(false); execClickCount = 0;
    } // конец условия таймаута бездействия пользователя
 
    if (execClickCount > 0 && (millis() - lastExecClickTime > 600)) {
      if (execClickCount == 2) {
        commSession(CMD_NORMAL_MODE, 1, CMD_NORMAL_MODE_OK, 500, WORK_COMM_ATTEMPTS);
        currentState = STATE_NORMAL; updateStatusLed(false);
      } else if (execClickCount == 1) {
        if (commSession(CMD_CYCLE_EXEC, 1, CMD_CYCLE_EXEC_OK, 500, WORK_COMM_ATTEMPTS)) {
           rxSettings.rxEnableBigLed = (rcvData & 0x01);
           rxSettings.rxEnableBuzzer = (rcvData & 0x02) >> 1;
           saveConfig();
        } else { flashStatusLed(2); } // конец условия ошибки цикла
      } // конец разбора кликов режима управления
      execClickCount = 0;
    } // конец условия обработки кликов
 
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
 
 void processPing() {
    if (!buttonPressedFirstTime) return; 
    
    if (pingFlash) {
      if ((millis() - pingFlashTimer) > PING_FLASH) {
        pingFlash = false; updateStatusLed(currButtonState);
      } // конец условия гашения вспышки
    } else if ((millis() - pingTimer) > pingTimeout) {
      if (commSession(CMD_PING, currButtonState, CMD_PING_OK, 500, WORK_COMM_ATTEMPTS)) {
        updateStatusLed(!currButtonState); pingFlash = true;
        pingFlashTimer = millis(); pingTimer = millis();
      } else { flashStatusLed(2); } // конец условия ошибки пинга
    } // конец проверки таймера пинга
    
    if ((millis() - lastButtonTime) > bigTimeout) {
      flashStatusLed(3); buttonPressedFirstTime = false;
    } // конец условия таймаута неактивности
 } // конец функции processPing
 
 void processUserButton() {
    static unsigned long userButtonTimer = 0;
    if (digitalRead(PIN_USER) == LOW) { 
      if (userButtonTimer == 0) userButtonTimer = millis();
      else if (millis() - userButtonTimer > 5000) { 
        sleepSystem(); userButtonTimer = 0; 
      } // конец условия зажатия 5 сек
    } else { userButtonTimer = 0; } // конец условия отпускания кнопки
 } // конец функции processUserButton
 
 // ======================= УПРАВЛЕНИЕ БАТАРЕЕЙ =======================
 
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
 
 float batteryVoltage() {
    digitalWrite(PIN_ADC_CTRL, LOW); delay(10);
    float measuredvbat = analogRead(PIN_BATTERY_INTERNAL);
    digitalWrite(PIN_ADC_CTRL, HIGH);
    measuredvbat *= 3.3 * HELTEC_BATTERY_MULTIPLIER / 4095.0;
    return measuredvbat;
 } // конец функции batteryVoltage
 
 void showBatteryVoltage() {
    float voltage = batteryVoltage();
    if (voltage > BATTERY_VOLTAGE_1) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_2) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_3) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_4) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_5) flashBatteryLEDOnce(); 
 } // конец функции showBatteryVoltage
 
 void flashBatteryLEDOnce() { updateStatusLed(true); delay(250); updateStatusLed(false); delay(250); } // конец функции flashBatteryLEDOnce
 
 void processBattery() { if (batteryVoltage() < BATTERY_MIN_VOLTAGE) stopWorking(); } // конец функции processBattery
 
 void stopWorking() { enterDeepSleep(); } // конец функции stopWorking
 
 // ======================= ОСНОВНЫЕ ФУНКЦИИ (SETUP & LOOP) =======================
 
 void setup() {
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_USER, INPUT_PULLUP); 
    pinMode(PIN_FB_LED, OUTPUT);
    updateStatusLed(false);
 
    loadConfig();
    runWakeUpProtection(PIN_BUTTON);
    delay(2000);
 
 #ifdef DEBUG_ENABLE
    Serial.begin(115200);
    while (!Serial); 
 #endif
 
    DEBUGln(F("================================"));
    DEBUGln(F("=========== START TX v1.42 ==========="));
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
 
    pinMode(PIN_VEXT, OUTPUT); digitalWrite(PIN_VEXT, HIGH);
    pinMode(PIN_ADC_CTRL, OUTPUT); digitalWrite(PIN_ADC_CTRL, HIGH);
    pinMode(PIN_BIG_LED, OUTPUT); analogWrite(PIN_BIG_LED, 0);
 
    updateStatusLed(true); analogWrite(PIN_BIG_LED, pwmledBrightness); delay(1000);
    updateStatusLed(false); analogWrite(PIN_BIG_LED, 0); delay(1000);
 
    if (measurebattery) {
      isBatteryConnected = testBattery(); 
      if (isBatteryConnected) { processBattery(); showBatteryVoltage(); }
    } // конец условия проверки батареи
 
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
    
    if (currentState == STATE_NORMAL && currButtonState) {
      if (millis() - buttonPressStartTime > 10000) {
        currentState = STATE_PREPARATION; prepModeTimer = millis(); prepClickCount = 0;
        updateBIGLed(false); buttonPressedFirstTime = false; 
      } // конец условия перехода в подготовку
    } // конец условия зажатия кнопки
 
    if (currentState == STATE_NORMAL) processPing();
    else if (currentState == STATE_PREPARATION) processPreparationMode();
    else if (currentState == STATE_CONFIG_STANDBY) processConfigStandby();
    else if (currentState == STATE_EXEC_CONFIG) processExecConfigStandby();
    
    if (isWifiActive) {
      dnsServer.processNextRequest();
      server.handleClient();
      
      if (exitConfigRequested || (millis() - wifiStartTime > configTimeout)) {
        if (millis() - wifiStartTime > configTimeout) DEBUGln(F("[STATE] WiFi Portal Timeout -> Exit to STATE_NORMAL"));
        stopWiFiPortal();
        currentState = STATE_NORMAL; updateStatusLed(false);
      } // конец условия выхода из WiFi
    } // конец условия работы WiFi
    
    EVERY_MS(batteryPeriod) { 
      if (measurebattery && isBatteryConnected) processBattery(); 
    } // конец интервала проверки батареи
 } // конец функции loop