/**
 * @file main.cpp (TX)
 * @version 1.35 (TX: Надежный выход из Exec-Config)
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
 
 // ======================= ПЕРЕМЕННЫЕ WIFI =======================
 WebServer server(80);
 const byte DNS_PORT = 53;
 DNSServer dnsServer;
 
 bool isWifiActive = false;
 unsigned long wifiStartTime = 0;
 bool exitConfigRequested = false;
 
 // --- ПРОТОТИПЫ ---
 // Необходимы для того, чтобы компилятор знал о функциях до их реализации
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
  * Если ключ не найден, применяется значение по умолчанию (второй аргумент).
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
    
    pingTimeoutRX = preferences.getULong("pingRx", 9000); 
    rxSettings.rxEnableBigLed = preferences.getBool("rxEnBigLed", true);
    rxSettings.rxPwmledBrightness = preferences.getInt("rxBigBright", 35);
    rxSettings.rxEnableBuzzer = preferences.getBool("rxEnBuzzer", false);
    rxSettings.rxBuzzerVolume = preferences.getInt("rxBuzVol", 255);
    rxSettings.rxCutoffTime = preferences.getULong("rxCutoff", 2000);
 
    preferences.end();
    DEBUGln(F("Config loaded."));
 }
 
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
    
    preferences.putULong("pingRx", pingTimeoutRX); 
    preferences.putBool("rxEnBigLed", rxSettings.rxEnableBigLed);
    preferences.putInt("rxBigBright", rxSettings.rxPwmledBrightness);
    preferences.putBool("rxEnBuzzer", rxSettings.rxEnableBuzzer);
    preferences.putInt("rxBuzVol", rxSettings.rxBuzzerVolume);
    preferences.putULong("rxCutoff", rxSettings.rxCutoffTime);
    
    preferences.end();
    DEBUGln(F("Config saved."));
 }
 
 // ======================= РАДИООБМЕН =======================
 
 // Обработчик прерывания для пина DIO1 (срабатывает при приеме пакета)
 #if defined(ESP8266) || defined(ESP32)
    ICACHE_RAM_ATTR
 #endif
 void setFlag(void) {
    receivedFlag = true;
 }
 
 /**
  * Физическая отправка пакета через LoRa
  */
 void transmitPacket(byte* payload, size_t size) {
    DEBUG(F("[RADIO] >>> TX Packet [Size: ")); DEBUG(size); DEBUG(F("]: "));
    for (size_t i = 0; i < size; i++) { DEBUG(payload[i]); DEBUG(F(" ")); }
    DEBUGln();
 
    int state = radio.transmit(payload, size);
    if (state != RADIOLIB_ERR_NONE) {
      DEBUG(F("[RADIO] >>> Transmit failed, code: ")); DEBUGln(state);
    }
    
    lastSendTime = millis();
    receivedFlag = false; 
    radio.startReceive(); 
 }
 
 /**
  * Обертка для создания и отправки стандартного 3-байтового пакета
  */
 void sendMessage(byte msgCmd, byte sndData) {
    byte payload[3] = {workAddress, msgCmd, sndData};
    transmitPacket(payload, 3);
 }
 
 /**
  * Проверка флага прерывания и чтение данных из буфера радиомодуля
  */
 void checkReceive() {
    if (receivedFlag) {
      receivedFlag = false;
      byte payload[256];
      int state = radio.readData(payload, sizeof(payload));
      
      if (state == RADIOLIB_ERR_NONE) {
        int packetSize = radio.getPacketLength();
        onReceive(payload, packetSize);
      }
      radio.startReceive();
    }
 }
 
 /**
  * Синхронная сессия обмена с ожиданием конкретного ответа (expectedReply)
  * Повторяет отправку doTimes раз при отсутствии подтверждения.
  */
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
      }
    } while ((doTimes > 0) && (!wasReceived));
    
    pingTimer = millis();
    if (wasReceived) { DEBUGln(F("[RADIO] CommSession SUCCESS")); }
    else { DEBUGln(F("[RADIO] CommSession FAILED (Timeout/No Reply)")); }
    return wasReceived; 
 }
 
 /**
  * Сериализация и отправка структуры ConfigPacket на RX
  */
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
        }
      }
    }
    DEBUGln(F("[RADIO] --- RX Sync Timeout! ---"));
    return false;
 }
 
 // Таблица частот, привязанная к адресам (workAddress)
 unsigned long workingFrequency[MAX_ADDRESS] = {
    434000000, 434120000, 434240000, 433820000, 433700000, 433940000, 434030000,
    434150000, 434270000, 433850000, 433730000, 433970000, 434060000, 434180000,
    433880000, 433760000, 434090000, 434210000, 433910000, 433790000,
 };
 
 /**
  * Базовая конфигурация параметров LoRa для обеспечения дальнобойности
  */
 void setLoRaParams() {
    DEBUGln("[RADIO] setLoRaParams()");
    radio.setOutputPower(20);
    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(8);
    radio.setCodingRate(5);
    radio.setPreambleLength(8);     
    radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE); 
 }
 
 /**
  * Парсинг входящего пакета, проверка адреса и ожидаемой команды
  */
 void onReceive(byte* payload, int packetSize) {
    DEBUG(F("[RADIO] <<< RX Packet [Size: ")); DEBUG(packetSize); DEBUG(F("]: "));
    for (int i = 0; i < packetSize; i++) { DEBUG(payload[i]); DEBUG(F(" ")); }
    DEBUGln();
 
    if (packetSize != 3) {
      DEBUGln(F("\t[!] Invalid packet size! Expected 3 bytes."));
      delay(30);
      return;
    }
 
    rcvAddress = payload[0];
    if ((rcvAddress != workAddress) && rcvAddress != 255) { 
      DEBUGln(F("\t[!] Ignored: Wrong address"));
      delay(30);
      return;
    }
 
    rcvCmd = payload[1];
    if (rcvCmd != cmdExpected) {
      DEBUGln("\t[!] Invalid Reply: " + String(rcvCmd) + F(", Expected: ") + String(cmdExpected));
      delay(30);
      return;
    }
    rcvData = payload[2];
 
    lastTurnaround = millis() - lastSendTime;
    lastFrequencyError = radio.getFrequencyError();
    lastRSSI = radio.getRSSI();
    lastSNR = radio.getSNR();
    
    DEBUGln(("\tTurnaround: ") + String(lastTurnaround));
    wasReceived = true; 
 }
 
 // ======================= WIFI & CAPTIVE PORTAL =======================
 
 /**
  * Формирование главной HTML-страницы.
  * Плейсхолдеры заменяются на текущие значения из переменных.
  */
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
 
    html.replace("%RX_BIG_EN%", rxSettings.rxEnableBigLed ? "checked" : "");
    html.replace("%RX_BIG_LED%", String(rxSettings.rxPwmledBrightness));
    html.replace("%RX_BUZ_EN%", rxSettings.rxEnableBuzzer ? "checked" : "");
    html.replace("%RX_BUZ_VOL%", String(rxSettings.rxBuzzerVolume));
    html.replace("%RX_CUTOFF%", String(rxSettings.rxCutoffTime));
    html.replace("%RX_PING%", String(pingTimeoutRX));
 
    server.send(200, "text/html", html);
 }
 
 /**
  * Обработка нажатия кнопки "Сохранить" на Web-странице.
  * Данные собираются, упаковываются, отправляются на RX, 
  * и в случае успеха сохраняются локально.
  */
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
      pingTimeoutRX = rxSettings.pingTimeoutRX;
 
      saveConfig();
      
      String msg = "<html><body style='background:#121212;color:#4CAF50;text-align:center;padding:50px;font-family:sans-serif;'>";
      msg += "<h2>✅ СИНХРОНИЗАЦИЯ УСПЕШНА!</h2><p style='color:#fff;'>Оба устройства сохранены и сейчас перезагрузятся.</p></body></html>";
      server.send(200, "text/html", msg);
      
      DEBUGln(F("[STATE] TX Rebooting now..."));
      delay(500);
      ESP.restart(); 
    } else {
      String err = "<html><body style='background:#121212;color:#f44336;text-align:center;padding:50px;font-family:sans-serif;'>";
      err += "<h2>❌ ОШИБКА СВЯЗИ!</h2><p style='color:#fff;'>Приёмник не ответил. Настройки НЕ сохранены.<br>Проверьте, что RX включен и находится в режиме Config.</p>";
      err += "<br><a href='/' style='color:#4CAF50;text-decoration:none;border:1px solid #4CAF50;padding:10px 20px;border-radius:5px;'>Попробовать снова</a></body></html>";
      server.send(200, "text/html", err);
    }
 }
 
 /**
  * Отмена режима конфигурации (возврат в нормальный режим)
  */
 void handleCancel() {
    DEBUGln(F("[WIFI] Received Cancel Request from browser"));
    server.send(200, "text/html", "<html><body style='background:#121212;color:#fff;text-align:center;padding:50px;'><h2>🚪 Отмена...</h2><p>Интерфейс закрыт. Пульт возвращается в рабочий режим.</p></body></html>");
    exitConfigRequested = true; 
 }
 
 /**
  * Поднятие точки доступа WiFi и запуск DNS/Web серверов
  */
 void startWiFiPortal() {
    DEBUGln(F("[WIFI] Starting WiFi AP (Captive Portal)..."));
    WiFi.mode(WIFI_AP);
    
    // Динамическое имя сети на основе адреса устройства (изменения версии 1.31)
    String ssidName = "DavayLoRa_" + String(workAddress);
    WiFi.softAP(ssidName.c_str());
    
    delay(100);
    
    IPAddress apIP = WiFi.softAPIP();
    DEBUG(F("[WIFI] AP IP address: "));
    DEBUGln(apIP);
    DEBUG(F("[WIFI] Network SSID: "));
    DEBUGln(ssidName);
    
    dnsServer.start(DNS_PORT, "*", apIP);
    
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/cancel", HTTP_GET, handleCancel);
    
    server.onNotFound([]() {
      server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
      server.send(302, "text/plain", "");
    });
    
    server.begin();
    isWifiActive = true;
    wifiStartTime = millis();
    DEBUGln(F("[WIFI] WebServer started successfully."));
 }
 
 void stopWiFiPortal() {
    DEBUGln(F("[WIFI] Stopping WiFi and WebServer..."));
    server.stop();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    isWifiActive = false;
    DEBUGln(F("[WIFI] Stopped."));
 }
 
 // ======================= БИЗНЕС-ЛОГИКА (СОН, КНОПКА И ПИНГ) =======================
 
 /**
  * Перевод микроконтроллера в режим максимального энергосбережения
  */
 void enterDeepSleep() {
    DEBUGln(F("[STATE] ---> ENTERING DEEP SLEEP"));
    if (isWifiActive) stopWiFiPortal(); 
    
    radio.sleep();
    SPI.end();
    // Перевод всех SPI пинов в INPUT для снижения токов утечки
    pinMode(csPin, INPUT); pinMode(mosiPin, INPUT); pinMode(misoPin, INPUT);
    pinMode(sckPin, INPUT); pinMode(resetPin, INPUT); pinMode(busyPin, INPUT);
    pinMode(irqPin, INPUT);
    
    // Отключение питания периферии (VEXT)
    pinMode(PIN_VEXT, OUTPUT); digitalWrite(PIN_VEXT, HIGH); 
    pinMode(PIN_ADC_CTRL, OUTPUT); digitalWrite(PIN_ADC_CTRL, HIGH);
    
    pinMode(PIN_FB_LED, OUTPUT); digitalWrite(PIN_FB_LED, LOW);
    pinMode(PIN_BIG_LED, OUTPUT); digitalWrite(PIN_BIG_LED, LOW);
    pinMode(PIN_BATTERY_LED, OUTPUT); digitalWrite(PIN_BATTERY_LED, LOW);
    
    rtc_gpio_pullup_en((gpio_num_t)PIN_BUTTON);
    rtc_gpio_pulldown_dis((gpio_num_t)PIN_BUTTON);
    
    // Защита от залипания кнопки перед сном
    if (digitalRead(PIN_BUTTON) == LOW) {
      DEBUGln(F("[STATE] Button is STUCK. Sleeping with timer..."));
      esp_sleep_enable_timer_wakeup(stuckSleepTime * 1000ULL);
    } else {
      DEBUGln(F("[STATE] Normal sleep. Wakeup on EXT0 (Button). Good night!"));
      esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BUTTON, 0);
    }
    
    esp_deep_sleep_start();
 }
 
 /**
  * Защита от случайных нажатий (в кармане). Требует удержания кнопки 
  * заданное время, а затем отпускания в строго определенном окне.
  */
 void runWakeUpProtection(uint8_t wakeupPin) {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) { enterDeepSleep(); }
    
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
      DEBUGln(F("[STATE] Woke up from Deep Sleep. Checking protection..."));
      updateStatusLed(true);
      unsigned long startHold = millis();
      // Стадия 1: Ожидание удержания
      while (millis() - startHold < wakeUpHoldTime) {
        if (digitalRead(wakeupPin) == HIGH) { 
          DEBUGln(F("[ACTION] Button released too early. Going back to sleep."));
          updateStatusLed(false); enterDeepSleep(); 
        }
        delay(10);
      }
      
      DEBUGln(F("[STATE] Waiting for button release in window..."));
      unsigned long startReleaseWindow = millis();
      bool releasedInWindow = false;
      // Стадия 2: Окно отпускания (сопровождается миганием)
      while (millis() - startReleaseWindow < wakeUpReleaseWindow) {
        updateStatusLed((millis() % 200) < 100); 
        if (digitalRead(wakeupPin) == HIGH) { 
          DEBUGln(F("[ACTION] Button released! WAKE UP SUCCESS."));
          releasedInWindow = true; break; 
        }
        delay(10);
      }
      
      // Если не отпустили вовремя - снова спать
      if (!releasedInWindow) { 
        DEBUGln(F("[ACTION] Button held too long. Going back to sleep."));
        updateStatusLed(false); enterDeepSleep(); 
      }
      updateStatusLed(false); 
    }
 }
 
 /**
  * Централизованный уход в сон (выключение и TX, и RX)
  */
 void sleepSystem() {
    DEBUGln(F("[ACTION] Sending Centralized SLEEP Command"));
    if (commSession(CMD_SLEEP, 1, CMD_SLEEP_OK, 5 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
       DEBUGln(F("[RADIO] RX is sleeping"));
    } else { 
       DEBUGln(F("[RADIO] Failed to send SLEEP to RX"));
       flashStatusLed(2); 
    }
    
    // Прощальная вспышка светодиода
    updateStatusLed(true); delay(sleepLedDuration); updateStatusLed(false);
    enterDeepSleep();
 }
 
 /**
  * Обработка изменения физического состояния главной кнопки
  */
 void processButton() {
    prevButtonState = currButtonState;
    currButtonState = !digitalRead(PIN_BUTTON); 
    
    if (prevButtonState != currButtonState) {
      lastButtonTime = millis();
      prevButtonState = currButtonState;
      
      if (currButtonState) { // Кнопка нажата
        DEBUGln(F("\n[ACTION] Main Button PRESSED"));
        if (currentState == STATE_NORMAL) {
          buttonPressStartTime = millis(); pingTimer = millis(); buttonPressedFirstTime = true;
          if (commSession(CMD_SIGNAL, 1, CMD_SIGNAL_OK, 2 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
            updateStatusLed(true); updateBIGLed(true);    
          } else { updateBIGLed(false); flashStatusLed(2); }
        } else if (currentState == STATE_PREPARATION) {
          // Подсчет кликов в режиме подготовки
          prepClickCount++; lastPrepClickTime = millis(); prepModeTimer = millis(); 
          DEBUG(F("[ACTION] Prep Click Count: ")); DEBUGln(prepClickCount);
        } else if (currentState == STATE_CONFIG_STANDBY) {
          // Подсчет кликов в режиме конфигурации
          configClickCount++; lastConfigClickTime = millis();
          DEBUG(F("[ACTION] Config Click Count: ")); DEBUGln(configClickCount);
        } else if (currentState == STATE_EXEC_CONFIG) {
          // Подсчет кликов в режиме управления исполнительными элементами
          execClickCount++; lastExecClickTime = millis();
          DEBUG(F("[ACTION] Exec Config Click Count: ")); DEBUGln(execClickCount);
        }
      } else { // Кнопка отпущена
        DEBUGln(F("\n[ACTION] Main Button RELEASED"));
        if (currentState == STATE_NORMAL) {
          sendMessage(CMD_SIGNAL, false); updateStatusLed(false); updateBIGLed(false);
        }
      }
    }
 }
 
 /**
  * Визуальный паттерн для режима конфигурации (тройная вспышка)
  */
 void processConfigLed() {
    if (!configBlinkActive) return;
    unsigned long elapsed = millis() - configBlinkStartTime;
    if (elapsed < 100) updateStatusLed(true); 
    else if (elapsed < 200) updateStatusLed(false); 
    else if (elapsed < 300) updateStatusLed(true); 
    else if (elapsed < 400) updateStatusLed(false); 
    else if (elapsed < 500) updateStatusLed(true); 
    else { updateStatusLed(false); configBlinkActive = false; }
 }
 
 /**
  * Визуальный паттерн для режима исполнительных элементов (двойная вспышка)
  */
 void processExecConfigLed() {
    if (!execBlinkActive) return;
    unsigned long elapsed = millis() - execBlinkStartTime;
    if (elapsed < 100) updateStatusLed(true); 
    else if (elapsed < 200) updateStatusLed(false); 
    else if (elapsed < 300) updateStatusLed(true); 
    else { updateStatusLed(false); execBlinkActive = false; }
 }
 
 /**
  * Обработка логики в режиме подготовки (после 10 секундного удержания)
  */
 void processPreparationMode() {
    // Быстрое мигание
    EVERY_MS(166) {
      static bool prepLedState = false;
      prepLedState = !prepLedState;
      updateStatusLed(prepLedState);
    }
 
    // Если были клики и прошло время паузы - принимаем решение
    if (prepClickCount > 0 && (millis() - lastPrepClickTime > 600)) {
      if (prepClickCount == 2) { 
        DEBUGln(F("[ACTION] 2 clicks -> Centralized Sleep"));
        sleepSystem(); 
      } 
      else if (prepClickCount == 3) {
        DEBUGln(F("\n[STATE] ---> STATE_EXEC_CONFIG"));
        if (commSession(CMD_EXEC_CONFIG, 1, CMD_EXEC_CONFIG_OK, 2 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
           DEBUGln(F("[RADIO] RX confirmed EXEC_CONFIG mode"));
        }
        currentState = STATE_EXEC_CONFIG;
        pingTimer = millis();
        execBlinkActive = true; execBlinkStartTime = millis(); 
        execClickCount = 0;
      }
      else if (prepClickCount == 4) {
        DEBUGln(F("\n[STATE] ---> STATE_CONFIG_STANDBY"));
        if (commSession(CMD_CONFIG, 1, CMD_CONFIG_OK, 2 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
           DEBUGln(F("[RADIO] RX confirmed CONFIG mode"));
        }
        currentState = STATE_CONFIG_STANDBY;
        pingTimer = millis();
        configBlinkActive = true; configBlinkStartTime = millis(); 
        configClickCount = 0;
      }
      else {
        DEBUGln(F("[ACTION] Other click count -> Exit to STATE_NORMAL"));
        currentState = STATE_NORMAL; updateStatusLed(false);
      }
      prepClickCount = 0; 
    }
 
    // Таймаут режима подготовки (возврат к норме)
    if (millis() - prepModeTimer > 10000 && currentState == STATE_PREPARATION) {
      DEBUGln(F("[STATE] Prep mode timeout ---> STATE_NORMAL"));
      currentState = STATE_NORMAL; prepClickCount = 0; updateStatusLed(false);
    }
 }
 
 /**
  * Обработка логики в режиме ожидания конфигурации (развертывание WiFi)
  */
 void processConfigStandby() {
    processConfigLed();
 
    if (configClickCount > 0 && (millis() - lastConfigClickTime > 600)) {
      if (configClickCount == 2) {
        DEBUGln(F("[ACTION] 2 clicks -> Exit Config ---> STATE_NORMAL"));
        if (isWifiActive) stopWiFiPortal();
        commSession(CMD_NORMAL_MODE, 1, CMD_NORMAL_MODE_OK, 2 * lastTurnaround, WORK_COMM_ATTEMPTS);
        currentState = STATE_NORMAL; updateStatusLed(false);
      } else if (configClickCount == 1) {
        DEBUGln(F("[ACTION] 1 click -> Enable WiFi"));
        if (!isWifiActive) startWiFiPortal();
      }
      configClickCount = 0;
    }
 
    // Поддержание режима CONFIG на стороне RX
    if ((millis() - pingTimer) > pingTimeout) {
      if (commSession(CMD_CONFIG, 1, CMD_CONFIG_OK, 5 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
        configBlinkActive = true; configBlinkStartTime = millis(); 
      } else {
        DEBUGln(F("[RADIO] Config Keepalive Failed! ---> STATE_NORMAL"));
        if (isWifiActive) stopWiFiPortal(); 
        currentState = STATE_NORMAL; updateStatusLed(false); flashStatusLed(2); 
      }
      pingTimer = millis(); 
    }
 }
 
 /**
  * Обработка логики в режиме настройки исполнительных элементов (управление LED/Buzzer)
  */
 void processExecConfigStandby() {
    processExecConfigLed();
 
    if (execClickCount > 0 && (millis() - lastExecClickTime > 600)) {
      if (execClickCount == 2) {
        DEBUGln(F("[ACTION] 2 clicks -> Exit Exec Config ---> STATE_NORMAL"));
        commSession(CMD_NORMAL_MODE, 1, CMD_NORMAL_MODE_OK, 2 * lastTurnaround, WORK_COMM_ATTEMPTS);
        currentState = STATE_NORMAL; updateStatusLed(false);
      } else if (execClickCount == 1) {
        DEBUGln(F("[ACTION] 1 click -> Cycle Actuators"));
        if (commSession(CMD_CYCLE_EXEC, 1, CMD_CYCLE_EXEC_OK, 2 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
           // rcvData содержит новое состояние с приемника (бит 0 - LED, бит 1 - Buzzer)
           rxSettings.rxEnableBigLed = (rcvData & 0x01);
           rxSettings.rxEnableBuzzer = (rcvData & 0x02) >> 1;
           saveConfig();
           DEBUG(F("[ACTION] RX Actuators cycled successfully. LED: ")); DEBUG(rxSettings.rxEnableBigLed);
           DEBUG(F(", BUZZER: ")); DEBUGln(rxSettings.rxEnableBuzzer);
        } else {
           DEBUGln(F("[RADIO] Cycle command failed!"));
           flashStatusLed(2);
        }
      } else {
        // НОВОЕ: Обработка любых других кликов как отмена
        DEBUGln(F("[ACTION] Other click count -> Exit Exec Config ---> STATE_NORMAL"));
        commSession(CMD_NORMAL_MODE, 1, CMD_NORMAL_MODE_OK, 2 * lastTurnaround, WORK_COMM_ATTEMPTS);
        currentState = STATE_NORMAL; updateStatusLed(false);
      }
      execClickCount = 0;
    }
 
    // Поддержание режима EXEC_CONFIG на стороне RX
    if ((millis() - pingTimer) > pingTimeout) {
      if (commSession(CMD_EXEC_CONFIG, 1, CMD_EXEC_CONFIG_OK, 5 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
        execBlinkActive = true; execBlinkStartTime = millis(); 
      } else {
        DEBUGln(F("[RADIO] Exec Config Keepalive Failed! ---> STATE_NORMAL"));
        currentState = STATE_NORMAL; updateStatusLed(false); flashStatusLed(2); 
      }
      pingTimer = millis(); 
    }
 }
 
 /**
  * Периодическая проверка связи (Heartbeat) в нормальном режиме
  */
 void processPing() {
    if (!buttonPressedFirstTime) return; 
    
    if (pingFlash) {
      if ((millis() - pingFlashTimer) > PING_FLASH) {
        pingFlash = false; updateStatusLed(currButtonState);
      }
    } else if ((millis() - pingTimer) > pingTimeout) {
      DEBUGln(F("[RADIO] Ping TX -> RX"));
      if (commSession(CMD_PING, currButtonState, CMD_PING_OK, 5 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
        updateStatusLed(!currButtonState); pingFlash = true;
        pingFlashTimer = millis(); pingTimer = millis();
      } else { flashStatusLed(2); }
    }
    
    if ((millis() - lastButtonTime) > bigTimeout) {
      DEBUGln(F("[STATE] Inactivity Timeout -> Stopping Ping"));
      flashStatusLed(3); buttonPressedFirstTime = false;
    }
 }
 
 /**
  * Обработка дополнительной пользовательской кнопки (PRG/USER пин 0)
  */
 void processUserButton() {
    static unsigned long userButtonTimer = 0;
    if (digitalRead(PIN_USER) == LOW) { 
      if (userButtonTimer == 0) userButtonTimer = millis();
      else if (millis() - userButtonTimer > 5000) { 
        DEBUGln(F("[ACTION] USER button held 5s -> Centralized Sleep"));
        sleepSystem(); 
        userButtonTimer = 0; 
      }
    } else { userButtonTimer = 0; }
 }
 
 // ======================= ИНДИКАЦИЯ =======================
 
 void updateStatusLed(bool ledStatus) { digitalWrite(PIN_FB_LED, ledStatus); }
 void updateBIGLed(bool ledStatus) { analogWrite(PIN_BIG_LED, ledStatus * pwmledBrightness); }
 void flashStatusLed(byte times) {
    for (int i = 0; i < times; i++) {
      updateStatusLed(true); delay(100); updateStatusLed(false); delay(200);
    }
 }
 
 // ======================= УПРАВЛЕНИЕ БАТАРЕЕЙ =======================
 
 bool testBattery() {
    if (batteryVoltageOK(5)) return true;
    return false;
 }
 
 bool batteryVoltageOK(byte tries) {
    float minV = 5.0, maxV = 0.0;
    for (byte i = 0; i < tries; i++) {
      float currentVBat = batteryVoltage();
      if (currentVBat < minV) minV = currentVBat; 
      if (currentVBat > maxV) maxV = currentVBat; 
      if ((currentVBat > 4.3) || (currentVBat < 2.5)) return false;
      delay(150);
    }
    // Проверка на нестабильность (шум)
    if ((maxV - minV) > 0.05) return false;
    return true;
 }
 
 float batteryVoltage() {
    digitalWrite(PIN_ADC_CTRL, LOW); delay(10);
    float measuredvbat = analogRead(PIN_BATTERY_INTERNAL);
    digitalWrite(PIN_ADC_CTRL, HIGH);
    measuredvbat *= 3.3; measuredvbat *= HELTEC_BATTERY_MULTIPLIER; measuredvbat /= 4095.0;
    return measuredvbat;
 }
 
 void showBatteryVoltage() {
    float voltage = batteryVoltage();
    if (voltage > BATTERY_VOLTAGE_1) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_2) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_3) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_4) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_5) flashBatteryLEDOnce(); 
 }
 
 void showNoBattery() { digitalWrite(PIN_BATTERY_LED, 1); delay(2000); digitalWrite(PIN_BATTERY_LED, 0); delay(250); }
 void flashBatteryLEDOnce() { digitalWrite(PIN_BATTERY_LED, 1); delay(250); digitalWrite(PIN_BATTERY_LED, 0); delay(250); }
 void flashLedBattery(byte times) { for (int i = 0; i < times; i++) flashBatteryLEDOnce(); delay(200); }
 void processBattery() { if (batteryVoltage() < BATTERY_MIN_VOLTAGE) stopWorking(); }
 void stopWorking() { 
    DEBUGln(F("[STATE] BATTERY DEAD! Stopping..."));
    flashLedBattery(7); digitalWrite(PIN_BATTERY_LED, 0); delay(5000); 
    flashLedBattery(7); digitalWrite(PIN_BATTERY_LED, 0); enterDeepSleep(); 
 }
 
 // ======================= ОСНОВНЫЕ ФУНКЦИИ (SETUP & LOOP) =======================
 
 void setup() {
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_USER, INPUT_PULLUP); 
    pinMode(PIN_FB_LED, OUTPUT);
    digitalWrite(PIN_FB_LED, LOW);
 
    loadConfig();
    runWakeUpProtection(PIN_BUTTON);
    delay(2000);
 
 #ifdef DEBUG_ENABLE
    Serial.begin(115200);
    while (!Serial); 
 #endif
 
    DEBUGln(F("================================"));
    DEBUGln(F("=========== START TX v1.35 ==========="));
    DEBUG(F("Work Channel/Address: ")); DEBUGln(workAddress);
    DEBUG(F("Battery Check Enabled: ")); DEBUGln(measurebattery ? "YES" : "NO");
    DEBUG(F("TX BIG LED Brightness: ")); DEBUGln(pwmledBrightness);
    DEBUG(F("RX BIG LED Brightness: ")); DEBUGln(rxSettings.rxPwmledBrightness);
    DEBUGln(F("[STATE] ---> STATE_NORMAL (Boot)"));
 
    pinMode(PIN_VEXT, OUTPUT); digitalWrite(PIN_VEXT, HIGH);
    pinMode(PIN_ADC_CTRL, OUTPUT); digitalWrite(PIN_ADC_CTRL, HIGH);
    pinMode(PIN_BIG_LED, OUTPUT); pinMode(PIN_BATTERY_LED, OUTPUT);
    analogWrite(PIN_BIG_LED, 0); digitalWrite(PIN_BATTERY_LED, 0); delay(300);
 
    // Визуальное приветствие
    updateStatusLed(true); analogWrite(PIN_BIG_LED, pwmledBrightness); digitalWrite(PIN_FB_LED, HIGH); digitalWrite(PIN_BATTERY_LED, HIGH); delay(1000);
    updateStatusLed(false); analogWrite(PIN_BIG_LED, 0); digitalWrite(PIN_FB_LED, LOW); digitalWrite(PIN_BATTERY_LED, LOW); delay(1000);
 
    if (measurebattery) {
      isBatteryConnected = testBattery(); 
    }
    
    if (measurebattery && isBatteryConnected) { 
      processBattery(); delay(500); showBatteryVoltage(); delay(2000); showBatteryVoltage(); delay(500); 
    } else if (measurebattery && !isBatteryConnected) { 
      showNoBattery(); delay(500); 
    }
 
    // Инициализация LoRa
    SPI.begin(sckPin, misoPin, mosiPin, csPin);
    workFrequency = workingFrequency[workAddress % MAX_ADDRESS];
    DEBUG(F("[RADIO] LoRa Init on Frequency: ")); DEBUGln(workFrequency);
    
    int state = radio.begin(workFrequency / 1000000.0);
    if (state != RADIOLIB_ERR_NONE) {
      DEBUG(F("[RADIO] Init FAILED, code: ")); DEBUGln(state);
      while (true) { flashStatusLed(6); delay(4000); }
    }
 
    setLoRaParams();
    radio.setDio1Action(setFlag);
    radio.startReceive();
    delay(100);
    DEBUGln(F("[STATE] Setup complete, waiting for input..."));
 }
 
 void loop() {
    checkReceive(); 
    
    if ((millis() - lastButtonTime) > DEBOUNCE_TIME) processButton();
    processUserButton(); 
    
    // Отслеживание удержания кнопки (10 сек) для перехода в режим подготовки
    if (currentState == STATE_NORMAL && currButtonState) {
      if (millis() - buttonPressStartTime > 10000) {
        DEBUGln(F("\n[STATE] ---> STATE_PREPARATION"));
        currentState = STATE_PREPARATION; prepModeTimer = millis(); prepClickCount = 0;
        sendMessage(CMD_SIGNAL, false); updateBIGLed(false); buttonPressedFirstTime = false; 
      }
    }
 
    // Роутинг логики в зависимости от текущего состояния стейт-машины
    if (currentState == STATE_NORMAL) processPing();
    else if (currentState == STATE_PREPARATION) processPreparationMode();
    else if (currentState == STATE_CONFIG_STANDBY) processConfigStandby();
    else if (currentState == STATE_EXEC_CONFIG) processExecConfigStandby();
    
    // Обработка WiFi если он активен
    if (isWifiActive) {
      dnsServer.processNextRequest();
      server.handleClient();
      
      // Выход по отмене или таймауту
      if (exitConfigRequested) {
        delay(500); 
        exitConfigRequested = false;
        stopWiFiPortal();
        DEBUGln(F("[STATE] ---> STATE_NORMAL (User Cancel)"));
        commSession(CMD_NORMAL_MODE, 1, CMD_NORMAL_MODE_OK, 2 * lastTurnaround, WORK_COMM_ATTEMPTS);
        currentState = STATE_NORMAL; updateStatusLed(false);
      } else if (millis() - wifiStartTime > configTimeout) {
        DEBUGln(F("[STATE] ---> STATE_NORMAL (WiFi Timeout)"));
        stopWiFiPortal();
        commSession(CMD_NORMAL_MODE, 1, CMD_NORMAL_MODE_OK, 2 * lastTurnaround, WORK_COMM_ATTEMPTS);
        currentState = STATE_NORMAL; updateStatusLed(false);
      }
    }
    
    EVERY_MS(batteryPeriod) { 
      if (measurebattery && isBatteryConnected) processBattery(); 
    }
 }