/**
 * @file RadioComm.cpp
 * @version 1.53
 * @brief Реализация радиосвязи SX1262 (TX)
 */
 #include "RadioComm.h"
 #include "Config.h"
 
 // Локальные макросы отладки
 // #define DEBUG_ENABLE // Логирование выключено
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
 
 // Список рабочих частот. Выбор частоты зависит от адреса устройства.
 unsigned long workingFrequency[MAX_ADDRESS] = {
    434000000, 434120000, 434240000, 433820000, 433700000, 433940000, 434030000,
    434150000, 434270000, 433850000, 433730000, 433970000, 434060000, 434180000,
    433880000, 433760000, 434090000, 434210000, 433910000, 433790000,
 };
 
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