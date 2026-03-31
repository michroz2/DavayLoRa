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
 
 // Список рабочих частот (каналов связи)
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