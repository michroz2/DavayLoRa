/**
 * @file main.cpp (RX)
 * @brief Прошивка приёмника (Receiver) для проекта DavayLoRa на базе Heltec Wireless Stick Lite V3
 * * Описание логики:
 * Приёмник находится в режиме постоянного прослушивания эфира (RX Continuous).
 * При получении команды от передатчика (TX) включает исполнительные устройства (LED, Buzzer)
 * и отправляет пакет-подтверждение (Acknowledge).
 * Реализована аппаратная проверка наличия батареи по "пилообразному" шуму контроллера заряда.
 */

 #include <Arduino.h>
 #include <esp_sleep.h>
 #include <SPI.h>
 #include <RadioLib.h>
 #include <Preferences.h>
 
 Preferences preferences;
 
 // ======================= ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ (ИЗ ПАМЯТИ NVS) =======================
 // Данные переменные загружаются из памяти при старте. Если память пуста, применяются эти дефолтные значения.
 byte workAddress = 4;                 // Уникальный ID пары (TX-RX). Должен совпадать на обоих устройствах!
 int pwmledBrightness = 35;            // Яркость главного светодиода (через ШИМ, 0-255)
 int buzzerVolume = 255;               // Громкость/мощность вибромотора (через ШИМ, 0-255)
 unsigned long cutoffTime = 2000;      // Максимальное время работы периферии при залипшей кнопке (мс)
 bool measurebattery = true;           // Флаг включения проверки батареи
 
 // Настройки надежности связи (Константы)
 #define PING_TIMEOUT 5000             // Таймаут ожидания пинга от TX (мс). Если превышен - связь потеряна
 
 // Пороги индикации заряда батареи (в Вольтах) - Аппаратные константы
 #define BATTERY_MIN_VOLTAGE 3.5       // Ниже этого порога устройство уходит в глубокий сон
 #define BATTERY_VOLTAGE_1 3.6
 #define BATTERY_VOLTAGE_2 3.7
 #define BATTERY_VOLTAGE_3 3.8
 #define BATTERY_VOLTAGE_4 3.9
 #define BATTERY_VOLTAGE_5 4.0
 
 #define BATTERY_PERIOD 300000         // Периодичность проверки батареи в рабочем цикле (мс) = 5 минут
 
 // ======================= АППАРАТНАЯ КОНФИГУРАЦИЯ (HELTEC WSL V3) =======================
 
 // --- Исполнительные пины (Управление MOSFET) ---
 #define PIN_SIGNAL_LED      41     // Пин затвора транзистора главного ЛЕДа (безопасный GPIO)
 #define PIN_SIGNAL_BUZZERS  42     // Пин затвора транзистора Баззера/Вибро (безопасный GPIO)
 
 // --- Индикация (Встроенный LED платы Heltec V3) ---
 #define PIN_STATUS_LED      35     // Встроенный белый светодиод
 #define PIN_BATTERY_LED     35     // Тот же пин используется для моргания заряда батареи
 
 // --- Пины измерения батареи ---
 #define PIN_BATTERY_INTERNAL 1     // Пин АЦП для замера напряжения (ADC1_CH0)
 #define PIN_VEXT 36                // Пин управления внешним питанием (Vext)
 #define PIN_ADC_CTRL 37            // Пин управления аппаратным делителем батареи
 #define HELTEC_BATTERY_MULTIPLIER 4.9 // Коэффициент встроенного делителя: (390k + 100k) / 100k
 
 // --- Пины аппаратной шины SPI и радиомодуля SX1262 ---
 const int sckPin = 9;
 const int misoPin = 11;
 const int mosiPin = 10;
 const int csPin = 8;          // Chip Select (NSS)
 const int resetPin = 12;      // Сброс модуля
 const int irqPin = 14;        // Прерывание DIO1
 const int busyPin = 13;       // Индикатор занятости BUSY
 
 // Инициализация объекта радиомодуля
 SX1262 radio = new Module(csPin, irqPin, resetPin, busyPin);
 
 // ======================= ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ И ПРОТОКОЛ =======================
 
 #define WORK_FREQUENCY 434E6       // Базовая частота (Гц)
 
 // Настройка отладочного вывода в Serial Monitor
 #define DEBUG_ENABLE
 #ifdef DEBUG_ENABLE
 #define DEBUG(x) Serial.print(x)
 #define DEBUGln(x) Serial.println(x)
 #else
 #define DEBUG(x)
 #define DEBUGln(x)
 #endif
 
 // Макрос для выполнения кода с заданным интервалом без блокировки (без delay)
 #define EVERY_MS(x) \
   static uint32_t tmr;\
   bool flg = millis() - tmr >= (x);\
   if (flg) tmr = millis();\
   if (flg)
 
 #define MAX_ADDRESS 20             // Максимальное количество адресов/каналов
 
 // Команды протокола обмена DavayLoRa
 #define CMD_SIGNAL         208     // TX: Включить/выключить исполнительные устройства
 #define CMD_SIGNAL_OK      209     // RX: Подтверждение сигнала
 #define CMD_PING           212     // TX: Пинг поддержания связи
 #define CMD_PING_OK        213     // RX: Подтверждение пинга
 
 // Переменные состояния связи
 byte rcvAddress = 0;
 byte rcvCmd = 0;
 byte rcvData = 0;
 byte sndCmd = CMD_PING_OK;
 byte sndData;
 bool signalStatus;
 byte workChannel;
 unsigned long workFrequency = WORK_FREQUENCY;
 
 // Метрики радиосвязи
 unsigned long lastSendTime = 0;
 int lastRSSI;
 float lastSNR;
 unsigned long lastTurnaround;
 float lastFrequencyError; // В RadioLib ошибка частоты возвращается во float
 
 // Таймеры
 unsigned long pingTimeOutLastTime;
 unsigned long cutoffTimer = 0;
 
 // Флаг аппаратного прерывания от SX1262
 volatile bool receivedFlag = false;
 
 // Таблица частот для распределения устройств
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
   
   preferences.end();
   DEBUGln(F("Config loaded."));
 }
 
 void saveConfig() {
   DEBUGln(F("Saving config to NVS..."));
   preferences.begin("davaylora", false);
   
   preferences.putUChar("workAddress", workAddress);
   preferences.putBool("measureBat", measurebattery);
   preferences.putInt("bigLedBright", pwmledBrightness);
   preferences.putInt("buzzerVol", buzzerVolume);
   preferences.putULong("cutoffTime", cutoffTime);
   
   preferences.end();
   DEBUGln(F("Config saved."));
 }
 
 // ======================= ЛОГИКА ПРЕРЫВАНИЙ И СЕТИ =======================
 
 // Обработчик прерывания. Вызывается аппаратно, когда SX1262 принимает пакет.
 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR
 #endif
 void setFlag(void) {
   receivedFlag = true;
 }
 
 // Установка радиофизических параметров модуляции LoRa
 void setLoRaParams() {
   DEBUGln("setLoRaParams()");
   radio.setOutputPower(20);                     // Максимальная мощность (22dBm - абсолютный предел, 20 - безопасно)
   radio.setBandwidth(125.0);                    // Ширина полосы
   radio.setSpreadingFactor(8);                  // Фактор расширения спектра
   radio.setCodingRate(5);                       // Помехоустойчивое кодирование (4/5)
   radio.setPreambleLength(8);                   // Стандартная преамбула для SX1262
   radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE); // Стандартное синхрослово Semtech для частных сетей (Private Network)
 }
 
 // Отправка ответа (ACK) на передатчик
 void sendMessage(byte msgAddr, byte msgCmd, byte msgData) {
   DEBUGln(F(">>>Sending Message"));
   
   byte payload[3] = {msgAddr, msgCmd, msgData}; // Формируем пакет
   
   int state = radio.transmit(payload, 3);       // Блокирующая отправка
   
   if (state == RADIOLIB_ERR_NONE) {
     DEBUGln(("\tMessage sent: ") + String(msgAddr) + " " + String(msgCmd) + " " + String(msgData));
   } else {
     DEBUGln(("\tTransmit failed, code: ") + String(state));
   }
   
   lastSendTime = millis();
   pingTimeOutLastTime = lastSendTime;
   radio.startReceive();                         // Обязательно возвращаемся в режим прослушивания
 }
 
 // Проверка наличия принятых данных (вызывается в главном цикле)
 void checkReceive() {
   if (receivedFlag) {
     receivedFlag = false;
     byte payload[256];
     int state = radio.readData(payload, sizeof(payload));
     
     if (state == RADIOLIB_ERR_NONE) {
       int packetSize = radio.getPacketLength();
       onReceive(payload, packetSize);           // Если чтение успешно - передаем на парсинг
     }
     radio.startReceive();                       // Возобновляем прослушивание
   }
 }
 
 // Парсинг принятого пакета
 void onReceive(byte* payload, int packetSize) {
   DEBUGln(F("\n<<<Package Received"));
 
   rcvAddress = payload[0];
   // Программный фильтр адресов. Если адрес чужой - игнорируем пакет
   if ((rcvAddress != workAddress) || (packetSize != 3)) {
     DEBUGln(F("Invalid package! "));
     return;
   }
 
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
 
   // Подстройка частоты (AFC) отключена, так как для SX1262 она может нарушить стейт-машину приема
   // workFrequency = workFrequency - (lastFrequencyError / 2.0);
   // radio.setFrequency(workFrequency / 1000000.0);
 
   DEBUGln(F("=== onReceive done ==="));
 }
 
 // ======================= БИЗНЕС-ЛОГИКА (УПРАВЛЕНИЕ ПЕРИФЕРИЕЙ) =======================
 
 // Обработка потери связи с TX
 void processTimeOut() {
   if ((millis() - pingTimeOutLastTime) > PING_TIMEOUT) {
     DEBUGln(F("ZZZZZZZ"));
     signalStatus = false;
     pingTimeOutLastTime = millis();
     flashStatusLed(2); // Двойное мигание означает потерю связи
   }
 }
 
 // Диспетчер команд от TX
 void processCommand() {
   switch (rcvCmd) {
     case CMD_SIGNAL:
       DEBUGln(F("=== CMD_SIGNAL ==="));
       signalStatus = rcvData;
       processSignal();
       if (signalStatus)
         sendMessage(rcvAddress, CMD_SIGNAL_OK, signalStatus); // Подтверждаем включение
       break;
     case CMD_PING:
       unsigned long flashStatus = millis();
       DEBUGln(F("=== CMD_PING ==="));
       signalStatus = rcvData;
       processSignal();
       
       // Краткая визуальная индикация успешного пинга
       updateStatusLed(true);
       unsigned long restDelay = 100 - (millis() - flashStatus);
       if (restDelay > 0) delay(restDelay);
       updateStatusLed(false);
       
       sendMessage(rcvAddress, CMD_PING_OK, signalStatus);     // Подтверждаем пинг
   }
   rcvCmd = 0; // Сбрасываем команду после обработки
 }
 
 // Включение/Выключение периферии по команде
 void processSignal() {
   cutoffTimer = millis(); // Обновляем таймер защиты от "залипания"
   
   // Умножение на signalStatus (0 или 1) элегантно включает или выключает ШИМ
   analogWrite(PIN_SIGNAL_LED, signalStatus * pwmledBrightness);
   analogWrite(PIN_SIGNAL_BUZZERS, signalStatus * buzzerVolume);
   digitalWrite(PIN_STATUS_LED, signalStatus);
 }
 
 // Аппаратная защита: если TX заклинило или сигнал длится слишком долго - отключаем периферию принудительно
 void processCutoff() {
   if (signalStatus && (millis() - cutoffTimer > cutoffTime)) {
     signalStatus = 0;
     // Обязательно используем analogWrite(0) для полного сброса аппаратного таймера ШИМ на ESP32
     analogWrite(PIN_SIGNAL_LED, 0);
     analogWrite(PIN_SIGNAL_BUZZERS, 0);
     digitalWrite(PIN_STATUS_LED, 0);
   }
 }
 
 // ======================= ИНДИКАЦИЯ =======================
 
 void updateStatusLed(bool ledStatus) {
   digitalWrite(PIN_STATUS_LED, ledStatus);
 }
 
 void flashStatusLEDOnce() {
   digitalWrite(PIN_STATUS_LED, 1);
   delay(250);
   digitalWrite(PIN_STATUS_LED, 0);
   delay(250);
 }
 
 void flashStatusLed(byte times) {
   for (int i = 0; i < times; i++) {
     flashStatusLEDOnce();
   }
   delay(200);
 }
 
 // ======================= УПРАВЛЕНИЕ БАТАРЕЕЙ =======================
 
 // Первичная проверка работоспособности измерителя
 bool testBattery()   {
   DEBUGln(F("Проверка измерителя напряжения батареи Heltec WSL V3..."));
   if (batteryVoltageOK(5)) {
     DEBUGln(F("Батарея подключена и измеритель РАБОТАЕТ!"));
     return (true);
   }
   DEBUGln(F("Батарея НЕ ПОДКЛЮЧЕНА (или сбой измерителя)!"));
   return (false);
 }
 
 // Логика определения отсутствия батареи по поведению контроллера заряда ("пила" напряжения)
 bool batteryVoltageOK(byte tries) {
   DEBUGln(F("Is battery voltage OK?"));
   float minV = 5.0;
   float maxV = 0.0;
   
   for (byte i = 0; i < tries; i++) {
     float currentVBat = batteryVoltage();
     
     if (currentVBat < minV) minV = currentVBat;
     if (currentVBat > maxV) maxV = currentVBat;
     
     // Абсолютно невозможные для LiPo значения
     if ((currentVBat > 4.5) || (currentVBat < 2.5)) {
       DEBUGln(F("Voltage out of bounds (No battery or deeply discharged)!"));
       return (false);
     }
     delay(150);
   }
   
   DEBUG(F("Min V: ")); DEBUG(minV); DEBUG(F(", Max V: ")); DEBUGln(maxV);
   
   // Если пульсации превышают 0.05 Вольта - батареи физически нет, контроллер заряжает воздух
   if ((maxV - minV) > 0.05) {
     DEBUGln(F("Voltage is unstable (Sawtooth ripple)! Батарея НЕ подключена!"));
     return (false);
   }
   
   return (true);
 }
 
 // Непосредственный замер через АЦП и управление транзистором делителя
 float batteryVoltage() {
   DEBUG(F("Battery Voltage: "));
 
   digitalWrite(PIN_ADC_CTRL, LOW); // Аппаратно подаем питание на делитель
   delay(10);                       // Ждем стабилизации напряжения на АЦП
 
   float measuredvbat = analogRead(PIN_BATTERY_INTERNAL);
 
   digitalWrite(PIN_ADC_CTRL, HIGH); // Отключаем делитель для экономии энергии
 
   measuredvbat *= 3.3;
   measuredvbat /= 4095.0;           // Разрядность АЦП ESP32 (12 бит = 4096 значений)
   measuredvbat *= HELTEC_BATTERY_MULTIPLIER; // Компенсация резистивного делителя
 
   DEBUGln(measuredvbat);
   return measuredvbat;
 }
 
 // Проверка на критический разряд
 void processBattery() {
   if (batteryVoltage() < BATTERY_MIN_VOLTAGE) {
     stopWorking();
   }
 }
 
 // Визуальный показ заряда (от 1 до 5 миганий)
 void showBatteryVoltage() {
   float voltage = batteryVoltage();
   if (voltage > BATTERY_VOLTAGE_1)   flashBatteryLEDOnce();
   if (voltage > BATTERY_VOLTAGE_2)   flashBatteryLEDOnce();
   if (voltage > BATTERY_VOLTAGE_3)   flashBatteryLEDOnce();
   if (voltage > BATTERY_VOLTAGE_4)   flashBatteryLEDOnce();
   if (voltage > BATTERY_VOLTAGE_5)   flashBatteryLEDOnce();
 }
 
 void showNoBattery() {
   digitalWrite(PIN_BATTERY_LED, 1);
   delay(2000); // Длинный гудок (свечение), если батареи нет
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(250);
 }
 
 void flashBatteryLEDOnce() {
   digitalWrite(PIN_BATTERY_LED, 1);
   delay(250);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(250);
 }
 
 void flashLedBattery(byte times) {
   for (int i = 0; i < times; i++) {
     flashBatteryLEDOnce();
   }
   delay(200);
 }
 
 // Аварийное отключение при севшем аккумуляторе
 void stopWorking() {
   flashLedBattery(7);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(3000);
   flashLedBattery(7);
   digitalWrite(PIN_BATTERY_LED, 0);
   
   // Перевод процессора ESP32 и радиомодуля в спящий режим с минимальным потреблением
   esp_deep_sleep_start();
 }
 
 // ======================= ОСНОВНЫЕ ФУНКЦИИ (SETUP & LOOP) =======================
 
 void setup() {
   delay(2000);
 
 #ifdef DEBUG_ENABLE
   Serial.begin(115200); // Скорость Serial увеличена для ESP32
   while (!Serial);
 #endif
 
   DEBUGln(F("================================"));
   DEBUGln(F("=========== START RX ==========="));
 
   // Загружаем настройки из энергонезависимой памяти
   loadConfig();
 
   // Инициализация сервисных пинов питания
   pinMode(PIN_VEXT, OUTPUT);
   digitalWrite(PIN_VEXT, HIGH);     // Vext выключен по умолчанию (Active Low)
   pinMode(PIN_ADC_CTRL, OUTPUT);
   digitalWrite(PIN_ADC_CTRL, HIGH); // Делитель выключен по умолчанию (Active Low)
 
   // Настройка периферии
   pinMode(PIN_STATUS_LED, OUTPUT);
   pinMode(PIN_SIGNAL_BUZZERS, OUTPUT);
   pinMode(PIN_SIGNAL_LED, OUTPUT);
   
   // Убеждаемся, что всё выключено (сброс ШИМ)
   analogWrite(PIN_SIGNAL_LED, 0);
   analogWrite(PIN_SIGNAL_BUZZERS, 0);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(300);
 
   // Приветственный сигнал: зажигаем всё на 1 секунду для проверки
   updateStatusLed(true);
   analogWrite(PIN_SIGNAL_LED, pwmledBrightness);
   analogWrite(PIN_SIGNAL_BUZZERS, buzzerVolume);
   delay(1000);
   updateStatusLed(false);
   analogWrite(PIN_SIGNAL_LED, 0);
   analogWrite(PIN_SIGNAL_BUZZERS, 0);  
   delay(1000);
 
   // Процедура замера и индикации батареи
   DEBUGln(F("Battery Test"));
   if (measurebattery) {
     measurebattery = testBattery();
   }
        
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
   }
 
   // Запуск шины SPI для радиомодуля
   SPI.begin(sckPin, misoPin, mosiPin, csPin);
   
   workFrequency = workingFrequency[workAddress % MAX_ADDRESS];
   DEBUG("LoRa begin on ");
   DEBUGln(workFrequency);
   
   // Инициализация SX1262
   int state = radio.begin(workFrequency / 1000000.0);
   if (state != RADIOLIB_ERR_NONE) {
     DEBUGln("LoRa init failed. Code: " + String(state));
     while (true) {
       flashStatusLed(6);    // Блокирующая ошибка (6 миганий)
       delay(4000);
     }
   }
 
   setLoRaParams();
 
   // Привязка прерывания к функции setFlag
   radio.setDio1Action(setFlag);
   radio.startReceive();
 
   pingTimeOutLastTime = millis();
 
   DEBUGln(F("DavayLoRa RX setup complete"));
 }
 
 void loop() {
   checkReceive(); // Проверяем флаг прерывания
 
   if (rcvCmd)
     processCommand(); // Если пришла команда - парсим
   else
     processTimeOut(); // Иначе проверяем, не отвалился ли передатчик
 
   processCutoff();    // Аппаратная защита от зависания сигнала
 
   // Периодическая проверка батареи без delay()
   EVERY_MS(BATTERY_PERIOD) {
     if (measurebattery) {
       processBattery();
     }
   }
 }