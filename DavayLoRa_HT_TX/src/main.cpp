/**
 * @file main.cpp (TX)
 * @brief Прошивка передатчика (Transmitter) для проекта DavayLoRa на базе Heltec Wireless Stick Lite V3
 * * Описание логики:
 * Пульт реагирует на нажатие физической кнопки. Отправляет команду включения,
 * ждет ACK от приёмника. Генерирует периодические ПИНГ-пакеты,
 * чтобы приёмник понимал, что связь не оборвалась.
 * Поддерживает режим отладки, индикацию состояния и мониторинг батареи.
 * Version 2.1.1
 * Реализация отслеживания непрерывного удержания кнопки в течение 10 секунд.
 */

 #include <Arduino.h>
 #include <esp_sleep.h>
 #include <SPI.h>
 #include <RadioLib.h>
 #include <Preferences.h>
 
 Preferences preferences;
 
 // ======================= ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ (ИЗ ПАМЯТИ NVS) =======================
 // Данные переменные загружаются из памяти при старте. Если память пуста, применяются эти дефолтные значения.
 byte workAddress = 4;                 // Уникальный ID пары (TX-RX). Должен совпадать!
 int pwmledBrightness = 35;            // Яркость главного сигнального светодиода
 int fbledBrightness = 255;            // Яркость светодиода обратной связи на кнопке (Feedback)
 unsigned long pingTimeout = 3000;     // Периодичность отправки пингов при удержании кнопки (мс)
 unsigned long bigTimeout = 3600000;   // Максимальное время работы пульта (1 час), защита от случайного нажатия
 bool measurebattery = true;           // Измерять ли батарею
 
 // Пороги индикации заряда батареи (в Вольтах) - Аппаратные константы
 #define BATTERY_MIN_VOLTAGE 3.5
 #define BATTERY_VOLTAGE_1 3.5
 #define BATTERY_VOLTAGE_2 3.6
 #define BATTERY_VOLTAGE_3 3.8
 #define BATTERY_VOLTAGE_4 3.9
 #define BATTERY_VOLTAGE_5 4.0
 
 #define BATTERY_PERIOD 300000         // 5 минут
 
 // ======================= АППАРАТНАЯ КОНФИГУРАЦИЯ (HELTEC WSL V3) =======================
 
 // --- Пины интерфейса ---
 #define PIN_BUTTON 45          // Пин подключения кнопки (срабатывает на замыкание к земле)
 #define PIN_FB_LED 35          // Пин встроенного светодиода (Feedback LED)
 #define PIN_BIG_LED 41         // Пин внешнего сигнального светодиода
 #define PIN_BATTERY_LED 35     // Пин индикации заряда
 
 // --- Пины измерения батареи ---
 #define PIN_BATTERY_INTERNAL 1 // АЦП
 #define PIN_VEXT 36            // Управление периферией платы
 #define PIN_ADC_CTRL 37        // Включение/выключение делителя
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
   if (flg) tmr = millis();\
   if (flg)
 
 #define MAX_ADDRESS 20
 
 // Настройки надежности радиообмена
 #define DEFAULT_TURNAROUND 300     // Базовое время ожидания ответа (мс)
 #define WORK_COMM_ATTEMPTS 3       // Количество попыток передачи, если нет ответа
 #define PING_FLASH 100             // Длительность мигания LED при пинге (мс)
 #define DEBOUNCE_TIME 100          // Антидребезг контактов кнопки (мс)
 
 // Команды протокола
 #define CMD_SIGNAL    208
 #define CMD_SIGNAL_OK 209
 #define CMD_PING      212
 #define CMD_PING_OK   213
 
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
 bool buttonChanged;
 bool buttonSent;
 
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

unsigned long buttonPressStartTime = 0; // Время начала удержания кнопки
unsigned long prepModeTimer = 0;        // Таймер 10 секунд для автовыхода

 // --- ПРОТОТИПЫ ФУНКЦИЙ ---
 void loadConfig();
 void saveConfig();
 void processButton();
 void processPing();
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
   preferences.begin("davaylora", false); // Открываем пространство имен (только чтение/запись)
   
   // Если ключа в памяти нет (первый запуск), берется второй аргумент (дефолт)
   workAddress = preferences.getUChar("workAddress", 4);
   measurebattery = preferences.getBool("measureBat", true);
   pwmledBrightness = preferences.getInt("bigLedBright", 35);
   fbledBrightness = preferences.getInt("fbLedBright", 255);
   pingTimeout = preferences.getULong("pingTimeout", 3000);
   bigTimeout = preferences.getULong("bigTimeout", 3600000);
   
   preferences.end();
   DEBUGln(F("Config loaded."));
 }
 
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
 }
 
 // ======================= ЛОГИКА ПРЕРЫВАНИЙ И СЕТИ =======================
 
 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR
 #endif
 void setFlag(void) {
   receivedFlag = true;
 }
 
 // Отправка пакета данных (без ожидания ответа)
 void sendMessage(byte msgCmd, byte sndData) {
   DEBUGln(">>>sendMessage()");
   byte payload[3] = {workAddress, msgCmd, sndData};
   
   int state = radio.transmit(payload, 3);
   
   if (state == RADIOLIB_ERR_NONE) {
     DEBUGln(("\tMessage sent: ") + String(workAddress) + " " + String(msgCmd) + " " + String(sndData));
   } else {
     DEBUGln(("\tTransmit failed, code: ") + String(state));
   }
   
   lastSendTime = millis();
   radio.startReceive(); // Возврат в режим ожидания ответа от RX
 }
 
 // Проверка эфира
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
 
 // Синхронная сессия связи (Отправка -> Ожидание ответа с ретраями)
 bool commSession(byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes) {
   DEBUGln(F("commSession()"));
   wasReceived = false;
   cmdExpected = expectedReply; // Какую команду ждем от приёмника
   pingTimer = millis();
   
   do {
     checkReceive();
     // Если время вышло, пробуем отправить пакет снова (retries)
     EVERY_MS(waitMilliseconds) {
       sendMessage(msgCmd, sndData);
       doTimes--;
     }
   } while ((doTimes > 0) && (!wasReceived)); // Крутимся, пока не получим ответ или не кончатся попытки
   
   pingTimer = millis();
   return wasReceived; // Возвращает true, если связь успешна
 }
 
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
   radio.setPreambleLength(8);     // Согласовано с RX
   radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE); // Согласовано с RX
 }
 
 // Парсинг ответа (ACK) от приёмника
 void onReceive(byte* payload, int packetSize) {
   DEBUGln("<<<PackageReceived");
 
   rcvAddress = payload[0];
   if ((rcvAddress != workAddress) || (packetSize != 3)) {
     delay(30);
     return;
   }
 
   rcvCmd = payload[1];
   // Проверяем, ответил ли приёмник ожидаемой командой
   if (rcvCmd != cmdExpected) {
     DEBUGln("\tInvalid Reply: " + String(rcvCmd) + F(", Expected: ") + String(cmdExpected));
     delay(30);
     return;
   }
   rcvData = payload[2];
 
   // Анализ задержки (Turnaround) помогает адаптировать таймауты
   lastTurnaround = millis() - lastSendTime;
   lastFrequencyError = radio.getFrequencyError();
   lastRSSI = radio.getRSSI();
   lastSNR = radio.getSNR();
   
   DEBUGln("\tReceived Message: "  + String(rcvAddress) +" " + String( rcvCmd) + " " + String( rcvData));
   DEBUGln(("\tTurnaround: ") + String(lastTurnaround));
   DEBUGln(F("=== onReceive done ==="));
 
   wasReceived = true; // Успешный прием
 }
 
 // ======================= БИЗНЕС-ЛОГИКА (КНОПКА И ПИНГ) =======================
 
 // Обработка физического нажатия главной кнопки (с учетом стейт-машины)
 void processButton() {
  prevButtonState = currButtonState;
  currButtonState = !digitalRead(PIN_BUTTON); // Инверсия (INPUT_PULLUP замыкает на землю)
  
  if (prevButtonState != currButtonState) {
    lastButtonTime = millis();
    pingTimer = millis();
    buttonPressedFirstTime = true;
    DEBUGln("\nprocessButton(): " + String(currButtonState));
    prevButtonState = currButtonState;
    
    if (currButtonState) {
      // Кнопка НАЖАТА
      buttonPressStartTime = millis(); // Фиксируем время нажатия для таймера 10 сек
      
      if (currentState == STATE_NORMAL) {
        if (commSession(CMD_SIGNAL, 1, CMD_SIGNAL_OK, 2 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
          updateStatusLed(true);
          updateBIGLed(true);
        } else {
          updateBIGLed(false);
          flashStatusLed(2);
        }
      }
    } else {
      // Кнопка ОТПУЩЕНА
      if (currentState == STATE_NORMAL) {
        sendMessage(CMD_SIGNAL, false);
        updateStatusLed(false);
        updateBIGLed(false);
      }
    }
  }
}

 // Генератор пингов (если кнопка удерживается, нужно подтверждать RX, что мы на связи)
 void processPing() {
   if (!buttonPressedFirstTime) return; // Пингуем, только если кнопка зажата
   
   // Управление кратковременным морганием светодиода при отправке пинга
   if (pingFlash) {
     if ((millis() - pingFlashTimer) > PING_FLASH) {
       DEBUGln(F("\tPing LED OFF"));
       pingFlash = false;
       updateStatusLed(currButtonState);
     }
   } else if ((millis() - pingTimer) > pingTimeout) {
     DEBUGln(F("\nStart Ping"));
     // Отправляем пинг и ждем ответ (commSession)
     if (commSession(CMD_PING, currButtonState, CMD_PING_OK, 5 * lastTurnaround, WORK_COMM_ATTEMPTS)) {
       DEBUGln(F("\tPing LED ON"));
       updateStatusLed(!currButtonState); // Инвертируем диод для вспышки
       pingFlash = true;
       pingFlashTimer = millis();
       pingTimer = millis();
     } else {
       flashStatusLed(2); // Ошибка: потеряли связь во время удержания
     }
   }
   
   // Защита от долгого удержания (уснули на пульте или залипла кнопка в сумке)
   if ((millis() - lastButtonTime) > bigTimeout) {
     flashStatusLed(3);
     buttonPressedFirstTime = false;
   }
 }
 
 void processPreparationMode() {
  // Мигание статусным диодом 3 Гц (период ~333 мс)
  EVERY_MS(166) {
    static bool prepLedState = false;
    prepLedState = !prepLedState;
    updateStatusLed(prepLedState);
  }

  // Если прошло 10 секунд бездействия - возвращаемся в норму
  if (millis() - prepModeTimer > 10000) {
    DEBUGln(F("Exit Preparation Mode (Timeout)"));
    currentState = STATE_NORMAL;
    updateStatusLed(false);
  }
}

 // ======================= ИНДИКАЦИЯ =======================
 
 // Обратная связь на встроенном LED (На TX используем digitalWrite для экономии ресурсов)
 void updateStatusLed(bool ledStatus) {
   digitalWrite(PIN_FB_LED, ledStatus);
 }
 
 // Управление мощным внешним LED (через ШИМ)
 void updateBIGLed(bool ledStatus) {
   analogWrite(PIN_BIG_LED, ledStatus * pwmledBrightness);
 }
 
 void flashStatusLed(byte times) {
   for (int i = 0; i < times; i++) {
     updateStatusLed(true);
     delay(100);
     updateStatusLed(false);
     delay(200);
   }
 }
 
 // ======================= УПРАВЛЕНИЕ БАТАРЕЕЙ =======================
 
 bool testBattery() {
   DEBUGln(F("Проверка измерителя напряжения батареи Heltec WSL V3..."));
   if (batteryVoltageOK(5)) {
     DEBUGln(F("Батарея подключена и измеритель РАБОТАЕТ!"));
     return (true);
   }
   DEBUGln(F("Батарея НЕ ПОДКЛЮЧЕНА (или сбой измерителя)!"));
   return (false);
 }
 
 bool batteryVoltageOK(byte tries) {
   DEBUGln(F("Is battery voltage OK?"));
   float minV = 5.0;
   float maxV = 0.0;
   
   for (byte i = 0; i < tries; i++) {
     float currentVBat = batteryVoltage();
     
     if (currentVBat < minV) minV = currentVBat;
     if (currentVBat > maxV) maxV = currentVBat;
     
     if ((currentVBat > 4.3) || (currentVBat < 2.5)) {
       DEBUGln(F("Voltage out of bounds (No battery or deeply discharged)!"));
       return (false);
     }
     
     delay(150);
   }
   
   DEBUG(F("Min V: ")); DEBUG(minV); DEBUG(F(", Max V: ")); DEBUGln(maxV);
   
   if ((maxV - minV) > 0.05) {
     DEBUGln(F("Voltage is unstable (Sawtooth ripple)! Батарея НЕ подключена!"));
     return (false);
   }
   
   return (true);
 }
 
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
 }
 
 void showBatteryVoltage() {
   float voltage = batteryVoltage();
   if (voltage > BATTERY_VOLTAGE_1) flashBatteryLEDOnce();
   if (voltage > BATTERY_VOLTAGE_2) flashBatteryLEDOnce();
   if (voltage > BATTERY_VOLTAGE_3) flashBatteryLEDOnce();
   if (voltage > BATTERY_VOLTAGE_4) flashBatteryLEDOnce();
   if (voltage > BATTERY_VOLTAGE_5) flashBatteryLEDOnce();
 }
 
 void showNoBattery() {
   digitalWrite(PIN_BATTERY_LED, 1);
   delay(2000);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(250);
 }
 
 // Зажигание диода проверки батареи. Работает через цифровой сигнал (digitalWrite)
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
 
 void processBattery() {
   if (batteryVoltage() < BATTERY_MIN_VOLTAGE) {
     stopWorking();
   }
 }
 
 // Аварийное засыпание передатчика
 void stopWorking() {
   flashLedBattery(7);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(5000);
   flashLedBattery(7);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(5000);
   
   esp_deep_sleep_start();
 }
 
 // ======================= ОСНОВНЫЕ ФУНКЦИИ (SETUP & LOOP) =======================
 
 void setup() {
   delay(2000);
 
 #ifdef DEBUG_ENABLE
   Serial.begin(115200);
   while (!Serial);
 #endif
 
   DEBUGln("================================");
   DEBUGln("=========== START TX ===========");
 
   // Загружаем настройки из энергонезависимой памяти
   loadConfig();
 
   // Настройка питания периферии и делителя АЦП
   pinMode(PIN_VEXT, OUTPUT);
   digitalWrite(PIN_VEXT, HIGH);
   pinMode(PIN_ADC_CTRL, OUTPUT);
   digitalWrite(PIN_ADC_CTRL, HIGH);
 
   // Инициализация рабочих пинов
   pinMode(PIN_BUTTON, INPUT_PULLUP);
   pinMode(PIN_FB_LED, OUTPUT);
   pinMode(PIN_BIG_LED, OUTPUT);
   pinMode(PIN_BATTERY_LED, OUTPUT);
   
   // Принудительный сброс состояний
   digitalWrite(PIN_FB_LED, 0);
   analogWrite(PIN_BIG_LED, 0);
   digitalWrite(PIN_BATTERY_LED, 0);
   delay(300);
 
   // Приветственный сигнал (Стартовая проверка)
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
 
   // Замер батареи перед переходом в боевой режим
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
       flashStatusLed(6);
       delay(4000);
     }
   }
 
   setLoRaParams();
 
   // Подключение аппаратного прерывания (чтобы слушать ответы от RX)
   radio.setDio1Action(setFlag);
   radio.startReceive();
   delay(100);
 
   DEBUGln("DavayLoRa TX setup complete, waiting for 1-st buttonpress");
 }
 

 void loop() {
  checkReceive(); 
  
  if ((millis() - lastButtonTime) > DEBOUNCE_TIME) {
    processButton();
  }
  
  // Логика перехода в режим подготовки (удержание 10 сек в нормальном режиме)
  if (currentState == STATE_NORMAL && currButtonState) {
    if (millis() - buttonPressStartTime > 10000) {
      DEBUGln(F("Enter Preparation Mode!"));
      currentState = STATE_PREPARATION;
      prepModeTimer = millis(); // Запускаем таймер бездействия
      
      // Выключаем рабочий сигнал, если он был отправлен
      sendMessage(CMD_SIGNAL, false); 
      updateBIGLed(false);
      
      // Искусственно сбрасываем состояние кнопки, чтобы не отправлять лишних команд
      buttonPressedFirstTime = false; 
    }
  }

  // Маршрутизация стейт-машины
  if (currentState == STATE_NORMAL) {
    processPing();
  } else if (currentState == STATE_PREPARATION) {
    processPreparationMode();
  }
  
  EVERY_MS(BATTERY_PERIOD) {
    if (measurebattery) processBattery();
  }
}