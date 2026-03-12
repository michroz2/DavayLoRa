#include <Arduino.h>
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%z_Description.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/*
  Код ПРИЁМНИКА для проекта DavayLoRa
  RX принимает  команды от TX, когда там нажата/отпущена кнопка, или TX пингует его по таймеру
  Посылается ответ-подтверждение на каждое сообщение.
  Проектировался под LoRa Adafruit Feather32u4 433MHz module
  В дальнейшем адаптирован для более доступной платы BSFrance LoRa32u4 - которая ПОЧТИ копия.
  Отличия модулей:
  - физические размеры
  - делители напряжения измерения батарейки - требуется произвести подстройку в коде
  - У BSFrance нужно перерезать перемычку на плате, помеченную: "Closed DI01 -> 6" )
 - У BSFrance замечена проблема со встроенным определителем напряжеения.

 Принята (на март 2023) такая стратегия:
Предварительно сортируем платы с помощью программы LoRa32u4BatteryMesureCheck.ino, включив Ардуино монитор.
ХОРОШИЕ ставим на приёмник, а плохие - добавляем резисторы и ставим на передатчик.
Таким образом, в приёмнике ДОЛЖНЫ оказываться только рабочие платы.
Если по каким-то причинам плата перестала мерять напряжение, то устанавливаем параметр чтобы не мерять батарейку
(резисторы на приёмник НЕ паяем)  

  Для каждой пары TX-RX надо указать в коде одинаковую рабочую частоту
  (изменять её рекомендуется по 0.1 мегагерц, в пределах рабочего диапазона 433.05E6 - 434.79E6)
  и/или выбрать одинаковый совместный байт WORK_ADDRESS в пределах 0 - 255

  Подготовка и СОЕДИНЕНИЯ (см. также схему Fritzing и картинку оттуда):
  - Перерезать перемычку на плате BSFrance, помеченную: "Closed DI01 -> 6" )
  - Перерезать дорожку на обратной стороне платы - ту, которая идёт вдоль платы в нижней части напечатанной буквы “u” в слове LoRa32u4
  (Подробнее - см. DAVAY RoNik Whitepaper )
  - DIP3 переключатели выбора вызывающих эффектов
      один контакт всех 3 переключателей совместно -> BAT микропроцессора
      противоположные контакты (точный порядок в принципе неважен, но мы придерживаемся):
        1 -> (+) основных LED
        2 -> (+) биппера
        3 -> (+) баззера
  - Баззер и биппер -
      (+) -> на DIP3 - см. выше
      (-) -> совместно на сток (drain или D) - центральный контакт MOSFET-2
  - Большие светодиоды индикации вызова
      плюс -> на DIP3 - см. выше
      R (Red) -> сток (drain) полевого тр-ра MOSFET-1 (центральный вывод)
  - MOSFET-1 60NO3/60NO2 - управляет включением ЛЕДов
      управляющий (gate) полевого тр-ра (левый вывод) -> 6 микропроцессора
      исток (source) полевого тр-ра (правый вывод) -> GND
  - MOSFET-2 60NO3/60NO2 - управляет включением баззера и биппера
      управляющий (gate) полевого тр-ра (левый вывод) -> 5 микропроцессора
      исток (source) полевого тр-ра (правый вывод) -> GND
  - Переключатель выключения
      центр (или край) -> BAT
      край (или центр) -> «+» вывод разъёма батареи на плате
      при замкнутом переключателе прибор Включен
      при разомкнутом переключателе - прибор вЫключен (однако, может «как-то» работать, когда на зарядке)
  - Батарею LiPo 1S подключить к своему JST разъёму

    USB порт можно использовать для зарядки батареи и для заливки прошивки в любое время

*/
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%DavayLoRa_BSF_RX1.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//  Для изменений пометить слово МЕНЯТЬ, и искать его (Ctrl-F)

//============================МЕНЯТЬ===================================
//МЕНЯТЬ синхронно для TX и RX в диапазоне 0-254
//! На самом деле пока сделаны разные частоты на 20 адресов, а потом частоты повторяются !
//Это означает, что приборы с адресами, отличающимися на 20 номеров, могут немного мешать друг другу при одновременной работе
//ВАЖНОЕ ЗНАЧЕНИЕ чтобы приборы работали парами!!!
//... и, в принципе, это всё что НАДО менять. Остальное можно не трогать.
#define WORK_ADDRESS 4 //По традиции первая прошивка: оставить «4» 
                       //Это позволяет при изготовлении проверить все приёмники и передатчики со всеми
                       //После всех проверок вернуться и настроить нужный адрес 

//МЕНЯТЬ яркость большого ЛЕДа, - аккуратно!, не сильно превышать 35, а то будет греться и сажать батарейку
//BUZZER_BIPPER_VOLUME - между 0 (выключен) и 255 (максимум) - ставить ближе к макс.
//Это менять только если партия ЛЕДов или конкретный ЛЕД (баззер/биппер) будет слишком сильно работать или наоборот
#define BIG_LED_BRIGHTNESS 35
#define BUZZER_BIPPER_VOLUME 255

//МЕНЯТЬ Cutoff settings:
#define CUTOFF_TIME 2000  //ms - максимальное время непрерывной работы сигнала
//после чего он самостоятельно выключается, даже если кнопка передатчика удерживается нажатой
//(в этом случае в приёмнике будет получаться периодический сигнал в течение CUTOFF_TIME с паузой PING_TIMEOUT)

//МЕНЯТЬ - если хочется сделать другую периодичность Пинга:
// считается, что пинг-таймаут приёмника должен быть больше, чем у передатчика,
//например: 3сек-5сек
#define PING_TIMEOUT 5000  //ms 

//МЕНЯТЬ: Далее идёт батарейный раздел, менять, в принципе, ничего не надо:
#define MEASURE_BATTERY true  // true или false = включить или выключить измерение батарейки
//Это будет определяться автоматически, но если хочется вообще выключить, то поставить false 

//МЕНЯТЬ - напряжение «отсечки» - по результатам использования
#define BATTERY_MIN_VOLTAGE 3.5   //Volt min.
#define BATTERY_VOLTAGE_1 3.6 // Мигание 1 раз, если V выше этого значения
#define BATTERY_VOLTAGE_2 3.7 // Мигание 2 раза
#define BATTERY_VOLTAGE_3 3.8 // Мигание 3 раза
#define BATTERY_VOLTAGE_4 3.9 // Мигание 4 раза
#define BATTERY_VOLTAGE_5 4.0 // Мигание 5 раз

//МЕНЯТЬ ? -
#define BATTERY_PERIOD 300000 //(= 5 минут) 
//Каждые столько миллисекунд непрерывной работы прибора измеряется остаточное напряжение батареи

#define PIN_BATTERY_RESISTOR 19 //то же что и A1 - Это номер пина, куда (если) запаяны оба резистора
//Менять, только если, например, случайно резисторы оказались запаяны не туда... 18=А0, 19=А1, 20=А2 и т.д.

/*Коэффициент делителя для измерения батарейки.
  Для «нормального» BSFrance поставить 1.27;
  Для «нормального» Adafruit поставить 2;
  Оставлено, чтобы не забыть данный факт. Трогать не надо.
*/
#define INTERNAL_VOLTAGE_MULTIPLIER 1.27  // Для платы BSFrance 1.27   Для Adafruit поставить 2;

//============================//МЕНЯТЬ-ЗАКОНЧЕНО!=================================

#define PIN_BATTERY_INTERNAL 9    // Номер внутреннего пина для измерения батарейки
#define RESISTOR_VOLTAGE_MULTIPLIER 2 //Если мы паяем делитель, то это всегда 2 одинаковых резистора

#define WORK_FREQUENCY 434E6  //Это значение ничего не определяет. Частота определяется автоматически по номеру адреса

float batteryVoltageMultiplier = 1.27; //Может измениться, если мы не обнаружим внутреннего делителя!
int batteryPIN = PIN_BATTERY_INTERNAL; //Для начала будем искать внутренний делитель                        //
bool measurebattery = MEASURE_BATTERY; //Для начала будем считать, что батарейка измеряется

#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <GyverPower.h>


// Дебагирование с монитором: раскомментить для использования 1 строчку:
//#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.print(x)
#define DEBUGln(x) Serial.println(x)
#else
#define DEBUG(x)
#define DEBUGln(x)
#endif

//==== MILLIS TIMER MACRO ====
// воспомогательное макро
// performs the {subsequent} code once and then, if needed, again after each x ms
#define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flg = millis() - tmr >= (x);\
  if (flg) tmr = millis();\
  if (flg)
//===========================

#define MAX_ADDRESS 20 //пока сделано такое максимальное количество частот

#define PIN_SIGNAL_LED  6  // Номер пина для вывода сигнала для ЛЕДа
#define PIN_SIGNAL_BUZZERS  5  // Номер пина для вывода сигнала для Баззера и Вибро
#define PIN_STATUS_LED  LED_BUILTIN  // Номер пина, к которому подключен вывод статусного ЛЕД-а (LED_BUILTIN=13) 
#define PIN_BATTERY_LED LED_BUILTIN  // Номер пина для показа напряжения батарейки

//Дефолтовые значения для SPI библиотеки: 10, 9, 2. Для наших плат действуют такие:
const int csPin = 8;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 7;         // change for your board; must be a hardware interrupt pin

//Описание протокола:
#define CMD_SIGNAL         208 //TX передаёт сигнал на изменение состояния лед-а
#define CMD_SIGNAL_OK      209 //RX подтверждает
#define CMD_PING        212 //TX пингует периодически с /состоянием леда
#define CMD_PING_OK     213 //то же что предыдущее

byte workAddress = WORK_ADDRESS;  // address of connection
byte rcvAddress = 0;          // received address
byte rcvCmd = 0;              // received command
byte rcvData = 0;                  // received data
byte sndCmd = CMD_PING_OK;              // outgoing command Default = PING
byte sndData;                         // additional data byte sent
bool signalStatus;                     //last received TX Button status
byte workChannel;                      //channel to send to RX while paring and work on it
unsigned long workFrequency = WORK_FREQUENCY; //working Frequency

unsigned long lastSendTime = 0;                // last send time
int lastRSSI;
float lastSNR;
unsigned long lastTurnaround;         // round-trip time between tx and rx
long lastFrequencyError;

unsigned long pingTimeOutLastTime;

unsigned long cutoffTimer = 0;

// --- ПРОТОТИПЫ ФУНКЦИЙ (Оглавление для компилятора) ---
void processTimeOut();
void processCommand();
void processSignal();
void processCutoff();
void updateStatusLed(bool ledStatus);
void flashStatusLEDOnce();
void flashStatusLed(byte times);
void sendMessage(byte msgAddr, byte msgCmd, byte msgData);
void setLoRaParams();
bool testBattery();
bool batteryVoltageOK(byte tries);
float batteryVoltage();
void processBattery();
void showBatteryVoltage();
void showNoBattery();
void flashBatteryLEDOnce();
void flashLedBattery(byte times);
void stopWorking();
// ------------------------------------------------------


void   processTimeOut() {
  if ((millis() - pingTimeOutLastTime) > PING_TIMEOUT) { // if long time no signal from TX
    DEBUGln(F("ZZZZZZZ"));
    signalStatus = false;
    pingTimeOutLastTime = millis();
    flashStatusLed(2);
  }
}// processTimeOut()

void processCommand() {
  switch (rcvCmd) {
    case CMD_SIGNAL:
      DEBUGln(F("=== CMD_SIGNAL ==="));
      signalStatus = rcvData;
      processSignal();
      if (signalStatus)
        sendMessage(rcvAddress, CMD_SIGNAL_OK, signalStatus);
      break;
    case CMD_PING:
      unsigned long flashStatus = millis();
      DEBUGln(F("=== CMD_PING ==="));
      signalStatus = rcvData;
      processSignal();
      updateStatusLed(true);
      unsigned long restDelay = 100 - (millis() - flashStatus);
      if (restDelay > 0) {
        delay(restDelay);
      }
      updateStatusLed(false);
      sendMessage(rcvAddress, CMD_PING_OK, signalStatus);
  }
  rcvCmd = 0;
} //void processCommand()


void  processSignal() {
  cutoffTimer = millis();
  analogWrite(PIN_SIGNAL_LED, signalStatus * BIG_LED_BRIGHTNESS);
  analogWrite(PIN_SIGNAL_BUZZERS, signalStatus * BUZZER_BIPPER_VOLUME);
  digitalWrite(PIN_STATUS_LED, signalStatus); //In case all DIPs are out, at least the status LED blinks!
}

void processCutoff() {
  if (millis() - cutoffTimer > CUTOFF_TIME) {
    signalStatus = 0;
    analogWrite(PIN_SIGNAL_LED, 0);
    digitalWrite(PIN_SIGNAL_BUZZERS, 0);
    digitalWrite(PIN_STATUS_LED, 0);
  }
}

void updateStatusLed(bool ledStatus) { // turn ON or OFF the status LED
  digitalWrite(PIN_STATUS_LED, ledStatus);
  //  DEBUGln("updateStatusLed: " + String(ledStatus));
}// updateStatusLed(bool ledStatus)

void flashStatusLEDOnce() {
  digitalWrite(PIN_STATUS_LED, 1);
  delay(250);
  digitalWrite(PIN_STATUS_LED, 0);
  delay(250);
}//void flashStatusLEDOnce()

void flashStatusLed(byte times) { //flash "times" times
  for (int i = 0; i < times; i++) {
    flashStatusLEDOnce();
  }
  delay(200);
}//void flashStatusLed(byte times)

//long frequencyByChannel(byte numChannel) {
//  DEBUGln(F("frequencyByChannel(byte numChannel)"));
//  if (numChannel > NUM_LORA_CHANNELS) {
//    DEBUGln("Invalid Channel: " + String(numChannel));
//    return CALL_FQ;
//  }
//  return (minFQ + numChannel * CHANNEL_WIDTH);
//}
//// done frequencyByChannel(byte numChannel)

void sendMessage(byte msgAddr, byte msgCmd, byte msgData) {
  DEBUGln(F(">>>Sending Message"));

  while (!LoRa.beginPacket()) {
    DEBUGln(F("\tWaiting to begin REPLY"));
  }                   // start packet
  LoRa.write(msgAddr);              // reply address
  LoRa.write(msgCmd);                  // reply command
  LoRa.write(msgData);                 // reply Data
  while (!LoRa.endPacket()) {            // finish packet and send it
    DEBUGln(F("\tWaiting to finish REPLY"));
  }
  lastSendTime = millis();            // timestamp the message
  pingTimeOutLastTime = lastSendTime; //variable for Ping Timeout
  LoRa.receive();                     // go back into receive mode
  DEBUGln(("\tMessage sent: ") + String(msgAddr)\
          + " " + String(msgCmd) + " " + String(msgData));
}// void sendMessage(byte messageByte)

//void onTxDone() {
//  DEBUGln("onTxDone()");
//  lastSendTime = millis();            // timestamp the message
//  LoRa.receive();                     // go back into receive mode
//}

unsigned long workingFrequency[MAX_ADDRESS] =
{
  434000000,
  434120000,
  434240000,
  433820000,
  433700000,
  433940000,
  434030000,
  434150000,
  434270000,
  433850000,
  433730000,
  433970000,
  434060000,
  434180000,
  433880000,
  433760000,
  434090000,
  434210000,
  433910000,
  433790000,
};
//%%%%%%%%%%%%%%%%%%%%%%%%%%%loraParams.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
void setLoRaParams() {
    DEBUGln("setLoRaParams()");
    //Trying optimal settings for LoRa for Longest Range possible
    //Use LoRa Modem Calculator Tool found on
    //https://www.semtech.com/products/wireless-rf/lora-core/sx1276#download-resources
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);    //max
    delay(50);
    LoRa.setSignalBandwidth(125E3);                 //..31.25E3, 41.7E3, 62.5E3, (125E3), and 250E3.
    delay(50);
    LoRa.setSpreadingFactor(8);                    //default = 7
    delay(50);
    LoRa.setPreambleLength(6);                    //min = 6, default = 8
    delay(50);
    LoRa.setSyncWord(WORK_ADDRESS);
  //  LoRa.enableCrc();                             //
    delay(50);
    //  LoRa.setCodingRate4(5);
  
  }// DONE void setLoRaParams()
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%batteryStuff.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
bool testBattery()   {//This function defines if the battery can be measured

    // 1) Проверка внутреннего делителя на пине 9 :
    DEBUGln(F("1. Проверка встроенного измерителя напряжения батареи"));
    batteryVoltageMultiplier = INTERNAL_VOLTAGE_MULTIPLIER;
    batteryPIN = PIN_BATTERY_INTERNAL;
    if (batteryVoltageOK(5)) {
      DEBUGln(F("Встроенный измеритель РАБОТАЕТ!"));
      return (true);
    }
    DEBUGln(F("Встроенный измеритель НЕ РАБОТАЕТ!"));
  
    // 2) Проверка делителя на резисторах на пине 19
    DEBUGln(F("2. Проверка измерителя напряжения НА ВНЕШНИХ РЕЗИСТОРАХ, подключенных к А1"));
    batteryVoltageMultiplier = RESISTOR_VOLTAGE_MULTIPLIER;
    batteryPIN = PIN_BATTERY_RESISTOR;
    if (batteryVoltageOK(5)) {
      DEBUGln(F("Измеритель на резисторах работает!"));
      return (true);
    }
    DEBUGln(F("Измеритель на резисторах также НЕ РАБОТАЕТ!"));
    return (false);
  
  }////bool testBattery();
  
  bool batteryVoltageOK(byte tries) { //Проверка «нормальности» напряжения батареи
                                      //Поскольку на измеряемом пине может быть какое-то случайное напряжение,
                                      //Измеряем несколько значений через полсекунды и смотрим насколько оно стабильно
                                      //и влезает ли вообще в диапазон возможного напряжения батареи 
    DEBUGln(F("Is battery voltage OK?"));
    float averageVBat, currentVBat;
    for (byte i = 0; i < tries; i++) {
      currentVBat = batteryVoltage();
      averageVBat *= i;
      averageVBat += currentVBat;
      averageVBat /= (i + 1);
      if ((currentVBat > 4.5) || (currentVBat < 3)) {
        DEBUGln(F("Voltage too high or too low!"));
        return (false);
      }
      DEBUG(F("Average Voltage: "));
      DEBUGln(averageVBat);
      if (abs(currentVBat - averageVBat) > 0.2) {
        DEBUGln(F("Voltage is not steady enough!"));
        DEBUGln(F("Возможно, батарея не подключена!"));
        return (false);
      }
      delay(500);
    }
    return (true);
  }
  
  float batteryVoltage() {
    DEBUG(F("Battery Voltage: "));
    float measuredvbat = analogRead(batteryPIN);
    measuredvbat *= batteryVoltageMultiplier;    // multiply according to the used board divider
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    DEBUGln(measuredvbat);
    return measuredvbat;
  }////batteryVoltage()
  
  void processBattery() {
    if (batteryVoltage() < BATTERY_MIN_VOLTAGE) {
      stopWorking();
    }
  }////processBattery()
  
  void showBatteryVoltage() {
    float voltage = batteryVoltage();
    //  delay(1000);
    if (voltage > BATTERY_VOLTAGE_1)   flashBatteryLEDOnce(); //1 раз
    if (voltage > BATTERY_VOLTAGE_2)   flashBatteryLEDOnce(); //2 раз
    if (voltage > BATTERY_VOLTAGE_3)   flashBatteryLEDOnce(); //3 раз
    if (voltage > BATTERY_VOLTAGE_4)   flashBatteryLEDOnce(); //4 раз
    if (voltage > BATTERY_VOLTAGE_5)   flashBatteryLEDOnce(); //5 раз
  }
  
  void showNoBattery() { //long flash if no battery measurement
    digitalWrite(PIN_BATTERY_LED, 1);
    delay(2000);
    digitalWrite(PIN_BATTERY_LED, 0);
    delay(250);
  }
  
  void flashBatteryLEDOnce() {
    digitalWrite(PIN_BATTERY_LED, 1);
    delay(250);
    digitalWrite(PIN_BATTERY_LED, 0);
    delay(250);
  }
  
  void flashLedBattery(byte times) { //flash "times" times
    DEBUGln(F("flashLedBattery()"));
    for (int i = 0; i < times; i++) {
      flashBatteryLEDOnce();
    }
    delay(200);
  }
  
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%stopWorking.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
void stopWorking() {
    flashLedBattery(7);
    digitalWrite(PIN_BATTERY_LED, 0);
    delay(3000);
    flashLedBattery(7);
    digitalWrite(PIN_BATTERY_LED, 0);
    while (1) {
      power.setSleepMode(POWERDOWN_SLEEP); // Крепко засыпаем
      delay(100); // даем время на отправку
      power.sleep(SLEEP_FOREVER); // спим до перезагрузки
  
    }
  }
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%onReceive.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
  void onReceive(int packetSize) {
    DEBUGln(F("\n<<<Package Received"));
  
    rcvAddress = LoRa.read();          // replied address
    if ((rcvAddress != workAddress) || (packetSize != 3)) {
      DEBUGln(F("Invalid package! "));
      DEBUG(F("Received address: "));
      DEBUGln(rcvAddress);
      DEBUG(F("Expected address: "));
      DEBUGln(workAddress);
      DEBUG(F("Package length: "));
      DEBUGln(packetSize);
      //    delay(30); //пропускаем немного времени - пока работает чужое радио
      return;
    }
  
    rcvCmd = LoRa.read();              // received command
    rcvData = LoRa.read();                  // received data
  
    lastFrequencyError = LoRa.packetFrequencyError();
  
  #ifdef DEBUG_ENABLE
    lastRSSI =  LoRa.packetRssi();
    lastSNR = LoRa.packetSnr();
    DEBUGln("\tReceived Message:\t"  + String(rcvAddress)\
            +" " + String( rcvCmd) + " " + String( rcvData));
    DEBUGln("\tRSSI:\t" + String(lastRSSI));
    DEBUGln("\tSnr:\t" + String(lastSNR));
    DEBUGln("\tFrequency Error:\t" + String(lastFrequencyError));
    DEBUGln(("\tWorking Frequency OLD:\t") + String(workFrequency));
  #endif
  
    workFrequency = workFrequency - lastFrequencyError / 2;
    LoRa.setFrequency(workFrequency);
    //  delay(30);
  
    DEBUGln(("\tWorking Frequency NEW:\t") + String(workFrequency));
    DEBUGln(F("=== onReceive done ==="));
  
  
  }//void onReceive(int packetSize)
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%1_Setup.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
  void setup() {//=======================SETUP===============================
    delay(2000);   // Give time to the ATMega32u4 port to wake up and be recognized by the OS.
    power.hardwareEnable(PWR_ALL);
  
    // initialize serial
  #ifdef DEBUG_ENABLE
    Serial.begin(9600);
    while (!Serial);
  #endif
  
    DEBUGln(F("================================"));
    DEBUGln(F("=========== START RX ==========="));
    DEBUGln(F("DavayLoRa RX setup()"));
  
    //INIT PINS
    pinMode(PIN_STATUS_LED, OUTPUT);
    pinMode(PIN_SIGNAL_BUZZERS, OUTPUT);
    pinMode(PIN_SIGNAL_LED, OUTPUT);
    analogWrite(PIN_SIGNAL_LED, 0); //just in case - switch off FB on big led
    digitalWrite(PIN_SIGNAL_BUZZERS, 0);
    digitalWrite(PIN_BATTERY_LED, 0);
    delay(300);
  
    //Приветственный сигнал 1 сек включаем все исполнительные элементы
    //(если они выключены ДИПами, то вспыхнет только статусный ЛЕД на плате!)
    updateStatusLed(true);
    analogWrite(PIN_SIGNAL_LED, BIG_LED_BRIGHTNESS);
    analogWrite(PIN_SIGNAL_BUZZERS, BUZZER_BIPPER_VOLUME);
    delay(1000);
    updateStatusLed(false);
    analogWrite(PIN_SIGNAL_LED, 0);
    digitalWrite(PIN_SIGNAL_BUZZERS, LOW);  //LOW is the same as 0
    delay(1000);
  
    DEBUGln(F("Battery Test"));
    measurebattery = testBattery();     //This function defines if the battery can be measured
    if (measurebattery) {
    DEBUGln(F("-Measuring"));
      processBattery(); //Если заряд батарейки недостаточен, то моргаем 2 серии по 7 раз и выключаемся
      // два раза показываем заряд батарейки:
      showBatteryVoltage();
      delay(2000);   //
      showBatteryVoltage();
      delay(500);   //
    }
    else {
      DEBUGln(F("-Cancelled"));
      showNoBattery();
      delay(500);   //
    }
  
    // override the default CS, reset, and IRQ pins (optional)
    LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
    delay(300);
  
    workFrequency = workingFrequency[WORK_ADDRESS % MAX_ADDRESS];
    DEBUG("LoRa begin on ");
    DEBUGln(workFrequency);
    if (!LoRa.begin(workFrequency)) {             // initialize radio at workFrequency
      DEBUGln(F("LoRa init failed. Check your connections."));
      while (true) {
        flashStatusLed(6);    // if LoRa failed, blink 6 times and do nothing
        delay(4000);
      }
    }
  
    setLoRaParams();
  
    LoRa.onReceive(onReceive);
    //  LoRa.onTxDone(onTxDone);
    LoRa.receive(); //Always listen by default
  
    pingTimeOutLastTime = millis();
  
    DEBUGln(F("DavayLoRa RX setup complete"));
  }//setup      //===================END SETUP===============================
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%2_Loop.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
void loop() { //  ===!!!===!!!===!!!===LOOP===!!!===!!!===!!!===!!!===!!!===

    if (rcvCmd)
      processCommand();
    else
      processTimeOut();  //see if we haven't received from TX for long enough
  
    processCutoff();
  
    EVERY_MS(BATTERY_PERIOD) {
      if (measurebattery) {
        processBattery();
      }
    }
  
  }//loop()         ===!!!===!!!===!!!===END LOOP===!!!===!!!===!!!===!!!===!!!===
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   