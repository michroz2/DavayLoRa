//  Для возможных изменений в коде пометить здесь слово МЕНЯТЬ, и искать его (Ctrl-F)

/*
  Код ПРИЁМНИКА для проекта DavayLoRa
  RX принимает  команды от TX, когда там нажата/отпущена кнопка, или TX пингует его по таймеру
  Посылается ответ-подтверждение на каждое сообщение.
  Проектировался под LoRa Adafruit Feather32u4 433MHz module
  В дальнейшем адаптирован для более доступной платы BSFrance LoRa32u4 - которая ПОЧТИ копия.
  Отличия модулей:
  - физические размеры
  - отличаются делители напряжения измерения батарейки - следует произвести подстройку в коде
  - У BSFrance нужно перерезать перемычку на плате, помеченную: "Closed DI01 -> 6" )
  Для каждой пары TX-RX надо указать в коде одинаковую рабочую частоту
  (изменять её рекомендуется по 0.1 мегагерц, в пределах рабочего диапазона 433.05E6 - 434.79E6)
  и/или выбрать одинаковый совместный байт WORK_ADDRESS

  СОЕДИНЕНИЯ (см. также схему Fritzing и картинку):
  - Перерезать перемычку на плате BSFrance, помеченную: "Closed DI01 -> 6" )
  - DIP3 переключатели выбора вызывающих эффектов
      один контакт всех 3 переключателей совместно -> BAT микропроцессора (или + батареи)
      противоположные контакты (точный порядок неважен):
        1 -> (+) основных LED
        2 -> (+) биппера
        3 -> (+) баззера
  - Баззер и биппер -
      (+) -> на DIP3 - см. выше
      (-) -> совместно на сток (drain или D) - центральный контакт MOSFET-2
  - Большие светодиоды индикации вызова
      плюс -> на DIP3 - см. выше
      R (Red) -> сток (drain) полевого тр-ра MOSFET-1 (центральный вывод)
  - MOSFET-1 60NO3 - управляет включением ЛЕДов
      управляющий (gate) полевого тр-ра (левый вывод) -> 6 микропроцессора
      исток (source) полевого тр-ра (правый вывод) -> GND
  - MOSFET-2 60NO3 - управляет включением баззера и биппера
      управляющий (gate) полевого тр-ра (левый вывод) -> 5 микропроцессора
      исток (source) полевого тр-ра (правый вывод) -> GND
  - Переключатель выключения
      центр (или край) -> GND
      край (или центр) -> EN микропроцессора
      при замкнутом переключателе прибор ВЫключен (заряжать батарейку от USB при этом можно)
      при разомкнутом переключателе - прибор включен
  - Батарею LiPo 1S подключить или припаять к своему JST разъёму

    USB порт можно использовать для зарядки батареи - в любое время
      и для заливки прошивки (при разомкнутом переключателе)
*/

#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <GyverPower.h>


// Дебагирование с компьютером : раскомментить для использования 1 строчку:
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

//Можно в принципе МЕНЯТЬ рабочую частоту (синхронно на TX и RX!)
//Желательно сильно не уходить от значения 434E6 ()
//просто добавлять-убавлять десятые, например: 433.9E6, 433.8E6, или 434.1E6, 434.2E6
#define WORK_FREQUENCY 434E6

//МЕНЯТЬ - напряжение «отсечки» - по результатам использования
#define BATTERY_MIN_VOLTAGE 3.5   //Volt min.
#define BATTERY_VOLTAGE_1 3.6 // Мигание 1 раз
#define BATTERY_VOLTAGE_2 3.7
#define BATTERY_VOLTAGE_3 3.8
#define BATTERY_VOLTAGE_4 3.9
#define BATTERY_VOLTAGE_5 4.0

//МЕНЯТЬ - надо, чтобы пинг-таймаут приёмника был больше, чем у передатчика,
//например: 3сек-5сек
#define PING_TIMEOUT 5000  //ms 

#define BATTERY_PERIOD 300000 //(5 минут) Каждые столько миллисекунд измеряется напряжение батареи 

#define PIN_SIGNAL_LED  6  // Номер пина для вывода сигнала для ЛЕДа
#define PIN_SIGNAL_BUZZERS  5  // Номер пина для вывода сигнала для Баззера и Вибро
#define PIN_STATUS_LED  LED_BUILTIN  // Номер пина, к которому подключен вывод статусного ЛЕД-а (LED_BUILTIN=13) 
#define PIN_BATTERY_LED LED_BUILTIN  // Номер пина для показа напряжения батарейки


//МЕНЯТЬ:
/*ПРОБЛЕМА: У некоторых модулей BSFrance не работает встроенный измеритель напряжения
  (для определения чего следует воспользоватьс программой BSFTest.ino со включенным монитором).
  Чтобы исользовать такие модули, следует добавить в схему 2 одинаковых резистора
  по 10-100КОм следующим образом:
  - один вывод каждого резистора паять на (+) и (-) разъёма батареи
  - вторые выводы соединить вместе и присоединить к пину (на выбор) A0 - A5
  (какой удобнее, например A2).
  Также надо скорректировать следующий define:
  #define PIN_BATTERY A2
  а также коэффициент делителя поставить 2:
  (BATTERY_VOLTAGE_MULTIPLIER)
*/
#define PIN_BATTERY A1  // Номер пина Адафрута для измерения батарейки
/*Коэффициент делителя для измерения батарейки.
  Для «нормального» BSFrance поставить 1.27;
  Для Adafruit поставить 2;
  Также поставить 2 для BSFrance «с резисторами»:
*/
#define BATTERY_VOLTAGE_MULTIPLIER 2

//Дефолтовые значения для SPI библиотеки: 10, 9, 2. Для наших плат действуют такие:
const int csPin = 8;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 7;         // change for your board; must be a hardware interrupt pin

//Описание протокола:
#define CMD_SIGNAL         208 //TX передаёт сигнал на изменение состояния лед-а
#define CMD_SIGNAL_OK      209 //RX подтверждает
#define CMD_PING        212 //TX пингует периодически с /состоянием леда
#define CMD_PING_OK     213 //то же что предыдущее

//МЕНЯТЬ синхронно для TX и RX в диапазоне 0-254
#define WORK_ADDRESS      1

//МЕНЯТЬ яркость большого ЛЕДа, - аккуратно, не сильно больше 35
#define BIG_LED_BRIGHTNESS 35

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

//МЕНЯТЬ ? Cutoff settings:
#define CUTOFF_TIME 2000  //ms - максимальное время нерпрерывной работы сигнала

unsigned long cutoffTimer = 0;

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
      if (signalStatus) {
        sendMessage(rcvAddress, CMD_SIGNAL_OK, signalStatus);
        break;
      }
    case CMD_PING:
      updateStatusLed(true);
      DEBUGln(F("=== CMD_PING ==="));
      signalStatus = rcvData;
      processSignal();
      //      unsigned long flashStatus = millis();
      sendMessage(rcvAddress, CMD_PING_OK, signalStatus);
      //      unsigned long restDelay = 100 - (millis() - flashStatus);
      //      if (restDelay > 0) {
      //        delay(restDelay);
      //      }
      updateStatusLed(false);
  }
  rcvCmd = 0;
} //void processCommand()


void  processSignal() {
  cutoffTimer = millis();
  analogWrite(PIN_SIGNAL_LED, signalStatus * BIG_LED_BRIGHTNESS);
  digitalWrite(PIN_SIGNAL_BUZZERS, signalStatus);
}

void processCutoff() {
  if (millis() - cutoffTimer > CUTOFF_TIME) {
    signalStatus = 0;
    analogWrite(PIN_SIGNAL_LED, 0);
    digitalWrite(PIN_SIGNAL_BUZZERS, 0);
  }
}

void updateStatusLed(bool ledStatus) { // turn ON or OFF the status LED
  digitalWrite(PIN_STATUS_LED, ledStatus);
  //  DEBUGln("updateStatusLed: " + String(ledStatus));
}// updateStatusLed(bool ledStatus)

void flashStatusLed(byte times) { //flash "times" times
  //  DEBUGln("flashStatusLed()");
  bool flash = true;
  for (int i = 0; i < times * 2; i++) {
    updateStatusLed(flash);
    flash = !flash;
    delay(150);
  }
}

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
