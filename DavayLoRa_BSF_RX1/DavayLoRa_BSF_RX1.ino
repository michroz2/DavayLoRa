//  Для возможных изменений в коде пометить здесь слово МЕНЯТЬ, и искать его (Ctrl-F)

//============================МЕНЯТЬ===================================
//МЕНЯТЬ синхронно для TX и RX в диапазоне 0-254
#define WORK_ADDRESS 1

//МЕНЯТЬ яркость большого ЛЕДа, - аккуратно, не сильно больше 35
//BUZZER_BIPPER_VOLUME - между 0 (выключен) и 255 (максимум) - ближе к макс.
#define BIG_LED_BRIGHTNESS 35
#define BUZZER_BIPPER_VOLUME 255

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

//МЕНЯТЬ -
#define BATTERY_PERIOD 300000 //(5 минут) 
//Каждые столько миллисекунд измеряется напряжение батареи 

//МЕНЯТЬ:
/*ПРОБЛЕМА: У некоторых модулей BSFrance не работает встроенный измеритель напряжения
  (для определения чего следует воспользоватьс программой BSFTest.ino со включенным монитором).
  Чтобы исользовать такие модули, следует добавить в схему 2 одинаковых резистора
  по 10-100КОм следующим образом:
  - один вывод каждого резистора паять на (+) и (-) разъёма батареи
  - вторые выводы соединить вместе и присоединить к пину (на выбор) A0 - A5
  (какой удобнее, например A1).
  Следующий define:
  #define PIN_BATTERY A1 - для «исправленных» модулей
  #define PIN_BATTERY 9 - для штатного измерителя батарейки
*/
#define PIN_BATTERY A1  // Номер пина Адафрута, куда припаяны резисторы
                        //для измерения батарейки

/*Коэффициент делителя для измерения батарейки.
  Поставить 2 для исправленного BSFrance «с резисторами»:
  Для «нормального» BSFrance поставить 1.27;
  Для «нормального» Adafruit поставить 2;
*/
#define BATTERY_VOLTAGE_MULTIPLIER 2

//МЕНЯТЬ Cutoff settings:
#define CUTOFF_TIME 2000  //ms - максимальное время нерпрерывной работы сигнала
//после чего он самостоятельно выключается

//============================//МЕНЯТЬ===================================

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
  analogWrite(PIN_SIGNAL_BUZZERS, signalStatus * BUZZER_BIPPER_VOLUME);
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

unsigned long workingFrequency[20] =
{
43400E4,
43412E4,
43424E4,
43382E4,
43370E4,
43394E4,
43403E4,
43415E4,
43427E4,
43385E4,
43373E4,
43397E4,
43406E4,
43418E4,
43388E4,
43376E4,
43409E4,
43421E4,
43391E4,
43379E4,
};
