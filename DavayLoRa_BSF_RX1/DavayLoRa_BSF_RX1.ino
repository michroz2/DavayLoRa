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
