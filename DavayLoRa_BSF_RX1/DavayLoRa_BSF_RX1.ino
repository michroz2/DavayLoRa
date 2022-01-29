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

#define CALL_FQ 434E6                       //рабочая частота.
//Это остатки от программирования «паринга», тут остались просто на всякий случай
#define MIN_FQ 433.05E6                     //минимальная частота диапазона.
#define MAX_FQ 434.79E6                     //максимальная рабочая частота.
#define CHANNEL_WIDTH 1E5                 // 100 или 200 KHz
#define NUM_LORA_CHANNELS (MAX_FQ - MIN_FQ)/CHANNEL_WIDTH   //количество каналов столько кГц
long minFQ = round(MIN_FQ / 1.0E5) * 1.0E5;
//TODO: сделать выбор рабочего канала, вместо частоты:
#define WORKING_CHANNEL 5
#define BROADCAST_ADDRESS 0xFF

//МЕНЯТЬ - напряжение «отсечки» - по результатам использования
#define BATTERY_MIN_VOLTAGE 3.5   //Volt min.
#define BATTERY_VOLTAGE_1 3.6 // Мигание 1 раз
#define BATTERY_VOLTAGE_2 3.7
#define BATTERY_VOLTAGE_3 3.8
#define BATTERY_VOLTAGE_4 3.9
#define BATTERY_VOLTAGE_5 4.0

//МЕНЯТЬ - надо, чтобы пинг приёмника был больше, чем у передатчика
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
  а также коэффициент делителя поставить равным 2:
  float batteryVoltageMultiplier = 2;
*/
#define PIN_BATTERY A1  // Номер пина Адафрута для измерения батарейки
/*Коэффициент делителя для измерения батарейки.
  Для «нормального» BSFrance поставить 1.27;
  Для Adafruit поставить 2;
  Также поставить 2 для BSFrance с неисправным измерителем напряжения:
*/
float batteryVoltageMultiplier = 2;

//Дефолтовые значения для SPI библиотеки: 10, 9, 2. Для наших плат действуют такие:
const int csPin = 8;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 7;         // change for your board; must be a hardware interrupt pin

//Практически, описание протокола:
#define CMD_PAIRING     200 //TX передаёт команду/команду с бродкастным адресом
#define CMD_PAIRING_OK  201 //RX отвечает /свой зашитый адрес с бродкастным адресом
#define CMD_ADDR        202 //TX передаёт по адресу полученный адрес/адрес
#define CMD_ADDR_OK     203 //RX отвечает /адрес с адресом
#define CMD_CHANNEL     204 //TX передаёт свой будущий /канал
#define CMD_CHANNEL_OK  205 //RX отвечает, что переключается/канал
#define CMD_START       206 //TX передаёт на канале старт/старт
#define CMD_START_OK    207 //RX отвечает на канале ok/ok - паринг закончился
#define CMD_SIGNAL         208 //TX передаёт сигнал на изменение состояния лед-а
#define CMD_SIGNAL_OK      209 //RX подтверждает
#define CMD_PING        212 //TX пингует периодически с /состоянием леда
#define CMD_PONG        213 //RX отвечает с состоянием леда
#define CMD_PING_OK     213 //то же что предыдущее

//МЕНЯТЬ синхронно для TX и RX в диапазоне 0-254
#define WORK_ADDRESS      1

byte workAddress = WORK_ADDRESS;  // address of connection
byte msgNumber = 0;                    // = number of the received message: reply always this number
byte rcvAddress = 0;          // received address
byte rcvCount = 0;            // received Number
byte rcvCmd = 0;              // received command
byte rcvData = 0;                  // received data
byte sndCmd = CMD_PONG;              // outgoing command Default = PING
byte sndData;                         // additional data byte sent
bool signalStatus;                     //last received TX Button status
byte workChannel;                      //channel to send to RX while paring and work on it
unsigned long workFrequency; //working Frequency

unsigned long lastSendTime = 0;                // last send time
int lastRSSI;
float lastSNR;
unsigned long lastTurnaround;         // round-trip time between tx and rx
long lastFrequencyError;

unsigned long pingTimeOutLastTime;

int pwmledBrightness = 35;           // 0 - 255 - Яркость большого леда

//МЕНЯТЬ ? Cutoff settings:
unsigned long cutoffTimer = 0;
#define CUTOFF_TIME 2000  //ms - максимальное время работы сигнала


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

  //Приветственный сигнал 1 сек
  updateStatusLed(true);
  analogWrite(PIN_SIGNAL_LED, pwmledBrightness);
  digitalWrite(PIN_SIGNAL_BUZZERS, HIGH);
  delay(1000);
  updateStatusLed(false);
  analogWrite(PIN_SIGNAL_LED, 0);
  digitalWrite(PIN_SIGNAL_BUZZERS, LOW);
  delay(1000);

  DEBUGln("Battery Test");
  processBattery(); //Если заряд батарейки недостаточен, то моргаем 7 раз (если можем!) и выключаемся
  // два раза показываем заряд батарейки:
  showBatteryVoltage();
  delay(2000);   //
  showBatteryVoltage();
  delay(2000);   //

  //МЕНЯТЬ рабочую частоту (синхронно на TX и RX!)
  //Желательно сильно не уходить от значения 434E6 ()
  //просто добавлять-убавлять десятые, например: 433.9E6, 433.8E6, или 434.1E6, 434.2E6
  workFrequency = 434E6;

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
  delay(300);

  if (!LoRa.begin(workFrequency)) {             // initialize radio
    DEBUGln(F("LoRa init failed. Check your connections."));
    while (true) {
      flashStatusLed(6);    // if LoRa failed, blink and do nothing
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

void loop() { //  ===!!!===!!!===!!!===LOOP===!!!===!!!===!!!===!!!===!!!===

  if (rcvCmd)
    processCommand();
  else
    processTimeOut();  //see if we haven't received from TX for long enough
  processCutoff();
  EVERY_MS(BATTERY_PERIOD) {
    processBattery();
  }

}//loop()         ===!!!===!!!===!!!===END LOOP===!!!===!!!===!!!===!!!===!!!===

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
      sendMessage(rcvAddress, msgNumber, CMD_SIGNAL_OK, signalStatus);
      break;
    case CMD_PING:
      updateStatusLed(true);
      DEBUGln(F("=== CMD_PING ==="));
      signalStatus = rcvData;
      processSignal();
      //      unsigned long flashStatus = millis();
      sendMessage(rcvAddress, msgNumber, CMD_PONG, signalStatus);
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
  analogWrite(PIN_SIGNAL_LED, signalStatus * pwmledBrightness);
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

long frequencyByChannel(byte numChannel) {
  DEBUGln(F("frequencyByChannel(byte numChannel)"));
  if (numChannel > NUM_LORA_CHANNELS) {
    DEBUGln("Invalid Channel: " + String(numChannel));
    return CALL_FQ;
  }
  return (minFQ + numChannel * CHANNEL_WIDTH);
}
// done frequencyByChannel(byte numChannel)

void setLoRaParams() {
  DEBUGln(F("setLoRaParams()"));
  //Set LoRa for Longest Range:
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);    //max
  LoRa.setSignalBandwidth(125E3);                 //..31.25E3, 41.7E3, 62.5E3, (125E3), and 250E3.
  LoRa.setSpreadingFactor(8);                    //default = 7
  LoRa.setPreambleLength(6);                    //min = 6, default = 8
  LoRa.enableCrc();                             //
  //  LoRa.setCodingRate4(5);

}// DONE void setLoRaParams()

void sendMessage(byte msgAddr, byte msgNumber, byte msgCmd, byte msgData) {
  DEBUGln(F(">>>Sending Message"));

  while (!LoRa.beginPacket()) {
    DEBUGln(F("\tWaiting to begin REPLY"));
  }                   // start packet
  LoRa.write(msgAddr);              // reply address
  LoRa.write(msgNumber);              // reply Msg Number
  LoRa.write(msgCmd);                  // reply command
  LoRa.write(msgData);                 // reply Data
  while (!LoRa.endPacket()) {            // finish packet and send it
    DEBUGln(F("\tWaiting to finish REPLY"));
  }
  lastSendTime = millis();            // timestamp the message
  pingTimeOutLastTime = lastSendTime; //variable for Ping Timeout
  LoRa.receive();                     // go back into receive mode
  DEBUGln(("\tMessage sent: ") + String(msgAddr)\
          + " " + String(msgNumber) + " " + String(msgCmd) + " " + String(msgData));
}// void sendMessage(byte messageByte)

//void onTxDone() {
//  DEBUGln("onTxDone()");
//  lastSendTime = millis();            // timestamp the message
//  LoRa.receive();                     // go back into receive mode
//}

void onReceive(int packetSize) {
  DEBUGln(F("\n<<<Package Received!"));
  if (packetSize != 4) { //нас интересуют только пакеты в 4 байта
    DEBUGln("\tWrong Packet Size: " + String(packetSize));
    return;          // not our packet, return
  }

  // read packet header bytes:
  rcvAddress = LoRa.read();          // received address

  if (rcvAddress != workAddress) {
    DEBUGln("\tWrong Address: " + String(rcvAddress) + " - " +  String(workAddress));
    return;
  }

  rcvCount = LoRa.read();            // received Number
  rcvCmd = LoRa.read();              // received command
  rcvData = LoRa.read();                  // received data

#ifdef DEBUG_ENABLE  //только имеет смысл для дебагинга
  lastRSSI =  LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
  lastTurnaround = millis() - lastSendTime;
  lastFrequencyError = LoRa.packetFrequencyError();
#endif


  msgNumber = rcvCount;

  DEBUGln("\tRSSI:\t" + String(lastRSSI));
  DEBUGln("\tSnr:\t" + String(lastSNR));
  DEBUGln("\tTurnaround:\t" + String(lastTurnaround));
  DEBUGln("\tFrequency Error:\t" + String(lastFrequencyError));
  DEBUGln("\tReceived Message:\t"  + String(rcvAddress) + " " + String(rcvCount)\
          +" " + String( rcvCmd) + " " + String( rcvData));

}//void onReceive(int packetSize)

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

void flashBatteryLEDOnce() {
  digitalWrite(PIN_BATTERY_LED, 1);
  delay(250);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(250);
}

float batteryVoltage() {
  float measuredvbat = analogRead(PIN_BATTERY);
  measuredvbat = analogRead(PIN_BATTERY);
  measuredvbat *= batteryVoltageMultiplier;    // multiply according to the used board divider
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  DEBUG(F("Battery Voltage: "));
  DEBUGln(measuredvbat);
  return measuredvbat;
}////batteryVoltage()

void stopWorking() {
  flashLedBattery(7);
  delay(5000);
  flashLedBattery(7);
  while (1) {
    power.setSleepMode(POWERDOWN_SLEEP); // Крепко засыпаем
    delay(100); // даем время на отправку
    power.sleep(SLEEP_FOREVER); // спим до перезагрузки

  }
}

void flashLedBattery(byte times) { //flash "times" times
  DEBUGln(F("flashLedBattery()"));
  bool flash = true;
  for (int i = 0; i < times * 2; i++) {
    digitalWrite(PIN_BATTERY_LED, flash);
    flash = !flash;
    delay(200);
  }
}
