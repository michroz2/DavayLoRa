//  Для возможных изменений в коде пометить здесь слово МЕНЯТЬ, и искать его (Ctrl-F)

/*
  Код ПЕРЕДАТЧИКА для проекта DavayLoRa
  TX отправляет сообщение на RX, когда нажата/отпущена кнопка, или пингует его по таймеру
  Ожидает ответ-подтверждение на каждое сообщение.
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
  - Кнопка нормально разомкнутая (NO), со встроенным светодиодом (+/-), использумым для индикации
    обратной связи, батарейки и ошибок.
      NO (любой) -> GND
      NO (другой) -> 6 микропроцессора
      светодиод (+) -> 5 микропроцессора
      светодиод (-) -> GND (или соединить с NO, который GND)
  - Большой светодиод (используется для индикации вызова при нажатии на кнопку)
      плюс -> BAT микропроцессора (или + батареи)
      минус -> сток (drain) полевого тр-ра (центральный вывод)
  - MOSFET 60NO3
      управляющий (gate) полевого тр-ра (левый вывод) -> 11 микропроцессора
      исток (source) полевого тр-ра (правый вывод) -> GND
  - Переключатель выключения
      центр (или край) -> GND
      край (или центр) -> EN микропроцессора
      (при замкнутом переключателе прибор вЫключен, заряжать батарейку при этом можно;
      при разомкнутом - прибор включен)
  - Батарею LiPo 1S подключить или припаять к своему JST разъёму

    USB порт можно использовать для зарядки батареи - в любое время
      и для заливки прошивки (при разомкнутом переключателе)

  ПРОБЛЕМА: У некоторых модулей BSFrance не работает встроенный измеритель напряжения
  (для определения этого можно воспользоватьс программой BSFTest.ino со включенным монитором).
  Чтобы исользовать такие модули, следует добавить в схему 2 одинаковых резистора
  по 10-100КОм следующим образом:
  - один вывод каждого резистора паять на (+) и (-) разъёма батареи
  - вторые выводы соединить вместе и присоединить к пину (на выбор) A0 - A5
  (какой удобнее, например A1).
  Также надо скорректировать следующие define-ы:
  #define PIN_BATTERY A1
  #define BATTERY_VOLTAGE_MULTIPLIER 2;
*/

#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <GyverPower.h>

// Дебагирование: раскомментить для использования 1 строчку:
//#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.print(x)
#define DEBUGln(x) Serial.println(x)
#else
#define DEBUG(x)
#define DEBUGln(x)
#endif

//==== MILLIS TIMER MACRO ====
// performs the {subsequent} code once and then, if needed, again after each x ms
#define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flg = millis() - tmr >= (x);\
  if (flg) tmr = millis();\
  if (flg)
//===========================

//МЕНЯТЬ рабочую частоту (синхронно на TX и RX!)
//Желательно сильно не уходить от значения 434E6 ()
//просто добавлять-убавлять десятые, например: 433.9E6, 433.8E6, или 434.1E6, 434.2E6
#define WORK_FREQUENCY 434E6

//МЕНЯТЬ - напряжение «отсечки» - по результатам использования
#define BATTERY_MIN_VOLTAGE 3.5   //Volt min.
#define BATTERY_VOLTAGE_1 3.5  // Мигание 1 раз
#define BATTERY_VOLTAGE_2 3.6
#define BATTERY_VOLTAGE_3 3.8
#define BATTERY_VOLTAGE_4 3.9
#define BATTERY_VOLTAGE_5 4.0

//МЕНЯТЬ - надо только, чтобы пинг приёмника был больше, чем у передатчика
#define PING_TIMEOUT 3000  //ms

#define DEFAULT_TURNAROUND 300    //начальный таймаут для отзыва приёмника (мс)
#define WORK_COMM_ATTEMPTS 4
#define PING_FLASH 100  //ms
#define BATTERY_PERIOD  300000 //(5 минут) Каждые столько миллисекунд измеряется напряжение батареи 

//МЕНЯТЬ - (если надо)
#define BIG_TIMEOUT    3600000 //Через час «холостой» работы передатчик прекращает пинг

#define PIN_BUTTON  6  // Номер пина Arduino, к которому подключен вывод кнопки (притянуто к 5в)
#define PIN_FB_LED  5  // Номер пина Arduino, к которому подключен вывод LED обратной связи
#define PIN_BIG_LED 11 // Номер пина для ШИМ большого ЛЕДа
#define PIN_BATTERY_LED LED_BUILTIN  // Номер ЛЕДа для индикации заряда батарейки - 5 = выбран тот же ЛЕД, что на кнопке
#define DEBOUNCE_TIME 100 //Period of time (ms) in which to ignore additional button events

//МЕНЯТЬ:
#define PIN_BATTERY A1  // Номер пина Адафрута для измерения батарейки
/*Коэффициент делителя для измерения батарейки.
  Для «нормального» BSFrance поставить 1.27;
  Для Adafruit поставить 2;
  Также поставить 2 для BSFrance «с резисторами»
*/
#define BATTERY_VOLTAGE_MULTIPLIER 2

//Дефолтовые для SPI библиотеки: 10, 9, 2, но для данных модулей используются другие:
const int csPin = 8;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 7;         // change for your board; must be a hardware interrupt pin

//Описание протокола (тут ещё сохранились команды для Паринга, но они не мешают):
#define CMD_SIGNAL         208 //TX передаёт сигнал на изменение состояния Сигнала
#define CMD_SIGNAL_OK      209 //RX подтверждает
#define CMD_PING        212 //TX пингует периодически с /состоянием леда
#define CMD_PING_OK     213 //то же что предыдущее

//МЕНЯТЬ синхронно для TX и RX в диапазоне 0-254
#define WORK_ADDRESS      1

byte workAddress = WORK_ADDRESS;  // address of connection
byte sndCmd = CMD_PING;              // outgoing command Default = PING
byte sndData;                         // additional data byte sent
bool wasReceived = 0;                     //indication of reply message received
byte cmdExpected = CMD_PING_OK;          // expected command (Default = PONG)
byte rcvAddress = 0;          // received address
byte rcvCmd = 0;                      // received command
byte rcvData;                         // additional data byte received
byte workChannel;                      //channel to send to RX while paring and work on it
unsigned long workFrequency = WORK_FREQUENCY; //working Frequency

long lastSendTime = 0;                // last send time
int lastRSSI;
float lastSNR;
unsigned long lastTurnaround = DEFAULT_TURNAROUND;         // round-trip time between tx and rx
long lastFrequencyError;
unsigned long lastButtonTime; //the last time button was active

bool currButtonState;               //Current state of Button, initially OFF
bool prevButtonState;               //Previous state of Button, initially OFF
bool buttonPressedFirstTime;              //Включается работа по первому нажатию
bool buttonChanged;
bool buttonSent;

unsigned long pingTimer;
unsigned long pingFlashTimer;
bool pingFlash;

int fbledBrightness = 255;           // 0 - 255 - Яркость леда в кнопке
int pwmledBrightness = 35;           // 0 - 30 - Яркость большого леда (больше 30 - слишком ярко! и много потребляет )


void processButton() {
  currButtonState = !digitalRead(PIN_BUTTON); // Читаем состояние кнопки 1=нажата; 0=отпущена
  if (prevButtonState != currButtonState) {   //button pressed or released
    lastButtonTime = millis();
    buttonPressedFirstTime = true;
    DEBUGln("\nprocessButton(): " + String(currButtonState));
    prevButtonState = currButtonState;
    // send Button state
    if (commSession( CMD_SIGNAL, currButtonState, CMD_SIGNAL_OK, \
                     6 * lastTurnaround, WORK_COMM_ATTEMPTS )) {
      updateStatusLed(currButtonState);
      updateBIGLed(currButtonState);
    }
    else {
      updateBIGLed(false);
      flashStatusLed(2);
    }
    wasReceived = false; //мы отработали сессию и больше ничего не ждём.
  }
}//////bool   processButton()


void   processPing() {
  if (!buttonPressedFirstTime) return;
  if (pingFlash) {// ping reply was already received, FB led is on, processing FB led off only
    if ((millis() - pingFlashTimer) > PING_FLASH) { //end led flash
      DEBUGln(F("\tPing LED OFF"));
      pingFlash = false;
      updateStatusLed(currButtonState);
    }
    return;  //??
  }
  if ((millis() - pingTimer) > PING_TIMEOUT) { // long time was no command - initiate ping
    DEBUGln(F("\nStart Ping"));
    if (commSession( CMD_PING, currButtonState, CMD_PING_OK,\
                    6 * lastTurnaround, WORK_COMM_ATTEMPTS )) {
      DEBUGln(F("\tPing LED ON"));
      updateStatusLed(!currButtonState);
      pingFlash = true;
      pingFlashTimer = millis();
    }
    else {
      //      updateStatusLed(false);
      flashStatusLed(2);
    }
  }
  if ((millis() - lastButtonTime) > BIG_TIMEOUT) { // 1 hour no button avtivity
    flashStatusLed(3);  //flash 3 times and
    buttonPressedFirstTime = false; //return to the state like after switch-on
    // - no more pings until the next buttonpress
  }
}//void   processPing()

void updateStatusLed(bool ledStatus) { // turn ON or OFF the Status LED
  analogWrite(PIN_FB_LED, ledStatus * fbledBrightness);
  //  DEBUGln("updateStatusLed(): " + String(ledStatus));
}

void updateBIGLed(bool ledStatus) { // turn ON or OFF the Status LED
  //  digitalWrite(PIN_FB_LED, ledStatus);
  analogWrite(PIN_BIG_LED, ledStatus * pwmledBrightness);
  //  DEBUGln("updateBIGLed(): " + String(ledStatus));
}

void flashStatusLed(byte times) { //flash n times
  DEBUGln("flashStatusLed()");
  for (int i = 0; i < times; i++) {
    updateStatusLed(true);
    delay(100);
    updateStatusLed(false);
    delay(200);
  }
}

void sendMessage(byte msgCmd, byte sndData) {
  DEBUGln(">>>sendMessage()");
  while (!LoRa.beginPacket()) {
    DEBUGln(("\tWaiting to begin TX"));
  }                   // start packet
  LoRa.write(workAddress);              // add address
  LoRa.write(msgCmd);                  // add command
  LoRa.write(sndData);                 // add Data
  while (!LoRa.endPacket()) {            // finish packet and send it
    DEBUGln(("\tWaiting to finish TX"));
  }
  //  LoRa.receive();                     // go back into receive mode
  lastSendTime = millis();            // timestamp the message
  DEBUGln(("\tMessage sent: ") + String(workAddress)\
          + " " + String(msgCmd) + " " + String(sndData));
  LoRa.receive();                     // go back into receive mode

}// void sendMessage(byte messageByte)

void onTxDone() {
  DEBUGln("onTxDone()");
  lastSendTime = millis();            // timestamp the message
  LoRa.receive();                     // go back into receive mode
}

bool commSession( byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes ) {
  DEBUGln(F("commSession()"));
  wasReceived = false;
  cmdExpected = expectedReply;
  do {
    EVERY_MS(waitMilliseconds) {
      sendMessage(msgCmd, sndData);
      doTimes--;
    }
  } while ((doTimes > 0) && (!wasReceived));
  pingTimer = millis(); //refresh the ping timer after every communication
  return wasReceived;

}//commSession(...)
