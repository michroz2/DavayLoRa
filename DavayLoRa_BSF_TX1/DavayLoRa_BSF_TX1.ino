//  Для возможных изменений в коде пометить здесь слово МЕНЯТЬ, и искать его (Ctrl-F)

//============================МЕНЯТЬ В ПЕРЕДАТЧИКЕ=================================
//МЕНЯТЬ синхронно для TX и RX в диапазоне 0-254
//! На самом деле пока сделаны разные частоты на 20 адресов, а потом частоты повторяются !
//Это означает, что приборы с адресами, отличающимися на 20 номеров, могут немного мешать друг другу при одновременной работе
//ВАЖНОЕ ЗНАЧЕНИЕ чтобы приборы работали парами!!!
//... и, в принципе, это всё что НАДО менять. Остальное можно не трогать.
#define WORK_ADDRESS 4 //По традиции первая прошивка: оставить «4» 
                       //Это позволяет при изготовлении проверить все приёмники и передатчики со всеми
                       //После всех проверок вернуться и настроить нужный адрес 

//МЕНЯТЬ яркость большого ЛЕДа, - аккуратно, не сильно больше 35
//FB_LED_BRIGHTNESS - между 0 (выключен) и 255 (максимум) - ближе к макс.
#define BIG_LED_BRIGHTNESS 35
#define FB_LED_BRIGHTNESS 255

//МЕНЯТЬ - надо только, чтобы пинг приёмника был больше, чем у передатчика
//например: 3сек-5сек
#define PING_TIMEOUT 3000  //ms

//МЕНЯТЬ - (если надо) - время стэнд-бай передатчика со включенным пингом
#define BIG_TIMEOUT    3600000 //Через час «холостой» работы 
//передатчик прекращает пинг

//МЕНЯТЬ: Далее идёт батарейный раздел, менять, в принципе, ничего не надо:
#define MEASURE_BATTERY true  // true или false = включить или выключить измерение батарейки
//Это будет определяться автоматически, но если хочется вообще выключить, то поставить false 

//МЕНЯТЬ - напряжение «отсечки» - по результатам использования
#define BATTERY_MIN_VOLTAGE 3.5   //Volt min.
#define BATTERY_VOLTAGE_1 3.5  // Мигание 1 раз
#define BATTERY_VOLTAGE_2 3.6
#define BATTERY_VOLTAGE_3 3.8
#define BATTERY_VOLTAGE_4 3.9
#define BATTERY_VOLTAGE_5 4.0

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

//=======================//конец чего можно нормально МЕНЯТЬ===================================
#define WORK_FREQUENCY 434E6

#define PIN_BATTERY_INTERNAL 9    // Номер внутреннего пина для измерения батарейки
#define RESISTOR_VOLTAGE_MULTIPLIER 2 //Если мы паяем делитель, то это всегда 2 одинаковых резистора

#define WORK_FREQUENCY 434E6  //Это значение ничего не определяет. Частота определяется автоматически по номеру адреса

float batteryVoltageMultiplier = 1.27; //Может измениться, если мы не обнаружим внутреннего делителя!
int batteryPIN = PIN_BATTERY_INTERNAL; //Для начала будем искать внутренний делитель                        //
bool measurebattery = MEASURE_BATTERY; //Для начала будем считать, что батарейка измеряется

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

#define MAX_ADDRESS 20 //пока сделано такое максимальное количество частот

#define DEFAULT_TURNAROUND 300    //начальный таймаут для отзыва приёмника (мс)
#define WORK_COMM_ATTEMPTS 3
#define PING_FLASH 100  //ms

#define PIN_BUTTON  6  // Номер пина Arduino, к которому подключен вывод кнопки (притянуто к 5в)
#define PIN_FB_LED  5  // Номер пина Arduino, к которому подключен вывод LED обратной связи
#define PIN_BIG_LED 11 // Номер пина для ШИМ большого ЛЕДа
#define PIN_BATTERY_LED 5 //LED_BUILTIN  // Номер ЛЕДа для индикации заряда батарейки - 5 = выбран тот же ЛЕД, что на кнопке
#define DEBOUNCE_TIME 100 //Period of time (ms) in which to ignore additional button events

//Дефолтовые для SPI библиотеки: 10, 9, 2, но для данных модулей используются другие:
const int csPin = 8;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 7;         // change for your board; must be a hardware interrupt pin

//Описание протокола:
#define CMD_SIGNAL         208 //TX передаёт сигнал на изменение состояния Сигнала
#define CMD_SIGNAL_OK      209 //RX подтверждает
#define CMD_PING        212 //TX пингует периодически с /состоянием леда
#define CMD_PING_OK     213 //то же что предыдущее

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

int fbledBrightness = FB_LED_BRIGHTNESS;           // 0 - 255 - Яркость леда в кнопке
int pwmledBrightness = BIG_LED_BRIGHTNESS;           // 0 - 30 - Яркость большого леда (больше 30 - слишком ярко! и много потребляет )

void processButton() {
  prevButtonState = currButtonState;
  currButtonState = !digitalRead(PIN_BUTTON); // Читаем состояние кнопки 1=нажата; 0=отпущена
  if (prevButtonState != currButtonState) {   //button pressed or released
    lastButtonTime = millis();
    pingTimer = millis(); //refresh the ping timer on button action
    buttonPressedFirstTime = true;
    DEBUGln("\nprocessButton(): " + String(currButtonState));
    prevButtonState = currButtonState;
    // send Button state
    if (currButtonState) {//only if the button turns ON
      if (commSession( CMD_SIGNAL, 1, CMD_SIGNAL_OK, \
                       2 * lastTurnaround, WORK_COMM_ATTEMPTS )) {
        updateStatusLed(true);
        updateBIGLed(true);
      }
      else {
        updateBIGLed(false);
        flashStatusLed(2);
      }
    }
    else { //if the button turns OFF
      sendMessage(CMD_SIGNAL, false); //short version of communication, w/o feedback
      updateStatusLed(false);
      updateBIGLed(false);
    }
    //    wasReceived = false; //мы отработали сессию и больше ничего не ждём.
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
  }
  else if ((millis() - pingTimer) > PING_TIMEOUT) { // long time was no command - initiate ping
    DEBUGln(F("\nStart Ping"));
    if (commSession( CMD_PING, currButtonState, CMD_PING_OK, \
                     5 * lastTurnaround, WORK_COMM_ATTEMPTS )) {
      DEBUGln(F("\tPing LED ON"));
      updateStatusLed(!currButtonState);
      pingFlash = true;
      pingFlashTimer = millis();
      pingTimer = millis();
    }
    else {
      flashStatusLed(2);
    }
  }
  if ((millis() - lastButtonTime) > BIG_TIMEOUT) { // 1 hour no button activity
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
  LoRa.receive();                     // go back into receive mode
  lastSendTime = millis();            // timestamp the message
  DEBUGln(("\tMessage sent: ") + String(workAddress)\
          + " " + String(msgCmd) + " " + String(sndData));

}// void sendMessage(byte messageByte)

//void onTxDone() {
//  DEBUGln("onTxDone()");
//  lastSendTime = millis();            // timestamp the message
//  LoRa.receive();                     // go back into receive mode
//}

bool commSession( byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes ) {
  DEBUGln(F("commSession()"));
  wasReceived = false;
  cmdExpected = expectedReply;
  pingTimer = millis(); //refresh the ping timer after every communication
  do {
    EVERY_MS(waitMilliseconds) {
      sendMessage(msgCmd, sndData);
      doTimes--;
    }
  } while ((doTimes > 0) && (!wasReceived));
  pingTimer = millis(); //refresh the ping timer after every communication
  return wasReceived;

}//commSession(...)

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
