#include <Arduino.h>
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%z_Description.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%DavayLoRa_BSF_TX1.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//   Для возможных изменений в коде пометить здесь слово МЕНЯТЬ, и искать его (Ctrl-F)

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

// --- ПРОТОТИПЫ ФУНКЦИЙ (Оглавление для компилятора) ---
void processButton();
void processPing();
void updateStatusLed(bool ledStatus);
void updateBIGLed(bool ledStatus);
void flashStatusLed(byte times);
void sendMessage(byte msgCmd, byte sndData);
bool commSession( byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes );
void setLoRaParams();
void onReceive(int packetSize);
bool testBattery();
bool batteryVoltageOK(byte tries);
float batteryVoltage();
void showBatteryVoltage();
void showNoBattery();
void flashBatteryLEDOnce();
void flashLedBattery(byte times);
void processBattery();
void stopWorking();
// ------------------------------------------------------

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
//%%%%%%%%%%loraParams.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

//%%%%%%%%%%onReceive.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void onReceive(int packetSize) {
  DEBUGln("<<<PackageReceived");

  rcvAddress = LoRa.read();          // replied address
  if ((rcvAddress != workAddress) || (packetSize != 3)) {
#ifdef DEBUG_ENABLE
    DEBUGln(F("Invalid package! "));
    DEBUG(F("Received address: "));
    DEBUGln(rcvAddress);
    DEBUG(F("Expected address: "));
    DEBUGln(workAddress);
    DEBUG(F("Package length: "));
    DEBUGln(packetSize);
#endif
    delay(30); //пропускаем немного времени - пока работает чужое радио
    return;
  }

  rcvCmd = LoRa.read();    // replied command
  if (rcvCmd != cmdExpected) {
    DEBUGln("\tInvalid Reply: " + String(rcvCmd) + F(", Expected: ") + String(cmdExpected));
    delay(30); //пропускаем немного времени - пока работает чужое радио
    return;
  }
  rcvData = LoRa.read();

  lastTurnaround = millis() - lastSendTime;

  lastFrequencyError = LoRa.packetFrequencyError();
  lastRSSI =  LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
  DEBUGln("\tReceived Message: "  + String(rcvAddress) +" " + String( rcvCmd) + " " + String( rcvData));
  DEBUGln(("\tRSSI: ") + String(lastRSSI));
  DEBUGln(("\tSnr: ") + String(lastSNR));
  DEBUGln(("\tTurnaround: ") + String(lastTurnaround));
  DEBUGln(("\tFrequency Error: ") + String(lastFrequencyError));
  DEBUGln(("\tRX Working Frequency:\t") + String(workFrequency - lastFrequencyError));
  DEBUGln(F("=== onReceive done ==="));

  wasReceived = true;

}//void onReceive(int packetSize)

//%%%%%%%%%%batteryStuff.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
}////bool batteryVoltageOK(byte tries)

float batteryVoltage() {
  DEBUG(F("Battery Voltage: "));
  float measuredvbat = analogRead(batteryPIN);
  measuredvbat *= batteryVoltageMultiplier;    // multiply according to the used board divider
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  DEBUGln(measuredvbat);
  return measuredvbat;
}////batteryVoltage()

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

void processBattery() {

  if (batteryVoltage() < BATTERY_MIN_VOLTAGE) {
    stopWorking();
  }

}
//%%%%%%%%%%stopWorking.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void stopWorking() {
  flashLedBattery(7);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(5000);
  flashLedBattery(7);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(5000);
  while (1) {
    power.setSleepMode(POWERDOWN_SLEEP); // Крепко засыпаем
    delay(100); // даем время на отправку
    power.sleep(SLEEP_FOREVER); // спим до перезагрузки

  }
}
//%%%%%%%%%%%%%1_Setup.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void setup() {//=======================SETUP===============================
  delay(2000);   // Give time to the ATMega32u4 port to wake up and be recognized by the OS.
  power.hardwareEnable(PWR_ALL); //На всякий случай включим все системы микропроцессора

  // initialize serial for debug output if needed
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
#endif

  DEBUGln("================================");
  DEBUGln("=========== START TX ===========");
  DEBUGln("DavayLoRa TX setup()");

  //INIT PINS for button and status LEDs:
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_FB_LED, OUTPUT);
  pinMode(PIN_BIG_LED, OUTPUT);
  pinMode(PIN_BATTERY_LED, OUTPUT);
  digitalWrite(PIN_FB_LED, 0); //switch off FB on status led
  digitalWrite(PIN_BIG_LED, 0); //switch off big monitoring led
  digitalWrite(PIN_BATTERY_LED, 0); //switch off battery indicating led (built-in normally)
  //  digitalWrite(PD5, HIGH);
  delay(300);

  //Приветственный сигнал 1 сек
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

  // override the LoRa library default CS, reset, and IRQ pins with the board values
  LoRa.setPins(csPin, resetPin, irqPin);  // set CS, reset, IRQ pin
  delay(300);

  workFrequency = workingFrequency[WORK_ADDRESS % MAX_ADDRESS];
  DEBUG("LoRa begin on ");
  DEBUGln(workFrequency);
  if (!LoRa.begin(workFrequency)) {             // initialize radio at workFrequency
    DEBUGln("LoRa init failed. Check your connections.");
    while (true) {
      flashStatusLed(6);    // if failed, do nothing
      delay(4000);
    }
  }

  setLoRaParams();          //Tweak parameters for best communication

  LoRa.onReceive(onReceive);
  delay(100);

//  LoRa.onTxDone(onTxDone);

  LoRa.idle();              //Until we decide how to continue
  delay(100);

  DEBUGln("DavayLoRa TX setup complete, waiting for 1-st buttonpress");

}//setup      //======================= /SETUP ===============================
//%%%%%%%%%%%%%2_Loop.ino%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void loop() { //  ===!!!===!!!===!!!===!!!= LOOP =!!!===!!!===!!!===!!!===!!!===

  if ((millis() - lastButtonTime) > DEBOUNCE_TIME)
    processButton();
  processPing();
  EVERY_MS(BATTERY_PERIOD) {
    processBattery();
  }

}//loop()         ===!!!===!!!===!!!===!!!= LOOP =!!!===!!!===!!!===!!!===!!!===
