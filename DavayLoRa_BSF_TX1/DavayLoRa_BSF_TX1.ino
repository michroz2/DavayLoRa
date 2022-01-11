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
  Для задуманных возможных изменений в коде пометить здесь слово МЕНЯТЬ, и искать его (Ctrl-F)
  СОЕДИНЕНИЯ (см. также схему Fritzing и картинку):
    Кнопка нормально разомкнутая (NO), со встроенным светодиодом (+/-), использумым для индикации
    обратной связи, батарейки и ошибок.
      NO (любой) -> GND
      NO (другой) -> 6 микропроцессора
      светодиод (+) -> 5 микропроцессора
      светодиод (-) -> GND (или соединить с NO, который GND)
    Большой светодиод (используется для индикации вызова при нажатии на кнопку)
      плюс -> BAT микропроцессора (или + батареи)
      минус -> сток (drain) полевого тр-ра (центральный вывод)
    MOSFET 60NO3
      управляющий (gate) полевого тр-ра (левый вывод) -> 11 микропроцессора
      исток (source) полевого тр-ра (правый вывод) -> GND
    Переключатель выключения
      центр (или край) -> GND
      край (или центр) -> EN микропроцессора
      (при замкнутом переключателе прибор вЫключен, заряжать батарейку при этом можно;
      при разомкнутом - прибор включен)
    Батарею LiPo 1S подключить или припаять к своему JST разъёму
    USB порт можно использовать в любое время для зарядки батареи или заливки прошивки

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

#define CALL_FQ 434E6                       //стартовая рабочая частота.

//#define MIN_FQ 433.05E6                     //минимальная частота диапазона.
//#define MAX_FQ 434.79E6                     //максимальная рабочая частота.
//#define CHANNEL_WIDTH 1E5                 // поставить 100 или 200 KHz
//#define NUM_LORA_CHANNELS (MAX_FQ - MIN_FQ)/CHANNEL_WIDTH   //количество каналов столько кГц
//long minFQ = round(MIN_FQ / 1.0E5) * 1.0E5;
////TODO: сделать автоматический выбор рабочего канала:
//#define WORKING_CHANNEL 5                 //fixed. supposed controlled by jumpers or by scan
//#define BROADCAST_ADDRESS 0xFF

#define DEFAULT_TIMEOUT 300    //начальный таймаут для отзыва приёмника (мс)
#define WORK_COMM_ATTEMPTS 3
#define PING_TIMEOUT 3000  //ms
#define PING_FLASH 100  //ms
#define BATTERY_MIN 3.3   //Volt min.
#define BATTERY_PERIOD  300000 //Каждые столько миллисекунд измеряется напряжение батареи 
#define BIG_TIMEOUT    3600000 //Через час «холостой» работы передатчик прекращает пинг

#define PIN_BUTTON  6  // Номер пина Arduino, к которому подключен вывод кнопки (притянуто к 5в)
#define PIN_FB_LED  5  // Номер пина Arduino, к которому подключен вывод LED обратной связи
#define PIN_BATTERY 9  // Номер пина для измерения батарейки (Adafruit = 9)
#define PIN_PWM_LED 11 // Номер пина для ШИМ большого ЛЕДа
#define PIN_BATTERY_LED LED_BUILTIN  // Номер ЛЕДа для индикации заряда батарейки - 5 = выбран тот же ЛЕД, что на кнопке

//МЕНЯТЬ Коэффициент делителя для измерения батарейки.
//Для Adafruit поставить 2, для BSFrance поставить 1.27 (разница исполнения модулей)
float batteryVoltageMultiplier = 1.27;

//Дефолтовые для SPI библиотеки: 10, 9, 2, но для данных модулей используются другие:
const int csPin = 8;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 7;         // change for your board; must be a hardware interrupt pin

//Практически, описание протокола (тут ещё сохранились команды для Паринга, но они не мешают):
#define CMD_PAIRING     200 //TX передаёт команду/команду с бродкастным адресом
#define CMD_PAIRING_OK  201 //RX отвечает /свой зашитый адрес с бродкастным адресом
#define CMD_ADDR        202 //TX передаёт полученный адрес/адрес с адресом :)
#define CMD_ADDR_OK     203 //RX отвечает /адрес с адресом
#define CMD_CHANNEL     204 //TX передаёт свой будущий /канал
#define CMD_CHANNEL_OK  205 //RX отвечает, что переключается/канал
#define CMD_START       206 //TX передаёт на канале старт/старт
#define CMD_START_OK    207 //RX отвечает на канале ok/ok - паринг закончился
#define CMD_SIGNAL         208 //TX передаёт сигнал на изменение состояния Сигнала
#define CMD_SIGNAL_OK      209 //RX подтверждает
#define CMD_PING        212 //TX пингует периодически с /состоянием леда
#define CMD_PONG        213 //RX отвечает с состоянием леда
#define CMD_PING_OK     213 //то же что предыдущее

//МЕНЯТЬ синхронно для TX и RX в диапазоне 0-254
#define WORK_ADDRESS      78

byte workAddress = WORK_ADDRESS;  // address of connection
byte msgCount = 0;                    // = number of outgoing message
byte sndCmd = CMD_PING;              // outgoing command Default = PING
byte sndData;                         // additional data byte sent
bool wasReceived = 0;                     //indication of reply message received
byte cmdExpected = CMD_PONG;          // expected command (Default = PONG)
byte rcvAddress = 0;          // received address
byte rcvCount = 0;            // received Number
byte rcvCmd = 0;                      // received command
byte rcvData;                         // additional data byte received
byte workChannel;                      //channel to send to RX while paring and work on it
unsigned long workFrequency = CALL_FQ; //working Frequency

long lastSendTime = 0;                // last send time
int lastRSSI;
float lastSNR;
unsigned long lastTurnaround = 100;         // round-trip time between tx and rx
long lastFrequencyError;
unsigned long expectedTimeout = DEFAULT_TIMEOUT;
unsigned long lastButtonTime; //the last time button was active

bool currButtonState = 0;               //Current state of Button, initially OFF
bool prevButtonState = 0;               //Previous state of Button, initially OFF
bool firstPressButton = 0;              //Включается работа по первому нажатию

unsigned long pingTimer;
unsigned long pingFlashTimer;
bool pingFlash;

int fbledBrightness = 255;           // 0 - 255 - Яркость леда в кнопке
int pwmledBrightness = 15;           // 0 - 30 - Яркость большого леда (больше 30 - слишком ярко! и много потребляет )


void setup() {//=======================SETUP===============================
  delay(2000);   // Give time to the ATMega32u4 port to wake up and be recognized by the OS.
  power.hardwareEnable(PWR_ALL);

  // initialize serial
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
#endif

  DEBUGln("================================");
  DEBUGln("=========== START TX ===========");
  DEBUGln("DavayLoRa TX setup()");

  // override the library default CS, reset, and IRQ pins
  LoRa.setPins(csPin, resetPin, irqPin);  // set CS, reset, IRQ pin
  delay(300);

  //INIT PINS for button and status LEDs:
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_FB_LED, OUTPUT);
  pinMode(PIN_PWM_LED, OUTPUT);
  digitalWrite(PIN_FB_LED, 0); //switch off FB on status led
  digitalWrite(PIN_PWM_LED, 0); //switch off big monitoring led
  digitalWrite(PIN_BATTERY_LED, 0); //switch off battery indicating led (built-in normally)
  digitalWrite(PIN_BATTERY_LED, 0);
  //  digitalWrite(PD5, HIGH);
  delay(300);

  // два раза показываем заряд батарейки:
  showBatteryVoltage();
  delay(2000);   //
  showBatteryVoltage();
  delay(2000);   //

  //МЕНЯТЬ рабочую частоту (синхронно на TX и RX!) в диапазоне 433.1E6 - 434.8E6
  //Желательно сильно не уходить от значения 434E6 ()
  //просто добавлять-убавлять десятые, например: 433.9E6, 433.8E6, или 434.1E6, 434.2E6
  workFrequency = 434E6;

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

  DEBUGln("DavayLoRa TX setup complete");

}//setup      //======================= SETUP ===============================

void loop() { //  ===!!!===!!!===!!!===!!!= LOOP =!!!===!!!===!!!===!!!===!!!===

  processButton();
  processPing();
  processCommand();
  EVERY_MS(BATTERY_PERIOD) {
    processBattery();
  }

}//loop()         ===!!!===!!!===!!!===!!!= LOOP =!!!===!!!===!!!===!!!===!!!===

void   processButton() {
  currButtonState = !digitalRead(PIN_BUTTON); // Читаем состояние кнопки 1=нажата; 0=отпущена
  if (prevButtonState != currButtonState) {   //button pressed or released
    lastButtonTime = millis();
    firstPressButton = true;
    DEBUGln("\nprocessButton(): " + String(currButtonState));
    prevButtonState = currButtonState;
    if (commSession( CMD_SIGNAL, currButtonState, \
                     CMD_SIGNAL_OK, 3 * lastTurnaround, WORK_COMM_ATTEMPTS )) {
      updateFBLed(currButtonState);
      updatePWMLed(currButtonState);
    }
    else {
      updatePWMLed(false);
      //      updateFBLed(false);
      flashStatusLed(3);
    }
  }
}//void   processButton()

void processCommand() {
  if (wasReceived) {
    DEBUGln(F("--- processCommand() start ---"));

    //    DEBUGln(("\tReply number: ") + String(rcvCount));
    //    DEBUGln(("\tReply command: ") + String(rcvCmd));
    //    DEBUGln(("\tReply Data: ") + String(rcvData));

    DEBUGln(("\tRSSI: ") + String(lastRSSI));
    DEBUGln(("\tSnr: ") + String(lastSNR));
    DEBUGln(("\tTurnaround: ") + String(lastTurnaround));
    DEBUGln(("\tFrequency Error: ") + String(lastFrequencyError));
    DEBUGln(("\tWorking Frequency OLD:\t") + String(workFrequency));
    workFrequency = workFrequency - lastFrequencyError / 2;
    DEBUGln(("\tWorking Frequency NEW:\t") + String(workFrequency));
    LoRa.setFrequency(workFrequency);
    delay(30);
    wasReceived = false;
    DEBUGln("=== processCommand() done ===");
  }
}//void processCommand()


void   processPing() {
  if (!firstPressButton) return;
  if (pingFlash) {// ping reply was already received, FB led is on, processing FB led off only
    if ((millis() - pingFlashTimer) > PING_FLASH) { //end led flash
      DEBUGln(F("\tPing LED OFF"));
      pingFlash = false;
      updateFBLed(currButtonState);
    }
    return;  //??
  }
  if ((millis() - pingTimer) > PING_TIMEOUT) { // long time was no command - initiate ping
    DEBUGln(F("\nStart Ping"));
    if (commSession( CMD_PING, currButtonState, CMD_PONG, \
                     3 * lastTurnaround, WORK_COMM_ATTEMPTS )) {
      DEBUGln(F("\tPing LED ON"));
      updateFBLed(!currButtonState);
      pingFlash = true;
      pingFlashTimer = millis();
    }
    else {
      //      updateFBLed(false);
      flashStatusLed(2);
    }
  }
  if ((millis() - lastButtonTime) > BIG_TIMEOUT) { // 1 hour no button avtivity
    flashStatusLed(3);  //flash 3 times and
    firstPressButton = false; //return to the state like after switch-on - no pings
  }
}//void   processPing()

void processBattery() {

  if (batteryVoltage() < BATTERY_MIN) {
    stopWorking();
  }

}

void showBatteryVoltage() {
  float voltage = batteryVoltage();
  //  delay(1000);
  if (voltage > 3.5)   flashBatteryOnce(); //1 раз
  if (voltage > 3.6)   flashBatteryOnce(); //2 раз
  if (voltage > 3.7)   flashBatteryOnce(); //3 раз
  if (voltage > 3.8)   flashBatteryOnce(); //4 раз
  if (voltage > 4.0)   flashBatteryOnce(); //5 раз
}

void flashBatteryOnce() {
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
}

void stopWorking() {
  flashlLedBattery(7);
  delay(5000);
  flashlLedBattery(7);
  while (1) {
    power.setSleepMode(POWERDOWN_SLEEP); // Крепко засыпаем
    delay(100); // даем время на отправку
    power.sleep(SLEEP_FOREVER); // спим до перезагрузки 
  }
}

void flashlLedBattery(byte times) { //flash 3 times total 1.5 sec
  DEBUGln("flashLedBattery()");
  bool flash = false;
  for (int i = 0; i < times * 2; i++) {
    digitalWrite(PIN_BATTERY_LED, flash);
    flash = !flash;
    delay(200);
  }
}

void updateFBLed(bool ledStatus) { // turn ON or OFF the Status LED
  analogWrite(PIN_FB_LED, ledStatus * fbledBrightness);
  //  DEBUGln("updateFBLed(): " + String(ledStatus));
}

void updatePWMLed(bool ledStatus) { // turn ON or OFF the Status LED
  //  digitalWrite(PIN_FB_LED, ledStatus);
  analogWrite(PIN_PWM_LED, ledStatus * pwmledBrightness);
  //  DEBUGln("updatePWMLed(): " + String(ledStatus));
}

void flashStatusLed(byte times) { //flash n times
  DEBUGln("flashStatusLed()");
  for (int i = 0; i < times; i++) {
    updateFBLed(true);
    delay(100);
    updateFBLed(false);
    delay(200);
  }
}

void setLoRaParams() {
  DEBUGln("setLoRaParams()");
  //Trying setting LoRa for Longest Range possible:
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);    //max
  delay(50);
  LoRa.setSignalBandwidth(125E3);                 //..31.25E3, 41.7E3, 62.5E3, (125E3), and 250E3.
  delay(50);
  LoRa.setSpreadingFactor(8);                    //default = 7
  delay(50);
  LoRa.setPreambleLength(6);                    //min = 6, default = 8
  delay(50);
  LoRa.enableCrc();                             //
  delay(50);
  //  LoRa.setCodingRate4(5);

}// DONE void setLoRaParams()

void sendMessage(byte msgCmd, byte sndData) {
  DEBUGln(">>>sendMessage()");
  while (!LoRa.beginPacket()) {
    DEBUGln(("\tWaiting to begin TX"));
  }                   // start packet
  LoRa.write(workAddress);              // add address
  LoRa.write(++msgCount);              // add Msg Number
  LoRa.write(msgCmd);                  // add command
  LoRa.write(sndData);                 // add Data
  while (!LoRa.endPacket()) {            // finish packet and send it
    DEBUGln(("\tWaiting to finish TX"));
  }
  LoRa.receive();                     // go back into receive mode
  lastSendTime = millis();            // timestamp the message
  DEBUGln(("\tMessage sent: ") + String(workAddress)\
          + " " + String(msgCount) + " " + String(msgCmd) + " " + String(sndData));
}// void sendMessage(byte messageByte)

//void onTxDone() {
//  DEBUGln("onTxDone()");
//  lastSendTime = millis();            // timestamp the message
//  LoRa.receive();                     // go back into receive mode
//}

void onReceive(int packetSize) {
  DEBUGln("<<<onReceive()");
  if (packetSize != 4) {
    DEBUGln(("\tInvalid Packet Size: ") + String(packetSize));
    return;          // not our packet, return
  }
  // read packet header bytes:
  rcvAddress = LoRa.read();          // replied address
  if (rcvAddress != workAddress) {
    DEBUGln(("\tInvalid Address: ") + String(rcvAddress) + F(", Expected: ") + String(workAddress));
    return;
  }
  rcvCount = LoRa.read();    // replied number of Message
  if (rcvCount != msgCount) {
    DEBUGln(("\tInvalid Number: ") + String(rcvCount) + F(", Expected: ") + String(msgCount));
    return;
  }
  rcvCmd = LoRa.read();    // replied command
  if (rcvCmd != cmdExpected) {
    DEBUGln("\tInvalid Reply: " + String(rcvCmd) + F(", Expected: ") + String(cmdExpected));
    return;
  }
  rcvData = LoRa.read();

#ifdef DEBUG_ENABLE
  lastRSSI =  LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
#endif
  lastTurnaround = millis() - lastSendTime;
  lastFrequencyError = LoRa.packetFrequencyError();

  DEBUGln("\tReceived Message: "  + String(rcvAddress) + " " + String(rcvCount)\
          +" " + String( rcvCmd) + " " + String( rcvData));

  wasReceived = true;

}//void onReceive(int packetSize)

bool commSession( byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes ) {
  DEBUGln(F("commSession()"));
  wasReceived = false;
  int totTimes = doTimes;
  do {
    EVERY_MS(waitMilliseconds) {
      cmdExpected = expectedReply;
      if (totTimes != doTimes) {
        DEBUG(("\tComm tries left: "));
        DEBUGln(doTimes);
      }
      sendMessage(msgCmd, sndData);
      doTimes--;
    }
  } while ((doTimes > 0) && (!wasReceived));
  pingTimer = millis(); //refresh the ping timer after every communication
  return wasReceived;

}//commSession(...)
