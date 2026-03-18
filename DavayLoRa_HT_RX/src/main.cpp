#include <Arduino.h>
#include <esp_sleep.h>
#include <SPI.h>              
#include <RadioLib.h> 

//============================МЕНЯТЬ===================================
#define WORK_ADDRESS 4 

#define BIG_LED_BRIGHTNESS 35
#define BUZZER_BIPPER_VOLUME 255

#define CUTOFF_TIME 2000  //ms 
#define PING_TIMEOUT 5000  //ms 

#define MEASURE_BATTERY true  

#define BATTERY_MIN_VOLTAGE 3.5   
#define BATTERY_VOLTAGE_1 3.6 
#define BATTERY_VOLTAGE_2 3.7 
#define BATTERY_VOLTAGE_3 3.8 
#define BATTERY_VOLTAGE_4 3.9 
#define BATTERY_VOLTAGE_5 4.0 

#define BATTERY_PERIOD 300000 //(= 5 минут) 

//======================= ПИНЫ HELTEC WIRELESS STICK LITE V3 ========================

// --- Исполнительные пины (Управление MOSFET) ---
#define PIN_SIGNAL_LED      41  //14 Пин для затвора транзистора главного ЛЕДа
#define PIN_SIGNAL_BUZZERS  42  //15 Пин для затвора транзистора Баззера и Вибро

// --- Индикация (Встроенный LED платы Heltec V3) ---
#define PIN_STATUS_LED      35  //8 Встроенный белый светодиод на Heltec V3
#define PIN_BATTERY_LED     35  //8 Тот же пин для индикации батареи

// --- Пины измерения батареи (Спецификация WSL V3) ---
#define PIN_BATTERY_INTERNAL 1 // Пин АЦП для замера батареи (ADC1_CH0)
#define PIN_VEXT 36            // Пин управления питанием встроенного делителя
#define PIN_ADC_CTRL 37        // Пин управления делителем батареи
#define HELTEC_BATTERY_MULTIPLIER 4.9 // Аппаратный делитель (390k + 100k) / 100k = 4.9

// --- Пины SPI и радиомодуля SX1262 (Внутренняя разводка WSL V3) ---
const int sckPin = 9;          
const int misoPin = 11;        
const int mosiPin = 10;        
const int csPin = 8;          // LoRa Chip Select (NSS)
const int resetPin = 12;      // LoRa Reset
const int irqPin = 14;        // LoRa DIO1 (Аппаратное прерывание)
const int busyPin = 13;       // LoRa BUSY

// Создаем экземпляр модуля SX1262
SX1262 radio = new Module(csPin, irqPin, resetPin, busyPin);

// ===================================================================

#define WORK_FREQUENCY 434E6  

bool measurebattery = MEASURE_BATTERY; 

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

#define CMD_SIGNAL         208 
#define CMD_SIGNAL_OK      209 
#define CMD_PING        212 
#define CMD_PING_OK     213 

byte workAddress = WORK_ADDRESS;  
byte rcvAddress = 0;          
byte rcvCmd = 0;              
byte rcvData = 0;                  
byte sndCmd = CMD_PING_OK;              
byte sndData;                         
bool signalStatus;                     
byte workChannel;                      
unsigned long workFrequency = WORK_FREQUENCY; 

unsigned long lastSendTime = 0;                
int lastRSSI;
float lastSNR;
unsigned long lastTurnaround;         
float lastFrequencyError; // У RadioLib ошибка частоты возвращается во float

unsigned long pingTimeOutLastTime;
unsigned long cutoffTimer = 0;

volatile bool receivedFlag = false;

// --- ПРОТОТИПЫ ФУНКЦИЙ ---
void processTimeOut();
void processCommand();
void processSignal();
void processCutoff();
void updateStatusLed(bool ledStatus);
void flashStatusLEDOnce();
void flashStatusLed(byte times);
void sendMessage(byte msgAddr, byte msgCmd, byte msgData);
void setLoRaParams();
void checkReceive();
void onReceive(byte* payload, int packetSize);
bool testBattery();
bool batteryVoltageOK(byte tries);
float batteryVoltage();
void processBattery();
void showBatteryVoltage();
void showNoBattery();
void flashBatteryLEDOnce();
void flashLedBattery(byte times);
void stopWorking();

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}

void processTimeOut() {
  if ((millis() - pingTimeOutLastTime) > PING_TIMEOUT) { 
    DEBUGln(F("ZZZZZZZ"));
    signalStatus = false;
    pingTimeOutLastTime = millis();
    flashStatusLed(2);
  }
}

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
} 

void processSignal() {
  cutoffTimer = millis();
  analogWrite(PIN_SIGNAL_LED, signalStatus * BIG_LED_BRIGHTNESS);
  analogWrite(PIN_SIGNAL_BUZZERS, signalStatus * BUZZER_BIPPER_VOLUME);
  digitalWrite(PIN_STATUS_LED, signalStatus); 
}

void processCutoff() {
  if (signalStatus && (millis() - cutoffTimer > CUTOFF_TIME)) {
    signalStatus = 0;
    analogWrite(PIN_SIGNAL_LED, 0);
    analogWrite(PIN_SIGNAL_BUZZERS, 0);
    digitalWrite(PIN_STATUS_LED, 0);
  }
}

void updateStatusLed(bool ledStatus) { 
  digitalWrite(PIN_STATUS_LED, ledStatus);
}

void flashStatusLEDOnce() {
  digitalWrite(PIN_STATUS_LED, 1);
  delay(250);
  digitalWrite(PIN_STATUS_LED, 0);
  delay(250);
}

void flashStatusLed(byte times) { 
  for (int i = 0; i < times; i++) {
    flashStatusLEDOnce();
  }
  delay(200);
}

void sendMessage(byte msgAddr, byte msgCmd, byte msgData) {
  DEBUGln(F(">>>Sending Message"));
  
  byte payload[3] = {msgAddr, msgCmd, msgData};
  
  int state = radio.transmit(payload, 3);
  
  if (state == RADIOLIB_ERR_NONE) {
    DEBUGln(("\tMessage sent: ") + String(msgAddr) + " " + String(msgCmd) + " " + String(msgData));
  } else {
    DEBUGln(("\tTransmit failed, code: ") + String(state));
  }
  
  lastSendTime = millis();            
  pingTimeOutLastTime = lastSendTime; 
  radio.startReceive(); 
}

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

unsigned long workingFrequency[MAX_ADDRESS] =
{
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
  radio.setPreambleLength(8);     // ВЕРНУЛИ СТАНДАРТНУЮ ПРЕАМБУЛУ (8)
  radio.setSyncWord(0x1424);      // ЖЕСТКО ЗАДАЛИ СТАНДАРТНОЕ СИНХРОСЛОВО Private
}

void onReceive(byte* payload, int packetSize) {
  DEBUGln(F("\n<<<Package Received"));

  rcvAddress = payload[0];          
  if ((rcvAddress != workAddress) || (packetSize != 3)) {
    DEBUGln(F("Invalid package! "));
    DEBUG(F("Received address: "));
    DEBUGln(rcvAddress);
    DEBUG(F("Expected address: "));
    DEBUGln(workAddress);
    DEBUG(F("Package length: "));
    DEBUGln(packetSize);
    return;
  }

  rcvCmd = payload[1];              
  rcvData = payload[2];                  

  lastFrequencyError = radio.getFrequencyError();

#ifdef DEBUG_ENABLE
  lastRSSI = radio.getRSSI();
  lastSNR = radio.getSNR();
  DEBUGln("\tReceived Message:\t"  + String(rcvAddress) + " " + String(rcvCmd) + " " + String(rcvData));
  DEBUGln("\tRSSI:\t" + String(lastRSSI));
  DEBUGln("\tSnr:\t" + String(lastSNR));
  DEBUGln("\tFrequency Error:\t" + String(lastFrequencyError));
  DEBUGln(("\tWorking Frequency OLD:\t") + String(workFrequency));
#endif

  // Подстройка частоты (RadioLib ожидает ввод в мегагерцах)
  workFrequency = workFrequency - (lastFrequencyError / 2.0);
  //radio.setFrequency(workFrequency / 1000000.0); //Закомментировано временно TODO

  DEBUGln(("\tWorking Frequency NEW:\t") + String(workFrequency));
  DEBUGln(F("=== onReceive done ==="));
}

bool testBattery()   {
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
    
    if ((currentVBat > 4.5) || (currentVBat < 2.5)) {
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

  digitalWrite(PIN_ADC_CTRL, LOW); // Включаем именно делитель батареи!
  delay(10); 

  float measuredvbat = analogRead(PIN_BATTERY_INTERNAL);

  digitalWrite(PIN_ADC_CTRL, HIGH); // Отключаем делитель

  measuredvbat *= 3.3;  
  measuredvbat /= 4095.0; 
  measuredvbat *= HELTEC_BATTERY_MULTIPLIER;

  DEBUGln(measuredvbat);
  return measuredvbat;
}

void processBattery() {
  if (batteryVoltage() < BATTERY_MIN_VOLTAGE) {
    stopWorking();
  }
}

void showBatteryVoltage() {
  float voltage = batteryVoltage();
  if (voltage > BATTERY_VOLTAGE_1)   flashBatteryLEDOnce(); 
  if (voltage > BATTERY_VOLTAGE_2)   flashBatteryLEDOnce(); 
  if (voltage > BATTERY_VOLTAGE_3)   flashBatteryLEDOnce(); 
  if (voltage > BATTERY_VOLTAGE_4)   flashBatteryLEDOnce(); 
  if (voltage > BATTERY_VOLTAGE_5)   flashBatteryLEDOnce(); 
}

void showNoBattery() { 
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

void flashLedBattery(byte times) { 
  DEBUGln(F("flashLedBattery()"));
  for (int i = 0; i < times; i++) {
    flashBatteryLEDOnce();
  }
  delay(200);
}

void stopWorking() {
  flashLedBattery(7);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(3000);
  flashLedBattery(7);
  digitalWrite(PIN_BATTERY_LED, 0);
  
  esp_deep_sleep_start();
}

void setup() {
  delay(2000);  

#ifdef DEBUG_ENABLE
  Serial.begin(115200);
  while (!Serial);
#endif

  DEBUGln(F("================================"));
  DEBUGln(F("=========== START RX ==========="));
  DEBUGln(F("DavayLoRa RX setup()"));

  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, HIGH);
  pinMode(PIN_ADC_CTRL, OUTPUT);
  digitalWrite(PIN_ADC_CTRL, HIGH); // Выключаем делитель по умолчанию 

  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_SIGNAL_BUZZERS, OUTPUT);
  pinMode(PIN_SIGNAL_LED, OUTPUT);
  analogWrite(PIN_SIGNAL_LED, 0); 
  analogWrite(PIN_SIGNAL_BUZZERS, 0);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(300);

  updateStatusLed(true);
  analogWrite(PIN_SIGNAL_LED, BIG_LED_BRIGHTNESS);
  analogWrite(PIN_SIGNAL_BUZZERS, BUZZER_BIPPER_VOLUME);
  delay(1000);
  updateStatusLed(false);
  analogWrite(PIN_SIGNAL_LED, 0);
  analogWrite(PIN_SIGNAL_BUZZERS, LOW);  
  delay(1000);

  DEBUGln(F("Battery Test"));
  measurebattery = testBattery();     
  if (measurebattery) {
  DEBUGln(F("-Measuring"));
    processBattery(); 
    delay(500);   
    showBatteryVoltage();
    delay(2000);   
    showBatteryVoltage();
    delay(500);   
  }
  else {
    DEBUGln(F("-Cancelled"));
    showNoBattery();
    delay(500);   
  }

  SPI.begin(sckPin, misoPin, mosiPin, csPin);
  
  workFrequency = workingFrequency[WORK_ADDRESS % MAX_ADDRESS];
  DEBUG("LoRa begin on ");
  DEBUGln(workFrequency);
  
  int state = radio.begin(workFrequency / 1000000.0);
  if (state != RADIOLIB_ERR_NONE) {             
    DEBUGln("LoRa init failed. Code: " + String(state));
    while (true) {
      flashStatusLed(6);    
      delay(4000);
    }
  }

  setLoRaParams();

  radio.setDio1Action(setFlag);
  radio.startReceive(); 

  pingTimeOutLastTime = millis();

  DEBUGln(F("DavayLoRa RX setup complete"));
}

void loop() { 
  checkReceive();

  if (rcvCmd)
    processCommand();
  else
    processTimeOut();  

  processCutoff();

  EVERY_MS(BATTERY_PERIOD) {
    if (measurebattery) {
      processBattery();
    }
  }
}