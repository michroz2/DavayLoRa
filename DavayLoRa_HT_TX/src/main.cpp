#include <Arduino.h>
#include <esp_sleep.h>
#include <SPI.h>              
#include <RadioLib.h> 

//============================МЕНЯТЬ В ПЕРЕДАТЧИКЕ=================================
#define WORK_ADDRESS 4 

#define BIG_LED_BRIGHTNESS 35
#define FB_LED_BRIGHTNESS 255

#define PING_TIMEOUT 3000  //ms
#define BIG_TIMEOUT    3600000 

#define MEASURE_BATTERY true  

#define BATTERY_MIN_VOLTAGE 3.5   
#define BATTERY_VOLTAGE_1 3.5  
#define BATTERY_VOLTAGE_2 3.6
#define BATTERY_VOLTAGE_3 3.8
#define BATTERY_VOLTAGE_4 3.9
#define BATTERY_VOLTAGE_5 4.0

#define BATTERY_PERIOD 300000 

//======================= ПИНЫ HELTEC WIRELESS STICK LITE V3 ========================

// --- Пины интерфейса (ВАМ НУЖНО ПЕРЕПОДКЛЮЧИТЬ ПРОВОДА СЮДА) ---
#define PIN_BUTTON  41         // Безопасный GPIO на колодке J2
#define PIN_FB_LED  35         // Безопасный GPIO на колодке J2
#define PIN_BIG_LED 47         // Безопасный GPIO на колодке J3
#define PIN_BATTERY_LED 35     // Тот же, что и FB_LED

// --- Пины измерения батареи (Спецификация WSL V3) ---
#define PIN_BATTERY_INTERNAL 1 // Пин АЦП для замера батареи (ADC1_CH0)
#define PIN_ADC_CTRL 37        // Пин управления питанием встроенного делителя
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

#define DEFAULT_TURNAROUND 300    
#define WORK_COMM_ATTEMPTS 3
#define PING_FLASH 100  //ms
#define DEBOUNCE_TIME 100 

//Описание протокола:
#define CMD_SIGNAL         208 
#define CMD_SIGNAL_OK      209 
#define CMD_PING        212 
#define CMD_PING_OK     213 

byte workAddress = WORK_ADDRESS;  
byte sndCmd = CMD_PING;              
byte sndData;                         
bool wasReceived = 0;                     
byte cmdExpected = CMD_PING_OK;          
byte rcvAddress = 0;          
byte rcvCmd = 0;                      
byte rcvData;                         
byte workChannel;                      
unsigned long workFrequency = WORK_FREQUENCY; 

long lastSendTime = 0;                
int lastRSSI;
float lastSNR;
unsigned long lastTurnaround = DEFAULT_TURNAROUND;         
long lastFrequencyError;
unsigned long lastButtonTime; 

bool currButtonState;               
bool prevButtonState;               
bool buttonPressedFirstTime;              
bool buttonChanged;
bool buttonSent;

unsigned long pingTimer;
unsigned long pingFlashTimer;
bool pingFlash;

int fbledBrightness = FB_LED_BRIGHTNESS;           
int pwmledBrightness = BIG_LED_BRIGHTNESS;           

volatile bool receivedFlag = false;

// --- ПРОТОТИПЫ ФУНКЦИЙ ---
void processButton();
void processPing();
void updateStatusLed(bool ledStatus);
void updateBIGLed(bool ledStatus);
void flashStatusLed(byte times);
void sendMessage(byte msgCmd, byte sndData);
bool commSession( byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes );
void setLoRaParams();
void onReceive(byte* payload, int packetSize); 
void checkReceive(); 
bool testBattery();
bool batteryVoltageOK(byte tries);
float batteryVoltage();
void showBatteryVoltage();
void showNoBattery();
void flashBatteryLEDOnce();
void flashLedBattery(byte times);
void processBattery();
void stopWorking();

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}

void processButton() {
  prevButtonState = currButtonState;
  currButtonState = !digitalRead(PIN_BUTTON); 
  if (prevButtonState != currButtonState) {   
    lastButtonTime = millis();
    pingTimer = millis(); 
    buttonPressedFirstTime = true;
    DEBUGln("\nprocessButton(): " + String(currButtonState));
    prevButtonState = currButtonState;
    
    if (currButtonState) {
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
    else { 
      sendMessage(CMD_SIGNAL, false); 
      updateStatusLed(false);
      updateBIGLed(false);
    }
  }
}

void processPing() {
  if (!buttonPressedFirstTime) return;
  if (pingFlash) {
    if ((millis() - pingFlashTimer) > PING_FLASH) { 
      DEBUGln(F("\tPing LED OFF"));
      pingFlash = false;
      updateStatusLed(currButtonState);
    }
  }
  else if ((millis() - pingTimer) > PING_TIMEOUT) { 
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
  if ((millis() - lastButtonTime) > BIG_TIMEOUT) { 
    flashStatusLed(3);  
    buttonPressedFirstTime = false; 
  }
}

void updateStatusLed(bool ledStatus) { 
  analogWrite(PIN_FB_LED, ledStatus * fbledBrightness);
}

void updateBIGLed(bool ledStatus) { 
  analogWrite(PIN_BIG_LED, ledStatus * pwmledBrightness);
}

void flashStatusLed(byte times) { 
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
  byte payload[3] = {workAddress, msgCmd, sndData};
  
  int state = radio.transmit(payload, 3);
  
  if (state == RADIOLIB_ERR_NONE) {
    DEBUGln(("\tMessage sent: ") + String(workAddress) + " " + String(msgCmd) + " " + String(sndData));
  } else {
    DEBUGln(("\tTransmit failed, code: ") + String(state));
  }
  
  lastSendTime = millis();
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

bool commSession( byte msgCmd, byte sndData, byte expectedReply, unsigned long waitMilliseconds, int doTimes ) {
  DEBUGln(F("commSession()"));
  wasReceived = false;
  cmdExpected = expectedReply;
  pingTimer = millis(); 
  do {
    checkReceive(); 
    EVERY_MS(waitMilliseconds) {
      sendMessage(msgCmd, sndData);
      doTimes--;
    }
  } while ((doTimes > 0) && (!wasReceived));
  pingTimer = millis(); 
  return wasReceived;
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
  radio.setPreambleLength(6);
  radio.setSyncWord(0x1400 | WORK_ADDRESS);     
}

void onReceive(byte* payload, int packetSize) {
  DEBUGln("<<<PackageReceived");

  rcvAddress = payload[0];          
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
    delay(30); 
    return;
  }

  rcvCmd = payload[1];    
  if (rcvCmd != cmdExpected) {
    DEBUGln("\tInvalid Reply: " + String(rcvCmd) + F(", Expected: ") + String(cmdExpected));
    delay(30); 
    return;
  }
  rcvData = payload[2];

  lastTurnaround = millis() - lastSendTime;
  lastFrequencyError = radio.getFrequencyError();
  lastRSSI = radio.getRSSI();
  lastSNR = radio.getSNR();
  
  DEBUGln("\tReceived Message: "  + String(rcvAddress) +" " + String( rcvCmd) + " " + String( rcvData));
  DEBUGln(("\tRSSI: ") + String(lastRSSI));
  DEBUGln(("\tSnr: ") + String(lastSNR));
  DEBUGln(("\tTurnaround: ") + String(lastTurnaround));
  DEBUGln(("\tFrequency Error: ") + String(lastFrequencyError));
  DEBUGln(("\tRX Working Frequency:\t") + String(workFrequency - lastFrequencyError));
  DEBUGln(F("=== onReceive done ==="));

  wasReceived = true;
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
  
  // Делаем серию замеров, чтобы поймать "пилу" зарядника
  for (byte i = 0; i < tries; i++) {
    float currentVBat = batteryVoltage();
    
    // Записываем экстремумы
    if (currentVBat < minV) minV = currentVBat;
    if (currentVBat > maxV) maxV = currentVBat;
    
    // Если напряжение абсолютно аномальное - батареи точно нет
    if ((currentVBat > 4.3) || (currentVBat < 2.5)) {
      DEBUGln(F("Voltage out of bounds (No battery or deeply discharged)!"));
      return (false);
    }
    
    delay(150); // Ждем между замерами, чтобы дать "пиле" опуститься/подняться
  }
  
  DEBUG(F("Min V: ")); DEBUG(minV); DEBUG(F(", Max V: ")); DEBUGln(maxV);
  
  // Ключевая проверка физики LiPo: 
  // Настоящая батарея держит напряжение стабильно. 
  // Если разброс больше 0.1V - это пульсации пустого контроллера заряда.
  if ((maxV - minV) > 0.05) {
    DEBUGln(F("Voltage is unstable (Sawtooth ripple)! Батарея НЕ подключена!"));
    return (false);
  }
  
  return (true);
}
float batteryVoltage() {
  DEBUG(F("Battery Voltage: "));
  
  // Включаем встроенный делитель напряжения батареи (ADC_CTRL, Active Low)
  digitalWrite(PIN_ADC_CTRL, LOW);
  delay(10); 
  
  float measuredvbat = analogRead(PIN_BATTERY_INTERNAL);
  
  // Отключаем делитель
  digitalWrite(PIN_ADC_CTRL, HIGH);

  measuredvbat *= 3.3;  
  measuredvbat /= 4095.0; 
  measuredvbat *= HELTEC_BATTERY_MULTIPLIER;
  // Из даташита: VBAT = 100 / (100+390) * VADC_IN1

  DEBUGln(measuredvbat);
  return measuredvbat;
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

void processBattery() {
  if (batteryVoltage() < BATTERY_MIN_VOLTAGE) {
    stopWorking();
  }
}

void stopWorking() {
  flashLedBattery(7);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(5000);
  flashLedBattery(7);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(5000);
  
  esp_deep_sleep_start();
}

void setup() {
  delay(2000);   

#ifdef DEBUG_ENABLE
  Serial.begin(115200); 
  while (!Serial);
#endif

  DEBUGln("================================");
  DEBUGln("=========== START TX ===========");
  DEBUGln("DavayLoRa TX setup()");

  // Инициализация пина управления делителем АЦП
  pinMode(PIN_ADC_CTRL, OUTPUT);
  digitalWrite(PIN_ADC_CTRL, HIGH); 

  // На всякий случай выключаем и Vext, чтобы не тратить батарею впустую
  pinMode(36, OUTPUT);
  digitalWrite(36, HIGH); 

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_FB_LED, OUTPUT);
  pinMode(PIN_BIG_LED, OUTPUT);
  pinMode(PIN_BATTERY_LED, OUTPUT);
  digitalWrite(PIN_FB_LED, 0); 
  digitalWrite(PIN_BIG_LED, 0); 
  digitalWrite(PIN_BATTERY_LED, 0); 
  delay(300);

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
  delay(100);

  DEBUGln("DavayLoRa TX setup complete, waiting for 1-st buttonpress");
}

void loop() {
  checkReceive(); 
  
  if ((millis() - lastButtonTime) > DEBOUNCE_TIME)
    processButton();
    
  processPing();
  
  EVERY_MS(BATTERY_PERIOD) {
    if (measurebattery) processBattery();
  }
}