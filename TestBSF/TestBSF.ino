/* Программа проверки плат BSFrance на работу делителя напряжения.
 *  Для проверки вставить батарею и, если плата уже «перерезана»,
 *  то Дюпонным папа-папа проводочком соединить контакт BAT и «+» батареи
 *  (засунуть один пин Дюпона  в контакт BAT, вместе с резистором, если он там уже есть,
 *  а второй пин - в разъём самой батарейки сзади, откуда подходит крастный провод батареи)
 *  
 *  Программа выводит результат на Монитор или в виде сигналов: 
 *  Порядок сигналов:
 *  1 длинная вспышка = Тестируем внутренний делитель!
 *  Далее следует количество коротких вспышек 1-5 в зависимости от напряжения батареи (как на готовом приборе)
 *  Если работает, то Опять 1 длинная вспышка и потом идут бесконечно по 1 короткой - это результат (=работает внутренний делитель)
 *  Если вспышек 7, то внутренний измеритель не работает, то далее следует:
 *  2 длинных вспышки = Тестируем внешний делитель на резисторах
 *  (если его физически нет, то смысла продолжать нет)
 *  Если работает делительн на резисторах, то Опять 2 длинных вспышки, 
 *  а потом идут бесконечно по 2 коротких - это результат (=работает внешний делитель на резисторах)
 *  Если вспышек 7, то не работает.
 *  Далее идёт бесконечная серия по 3 коротких вспышки - это результат (= делители не работают, измрять напряжение батареи нельзя)
 *  На такие платы надо или запаивать резисторы, или забивать на измерение батарейки (менять соответствующий параметр перед загрузкой) 
*/
#include <Arduino.h>              // include libraries

// Дебагирование с монитором : раскомментить для использования 1 строчку:
#define DEBUG_ENABLE
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

#define BATTERY_MIN_VOLTAGE 3.5   //Volt min.
#define BATTERY_VOLTAGE_1 3.5 // Мигание 1 раз
#define BATTERY_VOLTAGE_2 3.6
#define BATTERY_VOLTAGE_3 3.8
#define BATTERY_VOLTAGE_4 3.9
#define BATTERY_VOLTAGE_5 4.0
#define PIN_BATTERY_INTERNAL 9    // Номер внутреннего пина для измерения батарейки
#define PIN_BATTERY_RESISTOR 19 //то же что и A1 Номер пина для измерения батарейки, куда мы паяем резисторы
#define INTERNAL_VOLTAGE_MULTIPLIER 1.27  // Так выбрано у платы BSFrance
#define RESISTOR_VOLTAGE_MULTIPLIER 2 //Потому что мы если паяем делитель, то 2 одинаковых резистора
#define PIN_BATTERY_LED LED_BUILTIN  // Номер пина для показа напряжения батарейки

#define BATTERY_PERIOD 2000 //Каждые столько миллисекунд измеряется напряжение батареи 

float batteryVoltageMultiplier = 1.27;
int batteryPIN;


void setup() {// ///////////////////////////SETUP//////////////////////////////////////
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
#endif

  DEBUGln(F("Проверка платы BSFrance LoRa32u4"));

  // 1) Проверка внутреннего делителя на пине 9 :
  DEBUGln(F("1. Проверка встроенного измерителя напряжения батареи"));
  batteryVoltageMultiplier = INTERNAL_VOLTAGE_MULTIPLIER;
  batteryPIN = PIN_BATTERY_INTERNAL;
  // Flash long 1 time to show the first type of measuring
  flashLedBatteryOnceLong();
  if (batteryVoltageOK(5)) {
    showBatteryVoltage();
    DEBUGln(F("Встроенный измеритель работает!"));
    DEBUGln(F("Используем встроенный измеритель."));
    delay(1000);
    flashLedBatteryOnceLong();
    delay(1000);
    while (1) {
      EVERY_MS(BATTERY_PERIOD) {
        batteryVoltage();
        flashLedBattery(1);
      }
    }
  }
  DEBUGln(F("Встроенный измеритель НЕ РАБОТАЕТ!"));
  flashLedBattery(7);
  delay(1000);

  // 1) Проверка делителя на резисторах на пине 19
  DEBUGln(F("2. Проверка измерителя напряжения НА ВНЕШНИХ РЕЗИСТОРАХ, подключенных к А1"));
  batteryVoltageMultiplier = RESISTOR_VOLTAGE_MULTIPLIER;
  batteryPIN = PIN_BATTERY_RESISTOR;
  // Flash long 2 times to show the first type of measuring
  flashLedBatteryOnceLong();
  flashLedBatteryOnceLong();
  if (batteryVoltageOK(5)) {
    showBatteryVoltage();
    DEBUGln(F("Измеритель на резисторах работает!"));
    DEBUGln(F("Используем измеритель на резисторах"));
    delay(1000);
    flashLedBatteryOnceLong();
    flashLedBatteryOnceLong();
    delay(1000);
    while (1) {
      EVERY_MS(BATTERY_PERIOD) {
        batteryVoltage();
        flashLedBattery(2);
      }
    }
  }
  DEBUGln(F("Измеритель на резисторах НЕ РАБОТАЕТ!"));
  DEBUGln(F("Запаяйте резисторы или используйте режим без измерения напряжения батарейки!"));
  flashLedBattery(7);
  delay(1000);

  while (1) {
    EVERY_MS(BATTERY_PERIOD) {
      batteryVoltage();
      flashLedBattery(3);
    }
  }
}//////////////////////////////////////////setup

void loop() {

}

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
  delay(200);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(200);
}

float batteryVoltage() {
  DEBUG(F("Battery Voltage: "));
  float measuredvbat = analogRead(batteryPIN);
  measuredvbat *= batteryVoltageMultiplier;    // multiply according to the used board divider
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  DEBUGln(measuredvbat);
  return measuredvbat;
}////batteryVoltage()

void flashLedBattery(byte times) { //flash "times" times
  //  DEBUGln(F("flashLedBattery()"));
  for (int i = 0; i < times; i++)
    flashLedBatteryOnce();
  delay(200);
}

void flashLedBatteryOnce() {
  digitalWrite(PIN_BATTERY_LED, 1);
  delay(250);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(250);
}

void flashLedBatteryOnceLong() {
  digitalWrite(PIN_BATTERY_LED, 1);
  delay(1000);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(500);
}

bool batteryVoltageOK(byte tries) {
  DEBUGln(F("Is battery voltage OK?"));
  float averageVBat, currentVBat;
  for (byte i = 0; i < tries; i++) {
    currentVBat = batteryVoltage();
    averageVBat *= i;
    averageVBat += currentVBat;
    averageVBat /= (i + 1);
    if ((currentVBat > 4.5) || (currentVBat < 1)) {
      DEBUGln(F("Battery Voltage too high or too low!"));
      return (false);
    }
    DEBUG(F("Average Voltage: "));
    DEBUGln(averageVBat);
    if (abs(currentVBat - averageVBat) > 0.2) {
      DEBUGln(F("Battery Voltage not steady enough!"));
      DEBUGln(F("Возможно, батарея не подключена!"));
      return (false);
    }
    delay(500);
  }
  return (true);
}
