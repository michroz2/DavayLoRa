/**
 * @file Battery.cpp
 * @version 1.53
 * @brief Реализация подсистемы питания (TX)
 */
 #include "Battery.h"

 // Локальные макросы отладки
 // #define DEBUG_ENABLE // Логирование выключено
 #ifdef DEBUG_ENABLE
 #define DEBUG(x) Serial.print(x)
 #define DEBUGln(x) Serial.println(x)
 #else
 #define DEBUG(x)
 #define DEBUGln(x)
 #endif
 
 bool isBatteryConnected = false;
 
 // Проверяет, подключена ли батарея, делая несколько замеров
 bool testBattery() { return batteryVoltageOK(5); } // конец функции testBattery
 
 bool batteryVoltageOK(byte tries) {
    float minV = 5.0, maxV = 0.0;
    for (byte i = 0; i < tries; i++) {
      float currentVBat = batteryVoltage();
      if (currentVBat < minV) minV = currentVBat; 
      if (currentVBat > maxV) maxV = currentVBat; 
      delay(150);
    } // конец цикла замеров
    return (maxV - minV) <= 0.05;
 } // конец функции batteryVoltageOK
 
 // Чтение реального напряжения через внутренний АЦП и делитель
 float batteryVoltage() {
    digitalWrite(PIN_ADC_CTRL, LOW); delay(10);
    float measuredvbat = analogRead(PIN_BATTERY_INTERNAL);
    digitalWrite(PIN_ADC_CTRL, HIGH);
    measuredvbat *= 3.3 * HELTEC_BATTERY_MULTIPLIER / 4095.0;
    return measuredvbat;
 } // конец функции batteryVoltage
 
 // Индикация заряда (от 1 до 5 вспышек в зависимости от напряжения)
 void showBatteryVoltage() {
    float voltage = batteryVoltage();
    if (voltage > BATTERY_VOLTAGE_1) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_2) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_3) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_4) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_5) flashBatteryLEDOnce(); 
 } // конец функции showBatteryVoltage
 
 // Индикация отсутствия батареи (длинная вспышка)
 void showNoBattery() { 
    digitalWrite(PIN_BATTERY_LED, 1); delay(2000); digitalWrite(PIN_BATTERY_LED, 0); delay(250); 
 } // конец функции showNoBattery
 
 void flashBatteryLEDOnce() { updateStatusLed(true); delay(250); updateStatusLed(false); delay(250); } // конец функции flashBatteryLEDOnce
 
 // Регулярная проверка. Если батарея села - уходим в сон
 void processBattery() { if (batteryVoltage() < BATTERY_MIN_VOLTAGE) stopWorking(); } // конец функции processBattery
 
 void stopWorking() { 
    DEBUGln(F("[STATE] BATTERY DEAD! Stopping..."));
    enterDeepSleep(); 
 } // конец функции stopWorking