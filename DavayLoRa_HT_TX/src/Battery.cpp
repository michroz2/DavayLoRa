#include "Battery.h"

bool isBatteryConnected = false;

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

float batteryVoltage() {
   digitalWrite(PIN_ADC_CTRL, LOW); delay(10);
   float measuredvbat = analogRead(PIN_BATTERY_INTERNAL);
   digitalWrite(PIN_ADC_CTRL, HIGH);
   measuredvbat *= 3.3 * HELTEC_BATTERY_MULTIPLIER / 4095.0;
   return measuredvbat;
} // конец функции batteryVoltage

void showBatteryVoltage() {
   float voltage = batteryVoltage();
   if (voltage > BATTERY_VOLTAGE_1) flashBatteryLEDOnce(); 
   if (voltage > BATTERY_VOLTAGE_2) flashBatteryLEDOnce(); 
   if (voltage > BATTERY_VOLTAGE_3) flashBatteryLEDOnce(); 
   if (voltage > BATTERY_VOLTAGE_4) flashBatteryLEDOnce(); 
   if (voltage > BATTERY_VOLTAGE_5) flashBatteryLEDOnce(); 
} // конец функции showBatteryVoltage

void showNoBattery() { 
   digitalWrite(PIN_BATTERY_LED, 1); delay(2000); digitalWrite(PIN_BATTERY_LED, 0); delay(250); 
} // конец функции showNoBattery

void flashBatteryLEDOnce() { updateStatusLed(true); delay(250); updateStatusLed(false); delay(250); } // конец функции flashBatteryLEDOnce

void processBattery() { if (batteryVoltage() < BATTERY_MIN_VOLTAGE) stopWorking(); } // конец функции processBattery

void stopWorking() { enterDeepSleep(); } // конец функции stopWorking