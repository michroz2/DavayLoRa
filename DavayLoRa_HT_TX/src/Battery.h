/**
 * @file Battery.h
 * @version 1.53
 * @brief Подсистема питания и контроля батареи (TX)
 */
 #ifndef BATTERY_H
 #define BATTERY_H
 
 #include <Arduino.h>
 
 // --- Пороги напряжений ---
 #define BATTERY_MIN_VOLTAGE 3.5
 #define BATTERY_VOLTAGE_1 3.5
 #define BATTERY_VOLTAGE_2 3.6
 #define BATTERY_VOLTAGE_3 3.8
 #define BATTERY_VOLTAGE_4 3.9
 #define BATTERY_VOLTAGE_5 4.0
 
 // --- Пины и константы батареи ---
 #define PIN_BATTERY_LED 35     
 #define PIN_BATTERY_INTERNAL 1 
 #define PIN_ADC_CTRL 37        
 #define HELTEC_BATTERY_MULTIPLIER 4.9
 
 extern bool isBatteryConnected;
 
 // Внешние зависимости (функции, которые остались в main.cpp)
 extern void updateStatusLed(bool ledStatus);
 extern void enterDeepSleep();
 
 // Прототипы функций
 bool testBattery();
 bool batteryVoltageOK(byte tries);
 float batteryVoltage();
 void showBatteryVoltage();
 void showNoBattery();
 void flashBatteryLEDOnce();
 void processBattery();
 void stopWorking();
 
 #endif