/**
 * @file Battery.cpp
 * @version 1.53
 * @brief Реализация подсистемы питания (RX)
 */
 #include "Battery.h"

 // Локальные макросы отладки
 #define DEBUG_ENABLE // Логирование выключено
 #ifdef DEBUG_ENABLE
 #define DEBUG(x) Serial.print(x)
 #define DEBUGln(x) Serial.println(x)
 #else
 #define DEBUG(x)
 #define DEBUGln(x)
 #endif
 
 bool isBatteryConnected = false;
 
 // Проверка наличия подключенной батареи при запуске
 bool testBattery() { 
    if (batteryVoltageOK(5)) return true;
    return false;
 } // конец функции testBattery
 
 // Серия замеров напряжения для фильтрации шумов и проверки стабильности
 bool batteryVoltageOK(byte tries) {
    float minV = 5.0, maxV = 0.0;
    for (byte i = 0; i < tries; i++) {
      float currentVBat = batteryVoltage();
      if (currentVBat < minV) minV = currentVBat; 
      if (currentVBat > maxV) maxV = currentVBat; 
      if ((currentVBat > 4.5) || (currentVBat < 2.5)) return false;
      delay(150);
    } // конец цикла замеров
    if ((maxV - minV) > 0.05) return false;
    return true;
 } // конец функции batteryVoltageOK
 
 // Считывание напряжения через АЦП микроконтроллера
 float batteryVoltage() {
    digitalWrite(PIN_ADC_CTRL, LOW); delay(10); // Включение цепи делителя напряжения                      
    float measuredvbat = analogRead(PIN_BATTERY_INTERNAL);
    digitalWrite(PIN_ADC_CTRL, HIGH); // Отключение делителя для экономии энергии
    measuredvbat *= 3.3; measuredvbat /= 4095.0; measuredvbat *= HELTEC_BATTERY_MULTIPLIER; 
    return measuredvbat;
 } // конец функции batteryVoltage
 
 // Периодическая проверка. Если разряд критический — выключение устройства
 void processBattery() { 
    if (batteryVoltage() < BATTERY_MIN_VOLTAGE) stopWorking(); 
 } // конец функции processBattery
 
 // Индикация заряда статусной лампой (до 5 вспышек)
 void showBatteryVoltage() {
    float voltage = batteryVoltage();
    if (voltage > BATTERY_VOLTAGE_1) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_2) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_3) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_4) flashBatteryLEDOnce(); 
    if (voltage > BATTERY_VOLTAGE_5) flashBatteryLEDOnce(); 
 } // конец функции showBatteryVoltage
 
 // Индикация отсутствия батареи (например, питание от USB)
 void showNoBattery() { 
    digitalWrite(PIN_BATTERY_LED, 1); delay(2000); digitalWrite(PIN_BATTERY_LED, 0); delay(250); 
 } // конец функции showNoBattery
 
 void flashBatteryLEDOnce() { 
    digitalWrite(PIN_BATTERY_LED, 1); delay(250); digitalWrite(PIN_BATTERY_LED, 0); delay(250); 
 } // конец функции flashBatteryLEDOnce
 
 void flashLedBattery(byte times) { 
    for (int i = 0; i < times; i++) flashBatteryLEDOnce(); 
    delay(200); 
 } // конец функции flashLedBattery
 
 // Принудительное засыпание приемника для защиты аккумулятора
 void stopWorking() { 
    DEBUGln(F("[STATE] BATTERY DEAD! Stopping..."));
    flashLedBattery(7); digitalWrite(PIN_BATTERY_LED, 0); delay(3000); 
    flashLedBattery(7); digitalWrite(PIN_BATTERY_LED, 0); enterDeepSleep(); 
 } // конец функции stopWorking