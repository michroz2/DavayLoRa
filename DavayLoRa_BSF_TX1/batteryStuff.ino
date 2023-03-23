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
