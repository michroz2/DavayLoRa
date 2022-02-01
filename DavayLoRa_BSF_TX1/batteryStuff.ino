float batteryVoltage() {
  float measuredvbat = analogRead(PIN_BATTERY);
  measuredvbat = analogRead(PIN_BATTERY);
  measuredvbat *= BATTERY_VOLTAGE_MULTIPLIER;    // multiply according to the used board divider
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  DEBUG(F("Battery Voltage: "));
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

void flashBatteryLEDOnce() {
  digitalWrite(PIN_BATTERY_LED, 1);
  delay(250);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(250);
}

void flashLedBattery(byte times) { //flash 3 times total 1.5 sec
  DEBUGln("flashLedBattery()");
  bool flash = false;
  for (int i = 0; i < times * 2; i++) {
    digitalWrite(PIN_BATTERY_LED, flash);
    flash = !flash;
    delay(200);
  }
}

void processBattery() {

  if (batteryVoltage() < BATTERY_MIN_VOLTAGE) {
    stopWorking();
  }

}
