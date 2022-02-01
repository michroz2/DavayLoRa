void stopWorking() {
  flashLedBattery(7);
  delay(5000);
  flashLedBattery(7);
  while (1) {
    power.setSleepMode(POWERDOWN_SLEEP); // Крепко засыпаем
    delay(100); // даем время на отправку
    power.sleep(SLEEP_FOREVER); // спим до перезагрузки

  }
}
