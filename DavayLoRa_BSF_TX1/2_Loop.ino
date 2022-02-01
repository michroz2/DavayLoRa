void loop() { //  ===!!!===!!!===!!!===!!!= LOOP =!!!===!!!===!!!===!!!===!!!===

  if ((millis() - lastButtonTime) > DEBOUNCE_TIME)
    processButton();
  processPing();
  EVERY_MS(BATTERY_PERIOD) {
    processBattery();
  }

}//loop()         ===!!!===!!!===!!!===!!!= LOOP =!!!===!!!===!!!===!!!===!!!===
