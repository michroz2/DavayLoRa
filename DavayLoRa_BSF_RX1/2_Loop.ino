void loop() { //  ===!!!===!!!===!!!===LOOP===!!!===!!!===!!!===!!!===!!!===

  if (rcvCmd)
    processCommand();
  else
    processTimeOut();  //see if we haven't received from TX for long enough

  processCutoff();

  EVERY_MS(BATTERY_PERIOD) {
    if (measurebattery) {
      processBattery();
    }
  }

}//loop()         ===!!!===!!!===!!!===END LOOP===!!!===!!!===!!!===!!!===!!!===
