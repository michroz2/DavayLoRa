void loop() { //  ===!!!===!!!===!!!===LOOP===!!!===!!!===!!!===!!!===!!!===

  if (rcvCmd)
    processCommand();
  else
    processTimeOut();  //see if we haven't received from TX for long enough

  processCutoff();

  EVERY_MS(BATTERY_PERIOD) {
    processBattery();
  }

}//loop()         ===!!!===!!!===!!!===END LOOP===!!!===!!!===!!!===!!!===!!!===
