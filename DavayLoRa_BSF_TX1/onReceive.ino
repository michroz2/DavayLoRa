void onReceive(int packetSize) {
  DEBUGln("<<<onReceive()");

  rcvAddress = LoRa.read();          // replied address
  if ((rcvAddress != workAddress) || (packetSize != 3)) {
    DEBUGln("Invalid package!");
    //    + String(rcvAddress) + F(", Expected: ") + String(workAddress));
    delay(30); //пропускаем немного времени - пока работает чужое радио
    return;
  }

  rcvCmd = LoRa.read();    // replied command
  if (rcvCmd != cmdExpected) {
    DEBUGln("\tInvalid Reply: " + String(rcvCmd) + F(", Expected: ") + String(cmdExpected));
    delay(30); //пропускаем немного времени - пока работает чужое радио
    return;
  }
  rcvData = LoRa.read();

#ifdef DEBUG_ENABLE
  lastRSSI =  LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
#endif

  lastTurnaround = millis() - lastSendTime;
  lastFrequencyError = LoRa.packetFrequencyError();

  DEBUGln("\tReceived Message: "  + String(rcvAddress) +" " + String( rcvCmd) + " " + String( rcvData));
  DEBUGln(("\tRSSI: ") + String(lastRSSI));
  DEBUGln(("\tSnr: ") + String(lastSNR));
  DEBUGln(("\tTurnaround: ") + String(lastTurnaround));
  DEBUGln(("\tFrequency Error: ") + String(lastFrequencyError));
  DEBUGln(("\tWorking Frequency OLD:\t") + String(workFrequency));

  workFrequency = workFrequency - lastFrequencyError / 2;
  LoRa.setFrequency(workFrequency);
  delay(30);

  DEBUGln(("\tWorking Frequency NEW:\t") + String(workFrequency));
  DEBUGln("=== updateFrequency() done ===");

  wasReceived = true;

}//void onReceive(int packetSize)
