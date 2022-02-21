void onReceive(int packetSize) {
  DEBUGln("<<<PackageReceived");

  rcvAddress = LoRa.read();          // replied address
  if ((rcvAddress != workAddress) || (packetSize != 3)) {
#ifdef DEBUG_ENABLE
    DEBUGln(F("Invalid package! "));
    DEBUG(F("Received address: "));
    DEBUGln(rcvAddress);
    DEBUG(F("Expected address: "));
    DEBUGln(workAddress);
    DEBUG(F("Package length: "));
    DEBUGln(packetSize);
#endif
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

  lastTurnaround = millis() - lastSendTime;

  lastFrequencyError = LoRa.packetFrequencyError();
  lastRSSI =  LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
  DEBUGln("\tReceived Message: "  + String(rcvAddress) +" " + String( rcvCmd) + " " + String( rcvData));
  DEBUGln(("\tRSSI: ") + String(lastRSSI));
  DEBUGln(("\tSnr: ") + String(lastSNR));
  DEBUGln(("\tTurnaround: ") + String(lastTurnaround));
  DEBUGln(("\tFrequency Error: ") + String(lastFrequencyError));
  DEBUGln(("\tRX Working Frequency:\t") + String(workFrequency - lastFrequencyError));
  DEBUGln(F("=== onReceive done ==="));

  wasReceived = true;

}//void onReceive(int packetSize)
