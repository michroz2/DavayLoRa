void onReceive(int packetSize) {
  DEBUGln(F("\n<<<Package Received"));

  rcvAddress = LoRa.read();          // replied address
  if ((rcvAddress != workAddress) || (packetSize != 3)) {
    DEBUGln(F("Invalid package! "));
    DEBUG(F("Received address: "));
    DEBUGln(rcvAddress);
    DEBUG(F("Expected address: "));
    DEBUGln(workAddress);
    DEBUG(F("Package length: "));
    DEBUGln(packetSize);
    //    delay(30); //пропускаем немного времени - пока работает чужое радио
    return;
  }

  rcvCmd = LoRa.read();              // received command
  rcvData = LoRa.read();                  // received data

  lastFrequencyError = LoRa.packetFrequencyError();

#ifdef DEBUG_ENABLE
  lastRSSI =  LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
  DEBUGln("\tReceived Message:\t"  + String(rcvAddress)\
          +" " + String( rcvCmd) + " " + String( rcvData));
  DEBUGln("\tRSSI:\t" + String(lastRSSI));
  DEBUGln("\tSnr:\t" + String(lastSNR));
  DEBUGln("\tFrequency Error:\t" + String(lastFrequencyError));
  DEBUGln(("\tWorking Frequency OLD:\t") + String(workFrequency));
#endif

  workFrequency = workFrequency - lastFrequencyError / 2;
  LoRa.setFrequency(workFrequency);
  //  delay(30);

  DEBUGln(("\tWorking Frequency NEW:\t") + String(workFrequency));
  DEBUGln(F("=== onReceive done ==="));


}//void onReceive(int packetSize)
