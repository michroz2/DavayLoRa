void onReceive(int packetSize) {
  DEBUGln(F("\n<<<Package Received!"));
  rcvAddress = LoRa.read();          // replied address
  if ((rcvAddress != workAddress) || (packetSize != 3)) {
    DEBUGln("Invalid package! "  \
        + String(rcvAddress) + F(", Expected: ") + String(workAddress));
    return;
  }

  rcvCmd = LoRa.read();              // received command
  rcvData = LoRa.read();                  // received data

  lastFrequencyError = LoRa.packetFrequencyError();
  workFrequency = workFrequency - lastFrequencyError / 2;
  LoRa.setFrequency(workFrequency);
//  delay(30);

#ifdef DEBUG_ENABLE  //только имеет смысл для дебагинга
  lastRSSI =  LoRa.packetRssi();
  lastSNR = LoRa.packetSnr();
  lastTurnaround = millis() - lastSendTime;
  DEBUGln("\tRSSI:\t" + String(lastRSSI));
  DEBUGln("\tSnr:\t" + String(lastSNR));
  DEBUGln("\tTurnaround:\t" + String(lastTurnaround));
  DEBUGln("\tFrequency Error:\t" + String(lastFrequencyError));
  DEBUGln("\tReceived Message:\t"  + String(rcvAddress)\
          +" " + String( rcvCmd) + " " + String( rcvData));
#endif


}//void onReceive(int packetSize)
