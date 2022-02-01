void setLoRaParams() {
  DEBUGln("setLoRaParams()");
  //Trying optimal settings for LoRa for Longest Range possible
  //Use LoRa Modem Calculator Tool found on
  //https://www.semtech.com/products/wireless-rf/lora-core/sx1276#download-resources
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);    //max
  delay(50);
  LoRa.setSignalBandwidth(125E3);                 //..31.25E3, 41.7E3, 62.5E3, (125E3), and 250E3.
  delay(50);
  LoRa.setSpreadingFactor(8);                    //default = 7
  delay(50);
  LoRa.setPreambleLength(6);                    //min = 6, default = 8
  delay(50);
  LoRa.setSyncWord(WORK_ADDRESS);
//  LoRa.enableCrc();                             //
  delay(50);
  //  LoRa.setCodingRate4(5);

}// DONE void setLoRaParams()
