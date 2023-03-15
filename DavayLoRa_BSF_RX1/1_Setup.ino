void setup() {//=======================SETUP===============================
  delay(2000);   // Give time to the ATMega32u4 port to wake up and be recognized by the OS.
  power.hardwareEnable(PWR_ALL);

  // initialize serial
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
#endif

  DEBUGln(F("================================"));
  DEBUGln(F("=========== START RX ==========="));
  DEBUGln(F("DavayLoRa RX setup()"));

  //INIT PINS
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_SIGNAL_BUZZERS, OUTPUT);
  pinMode(PIN_SIGNAL_LED, OUTPUT);
  analogWrite(PIN_SIGNAL_LED, 0); //just in case - switch off FB on big led
  digitalWrite(PIN_SIGNAL_BUZZERS, 0);
  digitalWrite(PIN_BATTERY_LED, 0);
  delay(300);

  //Приветственный сигнал 1 сек
  updateStatusLed(true);
  analogWrite(PIN_SIGNAL_LED, BIG_LED_BRIGHTNESS);
  analogWrite(PIN_SIGNAL_BUZZERS, BUZZER_BIPPER_VOLUME);
  delay(1000);
  updateStatusLed(false);
  analogWrite(PIN_SIGNAL_LED, 0);
  digitalWrite(PIN_SIGNAL_BUZZERS, LOW);  //LOW is the same as 0
  delay(1000);

  DEBUGln(F("Battery Test"));
  if (measurebattery) {
  DEBUGln(F("-Measuring"));
    processBattery(); //Если заряд батарейки недостаточен, то моргаем 2 серии по 7 раз и выключаемся
    // два раза показываем заряд батарейки:
    showBatteryVoltage();
    delay(2000);   //
    showBatteryVoltage();
    delay(500);   //
  }
  else {
    DEBUGln(F("-Cancelled"));
    showNoBattery();
    delay(500);   //
  }

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
  delay(300);

  workFrequency = workingFrequency[WORK_ADDRESS % MAX_ADDRESS];
  DEBUG("LoRa begin on ");
  DEBUGln(workFrequency);
  if (!LoRa.begin(workFrequency)) {             // initialize radio at workFrequency
    DEBUGln(F("LoRa init failed. Check your connections."));
    while (true) {
      flashStatusLed(6);    // if LoRa failed, blink and do nothing
      delay(4000);
    }
  }

  setLoRaParams();

  LoRa.onReceive(onReceive);
  //  LoRa.onTxDone(onTxDone);
  LoRa.receive(); //Always listen by default

  pingTimeOutLastTime = millis();

  DEBUGln(F("DavayLoRa RX setup complete"));
}//setup      //===================END SETUP===============================
