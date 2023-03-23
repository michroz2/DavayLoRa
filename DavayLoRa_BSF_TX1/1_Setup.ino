void setup() {//=======================SETUP===============================
  delay(2000);   // Give time to the ATMega32u4 port to wake up and be recognized by the OS.
  power.hardwareEnable(PWR_ALL); //На всякий случай включим все системы микропроцессора

  // initialize serial for debug output if needed
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
#endif

  DEBUGln("================================");
  DEBUGln("=========== START TX ===========");
  DEBUGln("DavayLoRa TX setup()");

  //INIT PINS for button and status LEDs:
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_FB_LED, OUTPUT);
  pinMode(PIN_BIG_LED, OUTPUT);
  pinMode(PIN_BATTERY_LED, OUTPUT);
  digitalWrite(PIN_FB_LED, 0); //switch off FB on status led
  digitalWrite(PIN_BIG_LED, 0); //switch off big monitoring led
  digitalWrite(PIN_BATTERY_LED, 0); //switch off battery indicating led (built-in normally)
  //  digitalWrite(PD5, HIGH);
  delay(300);

  //Приветственный сигнал 1 сек
  updateStatusLed(true);
  analogWrite(PIN_BIG_LED, pwmledBrightness);
  digitalWrite(PIN_FB_LED, HIGH);
  digitalWrite(PIN_BATTERY_LED, HIGH);
  delay(1000);
  updateStatusLed(false);
  analogWrite(PIN_BIG_LED, 0);
  digitalWrite(PIN_FB_LED, LOW);
  digitalWrite(PIN_BATTERY_LED, LOW);
  delay(1000);

  DEBUGln(F("Battery Test"));
  measurebattery = testBattery();     //This function defines if the battery can be measured
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

  // override the LoRa library default CS, reset, and IRQ pins with the board values
  LoRa.setPins(csPin, resetPin, irqPin);  // set CS, reset, IRQ pin
  delay(300);

  workFrequency = workingFrequency[WORK_ADDRESS % MAX_ADDRESS];
  DEBUG("LoRa begin on ");
  DEBUGln(workFrequency);
  if (!LoRa.begin(workFrequency)) {             // initialize radio at workFrequency
    DEBUGln("LoRa init failed. Check your connections.");
    while (true) {
      flashStatusLed(6);    // if failed, do nothing
      delay(4000);
    }
  }

  setLoRaParams();          //Tweak parameters for best communication

  LoRa.onReceive(onReceive);
  delay(100);

//  LoRa.onTxDone(onTxDone);

  LoRa.idle();              //Until we decide how to continue
  delay(100);

  DEBUGln("DavayLoRa TX setup complete, waiting for 1-st buttonpress");

}//setup      //======================= /SETUP ===============================
