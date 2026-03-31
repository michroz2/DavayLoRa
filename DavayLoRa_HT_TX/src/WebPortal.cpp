/**
 * @file WebPortal.cpp
 * @version 1.53
 * @brief Реализация Captive Portal (TX)
 */
 #include "WebPortal.h"
 #include "Config.h"
 #include "webpage.h" 
 
 // Локальные макросы отладки
 #define DEBUG_ENABLE
 #ifdef DEBUG_ENABLE
 #define DEBUG(x) Serial.print(x)
 #define DEBUGln(x) Serial.println(x)
 #else
 #define DEBUG(x)
 #define DEBUGln(x)
 #endif
 
 // Константа протокола, нужная для перезагрузки
 #define CMD_REBOOT 222
 
 const byte DNS_PORT = 53;
 WebServer server(80);
 DNSServer dnsServer;
 
 bool isWifiActive = false;
 unsigned long wifiStartTime = 0;
 bool exitConfigRequested = false;
 
 // Шаблон для вывода крупных сообщений на мобильном экране
 const String MSG_HEADER = F("<html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><style>body{font-family:sans-serif;text-align:center;margin-top:30%;font-size:1.5rem;background-color:#f4f4f9;color:#333;}</style></head><body>");
 const String MSG_FOOTER = F("</body></html>");
 
 void handleRoot() {
    DEBUGln(F("[WIFI] Client requested root page"));
    String html = String(index_html);
    
    unsigned long elapsed = millis() - wifiStartTime;
    long remainingSec = (configTimeout > elapsed) ? (configTimeout - elapsed) / 1000 : 0;
    html.replace("%TIME_LEFT%", String(remainingSec));
 
    html.replace("%ADDR%", String(workAddress));
    html.replace("%BAT_CHK%", measurebattery ? "checked" : "");
    html.replace("%BAT_PER%", String(batteryPeriod));
    html.replace("%WK_HOLD%", String(wakeUpHoldTime));
    html.replace("%WK_REL%", String(wakeUpReleaseWindow));
    html.replace("%STUCK_SL%", String(stuckSleepTime));
    html.replace("%CONF_TO%", String(configTimeout));
    html.replace("%SLP_LED%", String(sleepLedDuration));
 
    html.replace("%TX_BIG_LED%", String(pwmledBrightness));
    html.replace("%TX_FB_LED%", String(fbledBrightness));
    html.replace("%TX_PING%", String(pingTimeout));
    html.replace("%TX_BIG_TO%", String(bigTimeout));
    html.replace("%TX_EXEC_TO%", String(execTimeout));
 
    html.replace("%RX_BIG_EN%", rxSettings.rxEnableBigLed ? "checked" : "");
    html.replace("%RX_BIG_LED%", String(rxSettings.rxPwmledBrightness));
    html.replace("%RX_BUZ_EN%", rxSettings.rxEnableBuzzer ? "checked" : "");
    html.replace("%RX_BUZ_VOL%", String(rxSettings.rxBuzzerVolume));
    html.replace("%RX_CUTOFF%", String(rxSettings.rxCutoffTime));
    html.replace("%RX_PING%", String(pingTimeoutRX));
 
    server.send(200, "text/html", html);
 } // конец функции handleRoot
 
 void handleSave() {
    DEBUGln(F("[WIFI] === Web UI: Save Requested ==="));
    
    rxSettings.workAddress = server.arg("workAddress").toInt();
    rxSettings.measurebattery = server.hasArg("measurebattery");
    rxSettings.batteryPeriod = server.arg("batteryPeriod").toInt();
    rxSettings.wakeUpHoldTime = server.arg("wakeUpHoldTime").toInt();
    rxSettings.wakeUpReleaseWindow = server.arg("wakeUpReleaseWindow").toInt();
    rxSettings.stuckSleepTime = server.arg("stuckSleepTime").toInt();
    rxSettings.configTimeout = server.arg("configTimeout").toInt();
    rxSettings.sleepLedDuration = server.arg("sleepLedDuration").toInt();
    
    rxSettings.rxEnableBigLed = server.hasArg("rxEnableBigLed");
    rxSettings.rxPwmledBrightness = server.arg("rxPwmledBrightness").toInt();
    rxSettings.rxEnableBuzzer = server.hasArg("rxEnableBuzzer");
    rxSettings.rxBuzzerVolume = server.arg("rxBuzzerVolume").toInt();
    rxSettings.rxCutoffTime = server.arg("rxCutoffTime").toInt();
    rxSettings.pingTimeoutRX = server.arg("pingTimeoutRX").toInt();
 
    if (syncConfigToRX()) {
      DEBUGln(F("[WIFI] === RX Confirmed. Proceeding with Reboot ==="));
      
      sendMessage(CMD_REBOOT, 1);
      delay(500); 
 
      workAddress = rxSettings.workAddress;
      measurebattery = rxSettings.measurebattery;
      batteryPeriod = rxSettings.batteryPeriod;
      wakeUpHoldTime = rxSettings.wakeUpHoldTime;
      wakeUpReleaseWindow = rxSettings.wakeUpReleaseWindow;
      stuckSleepTime = rxSettings.stuckSleepTime;
      configTimeout = rxSettings.configTimeout;
      sleepLedDuration = rxSettings.sleepLedDuration;
      
      pwmledBrightness = server.arg("pwmledBrightness").toInt();
      fbledBrightness = server.arg("fbledBrightness").toInt();
      pingTimeout = server.arg("pingTimeout").toInt();
      bigTimeout = server.arg("bigTimeout").toInt();
      execTimeout = server.arg("execTimeout").toInt();
      pingTimeoutRX = rxSettings.pingTimeoutRX;
 
      saveConfig();
      
      server.send(200, "text/html", MSG_HEADER + "<h2>✅ SUCCESS!<br>Rebooting...</h2>" + MSG_FOOTER);
      
      DEBUGln(F("[STATE] TX Rebooting now..."));
      delay(500);
      ESP.restart(); 
    } else {
      server.send(200, "text/html", MSG_HEADER + "<h2>❌ ERROR:<br>RX not responding!</h2>" + MSG_FOOTER);
    } // конец проверки успешной синхронизации
 } // конец функции handleSave
 
 void handleCancel() {
    DEBUGln(F("[WIFI] Received Cancel Request from browser"));
    server.send(200, "text/html", MSG_HEADER + "<h2>🚪 Cancelled.<br>Returning to Normal Mode.</h2>" + MSG_FOOTER);
    exitConfigRequested = true; 
 } // конец функции handleCancel
 
 void startWiFiPortal() {
    DEBUGln(F("[WIFI] Starting WiFi AP (Captive Portal)..."));
    WiFi.mode(WIFI_AP);
    
    String ssidName = "DavayLoRa_" + String(workAddress);
    WiFi.softAP(ssidName.c_str());
    
    delay(100);
    
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/cancel", HTTP_GET, handleCancel);
    
    // ФУНКЦИЯ, КОТОРУЮ НЕЛЬЗЯ УДАЛЯТЬ (Captive Portal Redirect)
    server.onNotFound([]() {
      server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
      server.send(302, "text/plain", "");
    }); // конец обработчика onNotFound
    
    server.begin();
    isWifiActive = true;
    wifiStartTime = millis();
    DEBUGln(F("[WIFI] WebServer started."));
 } // конец функции startWiFiPortal
 
 void stopWiFiPortal() {
    DEBUGln(F("[WIFI] Stopping WiFi..."));
    server.stop();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    isWifiActive = false;
    DEBUGln(F("[WIFI] WiFi Stopped."));
 } // конец функции stopWiFiPortal