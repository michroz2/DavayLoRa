#ifndef WEBPORTAL_H
#define WEBPORTAL_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

// Глобальные объекты и флаги WiFi, нужные в main.cpp
extern WebServer server;
extern DNSServer dnsServer;
extern bool isWifiActive;
extern unsigned long wifiStartTime;
extern bool exitConfigRequested;

// Внешние зависимости (функции радио из main.cpp)
extern void sendMessage(byte msgCmd, byte sndData);
extern bool syncConfigToRX();

// Прототипы функций веб-портала
void startWiFiPortal();
void stopWiFiPortal();
void handleRoot();
void handleSave();
void handleCancel();

#endif