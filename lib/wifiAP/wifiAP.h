#pragma once
#include <DNSServer.h>
#include <WiFi.h>
class wifiAP {
   private:
    DNSServer dnsServer;

    const char *ssid1 = "myWifiAP";
    const char *password1 = "12345678";
    bool mode = false;
    String macAddress = WiFi.macAddress();

   public:
    // void begin();
    wifiAP();
    // void begin();
    void setup();
    void loopDns();
    void connectToWiFi(const String& ssid, const String& pass);
    void connectAP();
    void connectSTA();
    // String getMac() { return macAddress; }
    void disconnectAP();
    bool getMode(){return mode;}
    bool reconnect(){return WiFi.reconnect();}
};
