#include "wifiAP.h"

wifiAP::wifiAP() {}

void wifiAP::setup() {
    IPAddress apIP(192, 168, 7, 2);
    IPAddress subNet(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, apIP, subNet);
    Serial.println(macAddress);
    WiFi.softAP(ssid1, password1, 6, 0, 4);
    
    dnsServer.setTTL(3600);
    dnsServer.start(53, "*", apIP);
    Serial.println("Mode AP aktif dengan IP: " + WiFi.softAPIP().toString());
}

void wifiAP::connectToWiFi(const String& ssid, const String& pass){
    WiFi.begin(ssid.c_str(), pass.c_str());

    Serial.print("Menghubungkan ke WiFi");
    int retry = 20; // 10 detik timeout
    while (WiFi.status() != WL_CONNECTED && retry > 0) {
        Serial.print(".");
        retry--;
        // if (retry == 0)
        // {
        //     if (WiFi.status() == WL_NO_SSID_AVAIL || WiFi.status() ==  WL_CONNECT_FAILED || WiFi.status() == WL_CONNECTION_LOST || WiFi.status() == WL_DISCONNECTED) {
        //         mode = false;
        //         connectAP();
        //     }
        // }
        delay(500);
    }
    
    if (WiFi.status() != WL_CONNECTED && retry == 0) {
        mode = false;
        connectAP();
    } else if(WiFi.status() == WL_CONNECTED){
        mode = true;
        Serial.println("\nTerhubung ke WiFi. IP: " + WiFi.localIP().toString());
    }
}

void wifiAP::connectAP(){
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
}

void wifiAP::connectSTA(){
    WiFi.disconnect();
    // WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
}

void wifiAP::disconnectAP(){
    if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
        WiFi.softAPdisconnect(true);
        Serial.println("Access Point dinonaktifkan");
    }
}

void wifiAP::loopDns() {
    if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
        dnsServer.processNextRequest();
    }
}
