#pragma once
// #include <DNSServer.h>
#include <ESPAsyncWebServer.h>
// #include <SPIFFS.h>

// #include "FS.h"
// #include "SD.h"
#include "storage.h"
// class dataJson;
class wifiAP;
class mcpNode;
class clientServer {
   private:
    AsyncWebServer server;
    storage *memory;

    String ssid, password;
    int node;
    bool connect = false;
    // remote
    void indexHtml();
    void address();

   public:
    // comunication(dataSpiffs &Spiffs,);
    clientServer(storage *_memory) : server(80){
        memory = _memory;
    }
    void setup();
    void notFound();
    void init();
    bool getConneect(){return connect;}
    String getSsid() { return ssid; }
    String getPass() {return password;}
    // void dns(const IPAddress &localIP);

    // void dnsInterval();
};