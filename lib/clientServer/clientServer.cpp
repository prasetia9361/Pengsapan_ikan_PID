#include "clientServer.h"

void clientServer::setup() {
    indexHtml();
    address();
    // notFound();
    // init();
}

void clientServer::indexHtml() {
    server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
}

void clientServer::address() {
    server.on("/inputAddress", HTTP_GET,
              [this](AsyncWebServerRequest *request) {
                  if (request->hasParam("ssid") && request->hasParam("password")) {
                      ssid = request->getParam("ssid")->value();
                      password = request->getParam("password")->value();
                      memory->writeWifi(ssid,password);
                      connect = true;
                      // Di sini kamu bisa tambahkan kode untuk menghubungkan ke WiFi
                      // menggunakan SSID dan password yang diterima
                      // Contoh: wifi->connect(mac, password);
                      Serial.print("ssid:");
                      Serial.println(ssid);
                      Serial.print("pass:");
                      Serial.println(password);
                      Serial.println("berhasil menyimpan ssid");
                      
                      request->send(200, "text/plain", "berhasil");
                  } else {
                    Serial.println("gagal menyimpan");
                      request->send(400, "text/plain",
                                    "Parameter SSID atau password tidak ditemukan.");
                  }
              });
}

void clientServer::notFound() {
    const String localUrl = "http://192.168.7.2";
    server.onNotFound([this, localUrl](AsyncWebServerRequest *request) {
        request->redirect(localUrl);
        Serial.print("onnotfound ");
        Serial.print(
            request->host());  // This gives some insight into whatever was
                               // being requested on the serial monitor
        Serial.print(" ");
        Serial.print(request->url());
        Serial.print(" sent redirect to " + localUrl + "\n");
    });
}
void clientServer::init() {
    server.serveStatic("/", SPIFFS, "/");
    server.begin();
    connect = false;
}
