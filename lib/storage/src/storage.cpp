#include "storage.h"

storage::storage() {
}

storage::~storage() {
}

void storage::init() {
    Serial.println("[DEBUG] Memulai inisialisasi storage");
    
    // Tidak perlu ambil semaphore

    if (!SPIFFS.begin(true)) {
        Serial.println("[ERROR] SPIFFS init failed");
        return;
    }
    Serial.println("[DEBUG] SPIFFS berhasil diinisialisasi");

    // Handle mode.json
    if (SPIFFS.exists("/dataPID.json")) {
        Serial.println("[DEBUG] File dataPID.json ditemukan");
        File file = SPIFFS.open("/dataPID.json", FILE_READ);
        if (file) {
            JsonDocument doc;
            if (deserializeJson(doc, file) == DeserializationError::Ok) {
                Serial.println("[DEBUG] JSON parsing dataPID.json berhasil");
                // dataPID.Kp = doc["Kp"];
                // Serial.printf("[DEBUG] Kp yang dibaca: %d\n", dataPID.Kp);

                // dataPID.Ki = doc["Ki"];
                // Serial.printf("[DEBUG] Ki yang dibaca: %d\n", dataPID.Ki);

                // dataPID.Kd = doc["Kd"];
                // Serial.printf("[DEBUG] Kd yang dibaca: %d\n", dataPID.Kd);

                JsonArray pid = doc["pid"];
                Serial.printf("[DEBUG] Jumlah data yang ditemukan: %d\n", pid.size());
                
                for (int i = 0; i < 8 && i < pid.size(); i++) {
                    dataPID.calibrration[i] = pid[i];
                    Serial.printf("[DEBUG] Mode[%d] = %d\n", i, dataPID.calibrration[i], 2);
                }
            } else {
                Serial.println("[ERROR] Gagal parsing JSON dataPID.json");
            }
            file.close();
        }
    } else {
        Serial.println("[DEBUG] File dataPID.json tidak ditemukan");
    }

    // Tidak perlu xSemaphoreGive
    Serial.println("[DEBUG] Inisialisasi storage selesai");
}

void storage::readWifi(){

    // Membaca data WiFi dari wifi.json
 if (SPIFFS.exists("/wifi.json")) {
     JsonDocument wifiDoc;
     File wifiFile = SPIFFS.open("/wifi.json", FILE_READ);
     
     if (!wifiFile) {
         Serial.println("Gagal membuka file wifi.json untuk dibaca");
         setWifi.ssid = "";
         setWifi.pass = "";
         return;
     }
     
     char wifiStr[256];
     wifiFile.readBytes(wifiStr, wifiFile.size());
     wifiFile.close();
     
     DeserializationError wifiError = deserializeJson(wifiDoc, wifiStr);
     if (wifiError) {
         Serial.print("deserializeJson() untuk wifi.json returned ");
         Serial.println(wifiError.c_str());
         setWifi.ssid = "";
         setWifi.pass = "";
         return;
     }
     
     setWifi.ssid = wifiDoc["ssid"].as<String>();
     setWifi.pass = wifiDoc["pass"].as<String>();
     
     Serial.println("Data WiFi berhasil dibaca");
     Serial.print("SSID: ");
     Serial.println(setWifi.ssid);
     Serial.print("Password: ");
     Serial.println(setWifi.pass);
 } else {
     Serial.println("File wifi.json tidak ditemukan");
     setWifi.ssid = "";
     setWifi.pass = "";
 }

}

void storage::saveKp(double Kp){
    JsonDocument doc;

    File fileRead = SPIFFS.open("/dataPID.json", FILE_READ);
    if (fileRead) {
        deserializeJson(doc, fileRead);
        fileRead.close();
    }

    File file = SPIFFS.open("/dataPID.json", FILE_WRITE);
    if (!file) {
        Serial.println("- gagal membuka file untuk menulis mode");
        return;
    }

    // Karena bufferMode bertipe uint8_t*, kita ambil 1 byte per mode
    doc["Kp"] = Kp;
    dataPID.Kp = Kp;

    if (serializeJson(doc, file) == 0) {
        Serial.println("- gagal menulis JSON ke file mode.json");
    } else {
        Serial.println("Mode berhasil disimpan ke SPIFFS");
    }
    file.close();
}

void storage::saveKi(double Ki){
    JsonDocument doc;

    File fileRead = SPIFFS.open("/dataPID.json", FILE_READ);
    if (fileRead) {
        deserializeJson(doc, fileRead);
        fileRead.close();
    }

    File file = SPIFFS.open("/dataPID.json", FILE_WRITE);
    if (!file) {
        Serial.println("- gagal membuka file untuk menulis mode");
        return;
    }

    // Karena bufferMode bertipe uint8_t*, kita ambil 1 byte per mode
    doc["Ki"] = Ki;
    dataPID.Ki = Ki;

    if (serializeJson(doc, file) == 0) {
        Serial.println("- gagal menulis JSON ke file mode.json");
    } else {
        Serial.println("Mode berhasil disimpan ke SPIFFS");
    }
    file.close();
}

void storage::saveKd(double Kd){
    JsonDocument doc;

    File fileRead = SPIFFS.open("/dataPID.json", FILE_READ);
    if (fileRead) {
        deserializeJson(doc, fileRead);
        fileRead.close();
    }

    File file = SPIFFS.open("/dataPID.json", FILE_WRITE);
    if (!file) {
        Serial.println("- gagal membuka file untuk menulis mode");
        return;
    }

    // Karena bufferMode bertipe uint8_t*, kita ambil 1 byte per mode
    doc["Kd"] = Kd;
    dataPID.Kd = Kd;

    if (serializeJson(doc, file) == 0) {
        Serial.println("- gagal menulis JSON ke file mode.json");
    } else {
        Serial.println("Mode berhasil disimpan ke SPIFFS");
    }
    file.close();
}

void storage::savePID(double kp, double ki, double kd){
    JsonDocument doc;

    File fileRead = SPIFFS.open("/dataPID.json", FILE_READ);
    if (fileRead) {
        deserializeJson(doc, fileRead);
        fileRead.close();
    }

    File file = SPIFFS.open("/dataPID.json", FILE_WRITE);
    if (!file) {
        Serial.println("- gagal membuka file untuk menulis mode");
        return;
    }

    doc["Kp"] = kp;
    dataPID.calibrration[0] = kp;
    doc["Ki"] = ki;
    dataPID.calibrration[1] = ki;
    doc["Kd"] = kd;
    dataPID.calibrration[2] = kd;

    if (serializeJson(doc, file) == 0) {
        Serial.println("- gagal menulis JSON ke file mode.json");
    } else {
        Serial.println("Mode berhasil disimpan ke SPIFFS");
    }
    file.close();
}

void storage::writeWifi(const String& ssid, const String& pass){
    File file = SPIFFS.open("/wifi.json", FILE_WRITE);
    if (!file) {
        Serial.println("- gagal membuka file untuk penulisan");
        return;
    }
    
    JsonDocument doc;
    doc["ssid"] = ssid;
    doc["pass"] = pass;
    
    serializeJson(doc, file);
    file.close();
    
    setWifi.ssid = ssid;
    setWifi.pass = pass;
    
    Serial.println("Data WiFi berhasil disimpan");
}



