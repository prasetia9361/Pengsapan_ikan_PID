#ifndef STORAGE_H
#define STORAGE_H

#include <SPIFFS.h>

#include <ArduinoJson.h>

#include <freertos/FreeRTOS.h>
#include "FS.h"

class storage {
private:
    typedef struct PID{
        double Kp;
        double Ki;
        double Kd;
        double calibrration[3] = {3.06 , 0.03 , 0.27};
    }PID;

    typedef struct wifi
    {
        String ssid;
        String pass;
    }wifi;
    wifi setWifi;

    PID dataPID;

    SemaphoreHandle_t semaphore;
    
public:
    storage();
    ~storage();

    double getKp(){return dataPID.calibrration[0];}
    double getKi(){return dataPID.calibrration[1];}
    double getKd(){return dataPID.calibrration[2];}
    double *getPID(){return dataPID.calibrration;}
    void readWifi();
    String getSsid() { return setWifi.ssid; }
    String getPass() {return setWifi.pass;}
    
    void init();
    void saveKp(double Kp);
    void saveKi(double Ki);
    void saveKd(double Kd);
    void savePID(double kp, double ki, double kd);
    void writeWifi(const String& ssid, const String& pass);
};
#endif