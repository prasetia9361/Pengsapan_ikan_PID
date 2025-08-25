#ifndef STORAGE_H
#define STORAGE_H

#include <SPIFFS.h>

#include <ArduinoJson.h>

#include <freertos/FreeRTOS.h>
#include "FS.h"

class storage {
private:
    typedef struct config {
        uint8_t macAddress[6] = {0,0,0,0,0,0};
        uint8_t macAddress1[6] = {0,0,0,0,0,0};
        int32_t modeArray[8] = {0,0,0,0,0,0,0,0};
        char nameDevice1[12] = "";
        char nameDevice2[12] = "";
        int dataInt;
    } config;

    typedef struct PID{
        double Kp;
        double Ki;
        double Kd;
        double calibrration[3] = {3.06 , 0.03 , 0.27};
    }PID;

    config configData;

    PID dataPID;

    SemaphoreHandle_t semaphore;
    
public:
    storage();
    ~storage();

    double getKp(){return dataPID.calibrration[0];}
    double getKi(){return dataPID.calibrration[1];}
    double getKd(){return dataPID.calibrration[2];}
    double *getPID(){return dataPID.calibrration;}
    
    void init();
    void saveKp(double Kp);
    void saveKi(double Ki);
    void saveKd(double Kd);
    void savePID(double kp, double ki, double kd);
};
#endif