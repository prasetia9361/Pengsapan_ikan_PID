#pragma once

#include "OneButton.h"
// #include "config.h"

class button
{
private:
    bool mode;
    bool pressData;
    int _pin;
    OneButton bindingButton;
    static button* instance;
    static void doubleClick();
    static void longPress();
    static void release();
public:
    button(int pin):_pin(pin),bindingButton(_pin, true){
        mode = false;
        pressData = false;
        instance = this;
    }
    void begin();
    void onDoubleClick();
    void onLongPress();
    void onRelease();
    bool getMode(){return mode;}
    bool setMode(bool value){
        mode = value;
        return mode;
    }
    bool getPress(){return pressData;}
    bool setPress(bool value){
        pressData = value;
        return pressData;
    }
    void tick();
};