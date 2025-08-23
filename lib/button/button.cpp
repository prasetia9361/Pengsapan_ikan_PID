#include "button.h"


button* button::instance = nullptr;

void button::begin(){
    bindingButton.attachDoubleClick(doubleClick);
    // bindingButton.attachLongPressStop(longPress);
    bindingButton.attachLongPressStart(longPress);
    pinMode(_pin,INPUT_PULLUP);
}

void button::doubleClick(){
    if (instance)
    {
        instance->onDoubleClick();
    }
}
void button::onDoubleClick(){
    mode = true;
}

void button::longPress(){
    if (instance)
    {
        instance->onLongPress();
    }
}

void button::onLongPress(){
    pressData = true;
}

void button::release(){
    if (instance)
    {
        instance->onRelease();
    }
}

void button::onRelease(){
    pressData = false;
}

void button::tick(){
    bindingButton.tick();
}

