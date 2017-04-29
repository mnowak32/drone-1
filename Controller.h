#ifndef Controller_h
#define Controller_h

#include <Arduino.h>
#include <stdint.h>

class Controller {
  public:
    Controller();
    float throttle;
    float pitch;
    float yaw;
    float roll;
    char button;
    float newP, newI, newD;
    boolean changed();
    boolean pressed();
    boolean submitted();
    

    void parseMessage(String msg);
};

#endif

