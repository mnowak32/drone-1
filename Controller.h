#ifndef Controller_h
#define Controller_h

#include <Arduino.h>
#include <stdint.h>

class Controller {
  public:
    Controller();
    int8_t throttle;
    int8_t pitch;
    int8_t yaw;
    int8_t roll;
    char button;
    boolean changed();
    boolean pressed();

    void parseMessage(String msg);
};

#endif

