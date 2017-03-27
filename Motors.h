#ifndef Drone_h
#define Drone_h

#include <Arduino.h>
#include <stdint.h>

#define MAX_SPEED 180

class Motors {
  public:
    Motors(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4);
    void setSpeed(uint16_t s);
    void setSpeeds(uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4);

  private:
    uint8_t p1, p2, p3, p4;
    void setSingleSpeed(uint8_t pin, uint16_t v);
};

#endif
