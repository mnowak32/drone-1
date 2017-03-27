#include "Motors.h"


Motors::Motors(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4): p1(p1), p2(p2), p3(p3), p4(p4) {
  analogWriteRange(MAX_SPEED);
  pinMode(p1, OUTPUT);
  pinMode(p2, OUTPUT);
  pinMode(p3, OUTPUT);
  pinMode(p4, OUTPUT);
  setSpeed(0);
}

void Motors::setSpeed(uint16_t spd) {
  setSpeeds(spd, spd, spd, spd);
}

void Motors::setSpeeds(uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4) {
//  Serial.print(s1); Serial.print(",");
//  Serial.print(s2); Serial.print(",");
//  Serial.print(s3); Serial.print(",");
//  Serial.println(s4);
  setSingleSpeed(p1, s1);
  setSingleSpeed(p2, s2);
  setSingleSpeed(p3, s3);
  setSingleSpeed(p4, s4);
}

void Motors::setSingleSpeed(uint8_t pin, uint16_t v) {
  if (v > MAX_SPEED) { v = MAX_SPEED; }
  if (v < 0) { v = 0; }
  analogWrite(pin, v);
}


