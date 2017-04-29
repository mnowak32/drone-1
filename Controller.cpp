#include "Controller.h"

boolean isChanged = false, isPressed = false, isSubmitted = false;

Controller::Controller() {
  
}

boolean Controller::changed() {
  boolean retVal = isChanged;
  isChanged = false;
  return retVal;
}

boolean Controller::pressed() {
  boolean retVal = isPressed;
  isPressed = false;
  return retVal;
}

boolean Controller::submitted() {
  boolean retVal = isSubmitted;
  isSubmitted = false;
  return retVal;
}

void Controller::parseMessage(String cmd) {
  char control = cmd.charAt(0);
  if (control == 'L' || control == 'R') {
    isChanged = true;
    int splitAt = cmd.indexOf(',');
    float val1 = cmd.substring(1, splitAt).toFloat();
    float val2 = cmd.substring(splitAt + 1).toFloat();
  
    switch(control) {
      case 'L': yaw = val1; throttle = val2; break;
      case 'R': pitch = val1; roll = val2; break;
      default: break;
    }
  } else if (control == 'P') {
    int splitAt = cmd.indexOf(',');
    int splitAt2 = cmd.indexOf(',', splitAt + 1);
    newP = cmd.substring(1, splitAt).toFloat();
    newI = cmd.substring(splitAt + 1, splitAt2).toFloat();
    newD = cmd.substring(splitAt2 + 1).toFloat();
    isSubmitted = true;
  } else {
//    Serial.println(control);
    isPressed = true;
    button = control;
  }

//  Serial.print("Throttle:"); Serial.print(throttle);
//  Serial.print(",yaw:"); Serial.print(yaw);
//  Serial.print(",pitch:"); Serial.print(pitch);
//  Serial.print(",roll:"); Serial.println(roll);
}

