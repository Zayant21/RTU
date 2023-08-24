#ifndef AnalogKeypad_h
#define AnalogKeypad_h

#include "Arduino.h"

class AnalogKeypad {
public:
  AnalogKeypad(int analogPin) {
    _analogPin = analogPin;
    pinMode(_analogPin, INPUT);
  }

  int getPressedKey() {
    int analogValue = analogRead(_analogPin);
    //Serial.println(analogValue);
    delay(125);

    if (analogValue >= 0 && analogValue <= 30) {
      return 1; // Button 1 pressed
    } else if (analogValue >= 31 && analogValue <= 80) {
      return 2; // Button 2 pressed
    } else if (analogValue >= 81 && analogValue <= 150) {
      return 3; // Button 3 pressed
    } else if (analogValue >= 151 && analogValue <= 300) {
      return 4; // Button 4 pressed
    } else if (analogValue >= 301 && analogValue <= 500) {
      return 5; // Button 5 pressed
    } else {
      return 0; // No button pressed
    }
  }

private:
  int _analogPin;
};

#endif
