#include "utils.h"
#include <Arduino.h>

int angleToDutyMG996R(int angle) {
    angle = constrain(angle, 0, 180);
    int pulseWidth = map(angle, 0, 180, 500, 2500);
    return (int) (pulseWidth * 4095) / 20000;
}

int angleToDuty25kgcm(int angle) {
    angle = constrain(angle, 0, 180);
    int pulseWidth = map(angle, 0, 180, 750, 2250);
    return (int) (pulseWidth * 4095) / 20000;
}

void debugPrint(const String &message) {
    // Serial.print(message);
}
  
  void debugPrintln(const String &message) {
    // Serial.println(message);
}
  