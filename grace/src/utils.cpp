#include "utils.h"
#include <Arduino.h>

int angleToDutyMG996R(int angle) {
    angle = constrain(angle, 0, 180);
    int pulseWidth = map(angle, 0, 180, 500, 2500);
    return (int) (pulseWidth * 4095) / 20000;
}

int angleToDutyScanner(int angle) {
    angle = constrain(angle, 0, 180);
    int pulseWidth = map(angle, 0, 180, 480, 2500);
    return (int) (pulseWidth * 4095) / 20000;
}

int angleToDutyZ(int angle) {
    int pulseWidth = map(angle, 0, 180, 775, 2100);
    return (int) (pulseWidth * 4095) / 20000;
}

int angleToDutyBase(int angle) {
    int pulseWidth = map(angle, 0, 180, 850, 2075);
    return (int) (pulseWidth * 4095) / 20000;
}

int angleToDutyElbow(int angle) {
    int pulseWidth = map(angle, 90, 180, 1530, 2180);
    return (int) (pulseWidth * 4095) / 20000;
}

void debugPrint(const String &message) {
    // Serial.print(message);
}
  
  void debugPrintln(const String &message) {
    // Serial.println(message);
}
  