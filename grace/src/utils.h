#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

struct PolarPoint {
    int distance;
    int angle;
};

struct CartesianPoint {
    int x;
    int y;
};

int angleToDutyMG996R(int angle);
int angleToDuty25kgcm(int angle);

void debugPrint(const String &message);
void debugPrintln(const String &message);

#endif // UTILS_H