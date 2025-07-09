#ifndef DETECT_H
#define DETECT_H

struct ScanResult {
    int distance;
    int angle;
    bool valid;
};

void initiateDetectionSystem();
bool scanForPets();

#endif