#include <WiFi.h>
#include "hallsensor.h"
#include "secrets.h"
#include "logger.h"
#include "wifiManager.h"
#include "drive.h"
#include "detect.h"

unsigned long lastLogTimeHall = 0;
unsigned long lastLogTimeDrive = 0;
const unsigned long hallLogInterval = 1000; //

HallSensor hallSensor(32); // GPIO pin for the hall sensor
WifiManager wifiMgr;

void setup() {
  Serial.begin(115200);
  wifiMgr.startWifi();
  initializeDrive();
}

void loop() {
  String driveData = drive();
  String hallData = hallSensor.sense();

  //logging
  unsigned long currentTime = millis();
  if (currentTime - lastLogTimeHall>= hallLogInterval) {
    lastLogTimeHall = currentTime;
    Logger::send(driveData + "\n" + hallData);
  }

}


