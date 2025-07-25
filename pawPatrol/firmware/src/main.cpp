#include <WiFi.h>
#include "hallsensor.h"
#include "secrets.h"
#include "logger.h"
#include "wifiManager.h"
#include "drive.h"
#include "detect.h"
#include "scanner.h"

Logger logger;
WifiManager wifiMgr;

HallSensor hallSensor(logger);
Drive driver(logger);
Scanner scanner(logger);

void setup() {
  Serial.begin(115200);
  wifiMgr.startWifi();
  logger.begin(); // Launch logger task (on secibd core!)
  driver.initializeDrive();
}

void loop() {
  driver.drive();
  hallSensor.sense();
}
