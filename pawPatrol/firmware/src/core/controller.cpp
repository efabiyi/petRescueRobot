
#include <WiFi.h>
#include "Arduino.h"
#include "ESP32Servo.h"

#include "secrets.h"
#include "logger.h"
#include "wifiManager.h"
#include "constants.h"
#include "controller.h"
#include "drive.h"
#include "hallSensor.h"
#include "scanner.h"
#include "lift.h"

int TARGET = 7;

void driveRoutine(){

//State: DRIVE AND SCAN 
    //drive()
    //bool objectDetected = scan() (not wall)

    //if objectDetected {
    // state = SNIFF

//State: SNIFF
    //petDetected = searchPet()

    //if petDetected {

    //state = GRABNPLACE
    //}


}