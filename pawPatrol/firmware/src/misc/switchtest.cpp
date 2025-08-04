/*#include <Arduino.h>

#include "secrets.h"
#include "driver/ledc.h"
#include "wifiManager.h"
#include "logger.h"
#include "constants.h"

#include "drive.h"
#include "hallsensor.h"
#include "scanner.h"
#include "claw.h"

const int IN_RANGE_DISTANCE = 300;
const int REACHABLE_DISTANCE = 100;
const int BASE_SPEED = 1300;
const int THRESHOLD = 0;
const int COOL_DOWN = 0;

Logger logger;
WifiManager wifiMgr;  


bool clawSwitchPressed = false;
bool liftSwitchPressed = false;


void setup() {
  Serial.begin(115200);
  pinMode(CLAW_LIMIT_PIN, INPUT_PULLUP);
  pinMode(LIFT_LIMIT_PIN, INPUT_PULLUP);

}


void loop(){
  clawSwitchPressed = digitalRead(CLAW_LIMIT_PIN);
  liftSwitchPressed = digitalRead(LIFT_LIMIT_PIN);


  String S1 ="S1: Unpressed, "; 
  String S2 = "S2: Unpressed";
  if(clawSwitchPressed){
    S1 ="S1: pressed, ";
  }

    if(liftSwitchPressed){
    S2 ="S2: pressed";
  }

  Serial.println(S1+S2);


}

/*
#include <Arduino.h>

const int PWM_CHANNEL = 0;
const int PIN = 25;

void setup() {
  Serial.begin(115200);
  ledcSetup(PWM_CHANNEL, 250, 12);
  ledcAttachPin(PIN, PWM_CHANNEL);
}

void loop() {
  Serial.print("Motor running");
  ledcWrite(PWM_CHANNEL, 1000);
  delay(4000);

  Serial.print("Motor Stopped");
  ledcWrite(PWM_CHANNEL, 0);
  delay(15000);
}*/
