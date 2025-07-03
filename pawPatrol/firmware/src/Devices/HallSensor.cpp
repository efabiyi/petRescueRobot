#include <Arduino.h>
#include "hall.h"

const int diffPin = 13;


void setup() {
  Serial.begin(115200);
}

void loop() {

  int sensorValue = analogRead(diffPin);
  int fudgeFactor = 4095 - sensorValue;

  float voltage = fudgeFactor * (3.3 / 4095.0); // Convert ADC value to voltage


  Serial.print("Differential ADC Value: ");
  Serial.println(voltage);
  delay(50); // Delay for readability

}