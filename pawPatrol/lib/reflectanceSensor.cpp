#include <Arduino.h>

const int reflectanceSensorPin = 35; // Reflectance sensor connected to analog pin A0   

void setup() {
  Serial.begin(115200);
}

void loop() {
  int sensorValue = analogRead(35); // Read from analog pin A0
  float voltage = sensorValue * (3.3 / 4095.0); // Convert ADC value to voltage

  //Serial.print("Reflectance Sensor Voltage: ");
  //Serial.println(voltage);
  
  delay(50); // Delay for readability
}