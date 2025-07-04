#include <WiFi.h>
#include "secrets.h"
#include "potentiometer.h"
#include "logger.h"
#include <hallsensor.h>

Potentiometer pot(33); // GPIO pin for the potentiometer
HallSensor hallSensor(32); // GPIO pin for the hall sensor
#define LED 2

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);


  Serial.print("Connecting to WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED, !digitalRead(LED));  // blink
    attempts++;

  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi. Please check your credentials.");
    return;
  }
  
  Serial.println("\nWiFi connected!");

  pinMode(LED, OUTPUT);
}

void loop() {
  //float potvoltage = pot.readVoltage();
  //String data = "Potentiometer Voltage: " + String(voltage, 2) + " V";
  //Logger::send(data);

  float hallVoltage = hallSensor.readVoltage();
  bool magnetDetected = hallSensor.magnetDetected();
  String hallData = "Hall Sensor Voltage: " + String(hallVoltage, 2) + " V, Magnet Detected: " + (magnetDetected ? "Yes" : "No");
  Logger::send(hallData);

  Serial.println(hallData);
  delay(100);
}
