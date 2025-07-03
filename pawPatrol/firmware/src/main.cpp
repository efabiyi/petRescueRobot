#include <WiFi.h>
#include "secrets.h"
#include "potentiometer.h"
#include "logger.h"

Potentiometer pot(33); // GPIO pin for the potentiometer
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
  float voltage = pot.readVoltage();
  String data = "Potentiometer Voltage: " + String(voltage, 2) + " V";
  Logger::send(data);
  delay(100);
}
