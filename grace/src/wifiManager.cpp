#include <Arduino.h>
#include "wifiManager.h"
#include "secrets.h"

WifiManager::WifiManager(){
    //Constructor for wifiManager
    Serial.println("WifiManager intitialized");

}

void  WifiManager::startWifi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(1000);
    Serial.print(".");
    attempts++;

  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi. Please check your credentials.");
    return;
  }
  
  Serial.println("\nWiFi connected!");

}