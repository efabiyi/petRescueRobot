#pragma once
#include <Arduino.h>
#include <WiFi.h>

class WifiManager {
    private:
   
    public:
    WifiManager();
    void startWifi(); 
    bool isConnected();
};