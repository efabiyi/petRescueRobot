#include <WiFi.h>
#include <HTTPClient.h>
#include "logger.h"
#include "secrets.h"

void Logger::send(const String& message) {

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(SERVER_URL);
    http.addHeader("Content-Type", "text/plain");
    int httpResponse = http.POST(message);
    Serial.printf("Sent: %s, Response: %d\n", message.c_str(), httpResponse);
    http.end();
  } else {
    Serial.println("WiFi not connected, cannot send log");
  }
  
}
