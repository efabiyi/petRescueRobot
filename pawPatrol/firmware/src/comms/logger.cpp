#include <WiFi.h>
#include <HTTPClient.h>
#include "logger.h"
#include "secrets.h"

QueueHandle_t Logger::logQueue;

void Logger::begin() {
  logQueue = xQueueCreate(10, sizeof(String*));  // Small queue, 10 messages max

  xTaskCreatePinnedToCore(
    loggingTask,      // Task function
    "Logger Task",    // Task name
    4096,             // Stack size
    NULL,             // Task parameters
    1,                // Priority
    NULL,             // Task handle
    0                 // Core 0 (freeing core 1 for WiFi etc.)
  );
}

void Logger::loggingTask(void* pvParameters) {
  String* msgPtr = nullptr;

  while (true) {
    // Block until a message is available
    if (xQueueReceive(logQueue, &msgPtr, portMAX_DELAY) == pdPASS) {
      if (msgPtr) {
        send(*msgPtr);
        delete msgPtr; // Free heap
      }
    }
  }
}

void Logger::log(const String& message) {
  if (!logQueue) return;

  String* msgPtr = new String(message);
  if (xQueueSend(logQueue, &msgPtr, 0) != pdPASS) {
    delete msgPtr;  // Drop if queue is full
  }
}

void Logger::send(const String& message) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Dropping log.");
    return;
  }

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "text/plain");

  int httpResponse = http.POST(message);
  Serial.printf("Sent: %s, Response: %d\n", message.c_str(), httpResponse);

  http.end();
}