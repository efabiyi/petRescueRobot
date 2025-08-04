#include <WiFi.h>
#include <HTTPClient.h>
#include "logger.h"
#include "secrets.h"

QueueHandle_t Logger::logQueue;

void Logger::begin() {
  logQueue = xQueueCreate(3, sizeof(String*));
  Serial.print("reached");
  xTaskCreatePinnedToCore(
    loggingTask,
    "Logger Task",
    4096,
    NULL,
    1,
    NULL,
    0
  );
}

void Logger::loggingTask(void* pvParameters) {
  while (true) {
    //Serial.print("reached");
    //Serial.printf("LOG: Running loop() on core %d\n", xPortGetCoreID());
    String batch = "";

    // Drain all messages from the queue
    String* msgPtr;
    while (xQueueReceive(logQueue, &msgPtr, 0) == pdPASS) {
      batch += *msgPtr + "\n";
      delete msgPtr; // Free heap memory
    }

    if (batch.length() > 0) {
      send(batch);
    }

    vTaskDelay(pdMS_TO_TICKS(500)); // Wait 1s before next send
  }
}

void Logger::log(const String& message) {
  if (!logQueue) return;

  String* msgPtr = new String(message);
  if (xQueueSend(logQueue, &msgPtr, 0) != pdPASS) {
    delete msgPtr; // Queue full, discard safely
  }
}

void Logger::send(const String& message) {
  if (WiFi.status() != WL_CONNECTED) {
      //Serial.println("WiFi not connected, can't send log.");
    return;
  }

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "text/plain");

  int httpResponse = http.POST(message);
    Serial.printf("Sent: %s, Response: %d\n", message.c_str(), httpResponse);

  http.end();
}
