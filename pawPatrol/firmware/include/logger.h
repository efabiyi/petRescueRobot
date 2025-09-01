#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

class Logger {
public:
  static void begin();
  static void log(const String& message);
  static void send(const String& batchedLogs);
  static void logState(State newState);

private:
  static void loggingTask(void* pvParameters);
  static QueueHandle_t logQueue;
};

#endif
