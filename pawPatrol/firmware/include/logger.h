#pragma once
#include <Arduino.h>

class Logger {
  public:
    static void send(const String& message);
};
