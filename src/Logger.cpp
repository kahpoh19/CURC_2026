#include "Logger.h"

#include <stdarg.h>
#include <stdio.h>

void logPrint(const char *message) {
  Serial.print(message);
  Serial0.print(message);
}

void logPrintln() {
  Serial.println();
  Serial0.println();
}

void logPrintln(const char *message) {
  Serial.println(message);
  Serial0.println(message);
}

void logPrintf(const char *format, ...) {
  char buffer[160];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  logPrint(buffer);
}

void waitForSerialMonitor(uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (!Serial && millis() - start < timeoutMs) {
    delay(10);
  }
}
