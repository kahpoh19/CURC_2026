#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

void logPrint(const char *message);
void logPrintln();
void logPrintln(const char *message);
void logPrintf(const char *format, ...);
void waitForSerialMonitor(uint32_t timeoutMs);

#endif
