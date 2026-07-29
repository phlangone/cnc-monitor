#pragma once

/**
 * Arduino header
 */
#include <Arduino.h>

/**
 * RTOS Mbed
 */
#include <rtos.h>

class Logger
{
public:
    explicit Logger(HardwareSerial& serial)
        : serial_{serial} {}

    void begin(unsigned long baudRate);
    void setEnabled(bool enabled);
    void printf(const char *fmt, ...);

private:
    HardwareSerial& serial_;
    rtos::Mutex mutex_;
    bool enabled_{false};
};