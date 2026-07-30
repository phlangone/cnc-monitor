#include <stdio.h>
#include <stdarg.h>
#include <ScopedLock.h>

#include "Logger.h"

void Logger::begin(unsigned long baudRate)
{
    serial_.begin(baudRate);
}

void Logger::setEnabled(bool enabled)
{
    enabled_ = enabled;
}

void Logger::printf(const char* fmt, ...)
{
    if (!enabled_)
    {
        return;
    }

    char buffer[128];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    mbed::ScopedLock<rtos::Mutex> lock{mutex_};

    serial_.println(buffer);
}