/**
 * @file Logger.h
 * @brief Declaração da classe de registro de mensagens.
 */
#pragma once

#include <Arduino.h>
#include <rtos.h>

/**
 * @brief Thread-safe serial logger.
 */
class Logger
{
public:
    /**
     * @brief Creates a logger for the specified serial interface.
     *
     * @param serial Serial interface used for output.
     */
    explicit Logger(HardwareSerial& serial)
        : serial_{serial} {}

    /**
     * @brief Initializes the serial interface.
     *
     * @param baudRate Serial communication baud rate.
     */
    void begin(unsigned long baudRate);

    /**
     * @brief Enables or disables log output.
     *
     * @param enabled True to enable logging.
     */
    void setEnabled(bool enabled);

    /**
     * @brief Prints a formatted log message.
     *
     * @param fmt Format string.
     */
    void printf(const char* fmt, ...);

private:
    HardwareSerial& serial_;
    rtos::Mutex mutex_;
    bool enabled_{false};
};