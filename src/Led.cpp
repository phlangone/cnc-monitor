/**
 * @file Led.cpp
 * @brief Implementação da classe de controle de LED.
 */
#include <ScopedLock.h>

#include "Led.h"

void Led::begin()
{
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, state() == LedState::On ? HIGH : LOW);

    lastToggleTime_ = rtos::Kernel::Clock::now();
}

void Led::setState(LedState state)
{
    mbed::ScopedLock<rtos::Mutex> lock{mutex_};
    state_ = state;
}

LedState Led::state() const
{
    mbed::ScopedLock<rtos::Mutex> lock{mutex_};
    return state_;
}

void Led::update()
{
    const auto now = rtos::Kernel::Clock::now();
    const LedState currentState = state();

    switch (currentState)
    {
        case LedState::Off:
            digitalWrite(pin_, LOW);
            break;

        case LedState::On:
            digitalWrite(pin_, HIGH);
            break;

        case LedState::BlinkSlow:
            if ((now - lastToggleTime_) >= kBlinkSlowInterval)
            {
                digitalWrite(pin_, !digitalRead(pin_));
                lastToggleTime_ = now;
            }
            break;

        case LedState::BlinkFast:
            if ((now - lastToggleTime_) >= kBlinkFastInterval)
            {
                digitalWrite(pin_, !digitalRead(pin_));
                lastToggleTime_ = now;
            }
            break;

        case LedState::Pulse:
            digitalWrite(pin_, HIGH);
            lastToggleTime_ = now;
            setState(LedState::Off);
            break;
    }
}
