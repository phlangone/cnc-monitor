/**
 * @file Led.h
 * @brief Declaração da classe de controle de LED.
 */
#pragma once

#include <Arduino.h>
#include <rtos.h>
#include <chrono>

/**
 * @brief Operating modes supported by an LED.
 */
enum class LedState
{
    Off,        /**< LED continuously off. */
    On,         /**< LED continuously on. */
    BlinkSlow,  /**< LED blinking at a slow rate. */
    BlinkFast,  /**< LED blinking at a fast rate. */
    Pulse       /**< LED activated for a single pulse. */
};

/**
 * @brief Controls an LED with support for continuous and blinking modes.
 *
 * The desired state can be changed safely from different threads.
 * The update() method must be called periodically by the LED control thread.
 */
class Led
{
public:
    /**
     * @brief Creates an LED initially turned off.
     *
     * @param pin Arduino pin connected to the LED.
     */
    explicit Led(uint8_t pin)
        : Led{pin, LedState::Off}
    {
    }

    /**
     * @brief Creates an LED with the specified initial state.
     *
     * @param pin Arduino pin connected to the LED.
     * @param state Initial operating state.
     */
    Led(uint8_t pin, LedState state)
        : pin_{pin},
          state_{state},
          lastToggleTime_{}
    {
    }

    /**
     * @brief Initializes the GPIO pin used by the LED.
     *
     * This method must be called after the Arduino core has been initialized.
     */
    void begin();

    /**
     * @brief Changes the desired operating state.
     *
     * This method is thread-safe.
     *
     * @param state New operating state.
     */
    void setState(LedState state);

    /**
     * @brief Returns the current desired operating state.
     *
     * This method is thread-safe.
     *
     * @return Current LED state.
     */
    LedState state() const;

    /**
     * @brief Updates the physical LED output.
     *
     * This method applies the current state and handles the timing of
     * blinking modes. It must be called periodically by a control thread.
     */
    void update();

private:
    uint8_t pin_;                                  /**< GPIO pin connected to the LED. */
    LedState state_;                               /**< Current desired operating state. */
    rtos::Kernel::Clock::time_point lastToggleTime_; /**< Time of the last output transition. */

    /**
     * @brief Protects access to the shared LED state.
     *
     * Declared mutable so it can be locked by the const state() method.
     */
    mutable rtos::Mutex mutex_;

    static constexpr auto kBlinkSlowInterval{
        std::chrono::milliseconds{500}
    }; /**< Slow blinking transition interval. */

    static constexpr auto kBlinkFastInterval{
        std::chrono::milliseconds{150}
    }; /**< Fast blinking transition interval. */
};