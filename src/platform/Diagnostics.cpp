#include "platform/Diagnostics.h"

#include "Config.h"

namespace cnc
{
namespace
{
constexpr auto slowBlinkInterval = Milliseconds{500};
constexpr auto fastBlinkInterval = Milliseconds{150};
constexpr auto pulseWidth = Milliseconds{50};
} // namespace

DiagnosticSignals::DiagnosticSignals() noexcept
{
    for (auto &mode : modes_)
    {
        mode.store(LedMode::Off, std::memory_order_relaxed);
    }
}

void DiagnosticSignals::set(const DiagnosticChannel channel, const LedMode mode) noexcept
{
    modes_[index(channel)].store(mode, std::memory_order_release);
}

LedMode DiagnosticSignals::mode(const DiagnosticChannel channel) const noexcept
{
    return modes_[index(channel)].load(std::memory_order_acquire);
}

bool DiagnosticSignals::consumePulse(const DiagnosticChannel channel) noexcept
{
    auto &signal = modes_[index(channel)];
    LedMode expected = LedMode::Pulse;
    return signal.compare_exchange_strong(expected, LedMode::Off, std::memory_order_acq_rel);
}

DiagnosticLeds::DiagnosticLeds(DiagnosticSignals &signals) noexcept
    : signals_(signals), leds_{{{config::systemLedPin},
                                {config::wifiLedPin},
                                {config::mqttLedPin},
                                {config::modbusLedPin}}}
{
}

void DiagnosticLeds::begin() noexcept
{
    for (std::size_t i = 0U; i < static_cast<std::size_t>(DiagnosticChannel::Count); ++i)
    {
        pinMode(leds_[i].pin, OUTPUT);
        write(leds_[i], false);
    }
}

void DiagnosticLeds::update(const MonotonicTimePoint now) noexcept
{
    for (std::size_t i = 0U; i < static_cast<std::size_t>(DiagnosticChannel::Count); ++i)
    {
        updateOne(static_cast<DiagnosticChannel>(i), leds_[i], now);
    }
}

void DiagnosticLeds::updateOne(const DiagnosticChannel channel, LedRuntime &led,
                               const MonotonicTimePoint now) noexcept
{
    if (signals_.consumePulse(channel))
    {
        led.pulseActive = true;
        led.pulseStarted = now;
        write(led, true);
    }

    if (led.pulseActive)
    {
        if (now - led.pulseStarted >= pulseWidth)
        {
            led.pulseActive = false;
            write(led, false);
        }
        return;
    }

    const LedMode requested = signals_.mode(channel);
    if (requested != led.activeMode)
    {
        led.activeMode = requested;
        led.lastTransition = now;
        if (requested == LedMode::Off)
        {
            write(led, false);
        }
        else if (requested == LedMode::On)
        {
            write(led, true);
        }
    }

    const Milliseconds interval = requested == LedMode::BlinkSlow   ? slowBlinkInterval
                                  : requested == LedMode::BlinkFast ? fastBlinkInterval
                                                                    : Milliseconds{0};
    if (interval != Milliseconds{0} && now - led.lastTransition >= interval)
    {
        write(led, !led.outputHigh);
        led.lastTransition = now;
    }
}

void DiagnosticLeds::write(LedRuntime &led, const bool high) noexcept
{
    digitalWrite(led.pin, high ? HIGH : LOW);
    led.outputHigh = high;
}

} // namespace cnc
