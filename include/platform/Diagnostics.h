/**
 * @file Diagnostics.h
 * @brief Declara a sinalização concorrente e o acionamento dos LEDs.
 */

#pragma once

#include "domain/Time.h"

#include <Arduino.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace cnc
{

/** @brief Identifica cada canal físico de diagnóstico. */
enum class DiagnosticChannel : std::uint8_t
{
    System,
    Wifi,
    Mqtt,
    Modbus,
    Count
};

/** @brief Define o comportamento visual solicitado para um LED. */
enum class LedMode : std::uint8_t
{
    Off,
    On,
    BlinkSlow,
    BlinkFast,
    Pulse
};

/** @brief Compartilha solicitações de LED sem expor os GPIOs às tarefas. */
class DiagnosticSignals final
{
  public:
    /** @brief Inicializa todos os canais no modo desligado. */
    DiagnosticSignals() noexcept;

    /** @brief Define o modo desejado para um canal. */
    void set(DiagnosticChannel channel, LedMode mode) noexcept;
    /** @brief Consulta o modo atual de um canal. */
    LedMode mode(DiagnosticChannel channel) const noexcept;
    /** @brief Consome atomicamente uma solicitação de pulso. */
    bool consumePulse(DiagnosticChannel channel) noexcept;

  private:
    static constexpr std::size_t index(DiagnosticChannel channel) noexcept
    {
        return static_cast<std::size_t>(channel);
    }

    std::array<std::atomic<LedMode>, static_cast<std::size_t>(DiagnosticChannel::Count)> modes_{};
};

/** @brief Único componente autorizado a escrever nos GPIOs dos LEDs. */
class DiagnosticLeds final
{
  public:
    /** @brief Associa o driver ao conjunto compartilhado de sinais. */
    explicit DiagnosticLeds(DiagnosticSignals &signals) noexcept;

    /** @brief Configura os pinos e força todos os LEDs ao estado desligado. */
    void begin() noexcept;
    /** @brief Atualiza padrões temporizados sem bloqueio. */
    void update(MonotonicTimePoint now) noexcept;

  private:
    struct LedRuntime
    {
        pin_size_t pin;
        LedMode activeMode{LedMode::Off};
        MonotonicTimePoint lastTransition{};
        MonotonicTimePoint pulseStarted{};
        bool outputHigh{false};
        bool pulseActive{false};
    };

    void updateOne(DiagnosticChannel channel, LedRuntime &led, MonotonicTimePoint now) noexcept;
    static void write(LedRuntime &led, bool high) noexcept;

    DiagnosticSignals &signals_;
    std::array<LedRuntime, static_cast<std::size_t>(DiagnosticChannel::Count)> leds_;
};

} // namespace cnc
