/**
 * @file Time.h
 * @brief Define os tipos de tempo monotônico compartilhados pelo domínio.
 */

#pragma once

#include <chrono>

namespace cnc
{

/**
 * @brief Relógio lógico do domínio usado para representar tempo desde o boot.
 *
 * O tipo não consulta o hardware. A camada de aplicação converte o instante do
 * relógio do RTOS para este relógio antes de chamar as regras de domínio.
 */
struct MonotonicClock final
{
    using duration = std::chrono::milliseconds;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<MonotonicClock>;

    static constexpr bool is_steady = true;
};

/** @brief Instante de um relógio monotônico, imune a ajustes do RTC/NTP. */
using MonotonicTimePoint = MonotonicClock::time_point;

/** @brief Duração-base usada nas cadências e integrações temporais. */
using Milliseconds = std::chrono::milliseconds;

} // namespace cnc
