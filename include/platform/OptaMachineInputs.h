/**
 * @file OptaMachineInputs.h
 * @brief Declara o adaptador das entradas digitais do Arduino Opta.
 */

#pragma once

#include "domain/Models.h"

namespace cnc
{

/** @brief Traduz os níveis elétricos das entradas para MachineData. */
class OptaMachineInputs final
{
  public:
    /** @brief Configura todos os pinos monitorados como entradas digitais. */
    void begin() const noexcept;

    /** @brief Obtém uma amostra instantânea dos seis sinais da CNC. */
    MachineData read() const noexcept;
};

} // namespace cnc
