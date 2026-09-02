/**
 * @file FinderEnergyReader.h
 * @brief Declara o adaptador Modbus para o medidor Finder 7M.
 */

#pragma once

#include "domain/Models.h"

#include <Arduino.h>
#include <Finder7M.h>

namespace cnc
{

/** @brief Converte leituras da biblioteca Finder7M para EnergyData. */
class FinderEnergyReader final
{
  public:
    /** @brief Inicializa a interface Modbus RTU. */
    bool begin() noexcept;

    /**
     * @brief Lê todas as grandezas elétricas configuradas.
     * @param destination Recebe o conjunto completo apenas quando todas as
     * leituras forem válidas.
     * @return true se o ciclo completo foi concluído sem erro.
     */
    bool read(EnergyData &destination) noexcept;

  private:
    Finder7M meter_{};
};

} // namespace cnc
