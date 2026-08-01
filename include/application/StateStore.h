/**
 * @file StateStore.h
 * @brief Declara o repositório sincronizado de estado compartilhado.
 */

#pragma once

#include "platform/Rtos.h"

#include "domain/Models.h"

namespace cnc
{

/**
 * @brief Mantém o único estado de negócio compartilhado entre as tarefas.
 *
 * Escritores substituem objetos completos e leitores obtêm uma cópia coerente.
 * O mutex é mantido apenas durante cópias rápidas, nunca durante I/O.
 */
class StateStore final
{
  public:
    /** @brief Atualiza, atomicamente, os sinais e KPIs produtivos. */
    void updateMachine(const MachineData &machine, const KpiData &kpis) noexcept;

    /** @brief Atualiza, atomicamente, as grandezas do medidor de energia. */
    void updateEnergy(const EnergyData &energy) noexcept;

    /** @brief Retorna uma cópia coerente do estado completo. */
    SystemSnapshot snapshot() const noexcept;

  private:
    /** @brief Guarda RAII usado internamente para liberar o mutex com segurança. */
    class Guard final
    {
      public:
        explicit Guard(rtos::Mutex &mutex) : mutex_(mutex)
        {
            mutex_.lock();
        }
        ~Guard()
        {
            mutex_.unlock();
        }
        Guard(const Guard &) = delete;
        Guard &operator=(const Guard &) = delete;

      private:
        rtos::Mutex &mutex_;
    };

    mutable rtos::Mutex mutex_{};
    SystemSnapshot state_{};
};

} // namespace cnc
