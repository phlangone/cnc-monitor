/**
 * @file KpiTracker.h
 * @brief Declara o rastreador de indicadores produtivos da CNC.
 */

#pragma once

#include "domain/Models.h"
#include "domain/Time.h"

#include <chrono>

namespace cnc
{

/**
 * @brief Calcula KPIs a partir de amostras temporais dos sinais da máquina.
 *
 * A classe é independente de Arduino e RTOS. O chamador fornece um instante
 * monotônico, o que permite testes nativos determinísticos.
 * Uma operação produtiva é inferida enquanto a porta está fechada, a peça está
 * fixada e o fim de programa ainda não ocorreu. Cada borda de ativação do fim
 * de programa contabiliza uma peça concluída.
 */
class KpiTracker final
{
  public:
    /**
     * @brief Constrói o rastreador com o filtro do tempo de ciclo configurado.
     * @param filterAlpha Peso da amostra nova no filtro exponencial, entre 0 e 1.
     */
    explicit KpiTracker(float filterAlpha) noexcept;

    /**
     * @brief Incorpora uma nova amostra e atualiza os indicadores.
     * @param sample Estado instantâneo dos sinais da CNC.
     * @param now Instante monotônico atual.
     * @return Referência válida até a próxima chamada a update().
     */
    const KpiData &update(const MachineData &sample, MonotonicTimePoint now) noexcept;

    /** @brief Retorna os últimos indicadores calculados. */
    const KpiData &value() const noexcept
    {
        return kpis_;
    }

  private:
    static bool isOperating(const MachineData &sample) noexcept;

    KpiData kpis_{};
    MachineData previousSample_{};
    float filterAlpha_;
    Milliseconds machineOnTime_{0};
    Milliseconds operationTime_{0};
    MonotonicTimePoint previousUpdate_{};
    MonotonicTimePoint previousProgramEnd_{};
    float smoothedCycleTimeMs_{0.0F};
    bool hasPreviousProgramEnd_{false};
    bool initialized_{false};
};

} // namespace cnc
