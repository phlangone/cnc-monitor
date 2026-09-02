/**
 * @file PublishPolicy.h
 * @brief Declara a regra independente de hardware para publicação MQTT.
 */

#pragma once

#include "domain/Models.h"
#include "domain/Time.h"

#include <chrono>

namespace cnc
{

/**
 * @brief Decide quando o estado da máquina deve ser publicado.
 *
 * Uma publicação é solicitada na primeira amostra, na alteração de qualquer
 * sinal monitorado ou ao expirar o heartbeat.
 */
class PublishPolicy final
{
  public:
    /**
     * @brief Constrói a política de publicação.
     * @param heartbeat Intervalo máximo entre publicações.
     */
    explicit PublishPolicy(Milliseconds heartbeat) noexcept;

    /**
     * @brief Verifica se o estado atual deve ser publicado.
     * @param current Estado atual da CNC.
     * @param now Instante monotônico atual.
     */
    bool shouldPublish(const MachineData &current, MonotonicTimePoint now) const noexcept;

    /**
     * @brief Registra uma publicação concluída com sucesso.
     * @param published Estado efetivamente publicado.
     * @param now Instante monotônico da publicação.
     */
    void markPublished(const MachineData &published, MonotonicTimePoint now) noexcept;

  private:
    MachineData lastPublished_{};
    MonotonicTimePoint lastPublish_{};
    Milliseconds heartbeat_;
    bool hasPublished_{false};
};

} // namespace cnc
