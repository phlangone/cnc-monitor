/**
 * @file TelemetrySerializer.h
 * @brief Declara a serialização do snapshot produtivo em JSON.
 */

#pragma once

#include "domain/Models.h"

#include <cstddef>

namespace cnc
{

/**
 * @brief Serializa um snapshot produtivo para o contrato JSON MQTT.
 * @param snapshot Estado coerente da máquina e dos KPIs.
 * @param isoTimestamp Data e hora em formato ISO 8601 UTC.
 * @param output Buffer de destino fornecido pelo chamador.
 * @param outputSize Capacidade total de output, em bytes.
 * @return Tamanho do JSON sem o terminador nulo, ou zero em caso de erro.
 */
std::size_t serializeTelemetry(const SystemSnapshot &snapshot, const char *isoTimestamp,
                               char *output, std::size_t outputSize) noexcept;

} // namespace cnc
