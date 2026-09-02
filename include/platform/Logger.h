/**
 * @file Logger.h
 * @brief Declara o logger serial opcional e sincronizado.
 */

#pragma once

namespace cnc
{

/** @brief Inicializa a saída serial quando CNC_LOG_ENABLED está habilitado. */
void initializeLogging() noexcept;

/** @brief Registra uma mensagem formatada sem bloquear indefinidamente. */
void logf(const char *format, ...) noexcept;

} // namespace cnc
