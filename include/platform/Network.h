/**
 * @file Network.h
 * @brief Declara os adaptadores de Wi-Fi, relógio e publicação MQTT.
 */

#pragma once

#include <Arduino.h>
#include <ArduinoMqttClient.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <cstddef>
#include <cstdint>

namespace cnc
{

/** @brief Encapsula conexão e consulta de estado da interface Wi-Fi. */
class WifiStation final
{
  public:
    /** @brief Informa se a estação está conectada. */
    bool connected() const noexcept;
    /** @brief Executa uma tentativa de conexão com a rede configurada. */
    bool connectOnce() const noexcept;
};

/** @brief Sincroniza o RTC via NTP e produz timestamps ISO 8601. */
class RtcClock final
{
  public:
    /** @brief Constrói o cliente NTP sobre seu transporte UDP. */
    RtcClock();

    /** @brief Atualiza o relógio do sistema a partir do servidor NTP. */
    bool synchronize() noexcept;
    /** @brief Escreve a hora UTC atual em formato ISO 8601. */
    bool isoTimestamp(char *output, std::size_t outputSize) const noexcept;

  private:
    WiFiUDP udp_{};
    NTPClient ntp_;
};

/** @brief Gerencia a sessão MQTT usada exclusivamente pela tarefa de telemetria. */
class MqttTelemetryPublisher final
{
  public:
    /** @brief Constrói o cliente MQTT sobre o transporte Wi-Fi. */
    MqttTelemetryPublisher();

    /** @brief Aplica identificação, autenticação e capacidade de transmissão. */
    void begin() noexcept;
    /** @brief Informa se a sessão MQTT está ativa. */
    bool connected() noexcept;
    /** @brief Executa uma tentativa de conexão com o broker configurado. */
    bool connect() noexcept;
    /** @brief Mantém viva uma sessão conectada. */
    void poll() noexcept;
    /** @brief Publica um payload completo no tópico informado. */
    bool publish(const char *topic, const char *payload, std::size_t length) noexcept;
    /** @brief Retorna o código da última falha de conexão. */
    int connectError() noexcept;

  private:
    WiFiClient transport_{};
    MqttClient client_;
};

} // namespace cnc
