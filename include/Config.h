/**
 * @file Config.h
 * @brief Reúne parâmetros de compilação, pinos e períodos do firmware.
 */

#pragma once

#include <Arduino.h>
#include <chrono>
#include <cstddef>
#include <cstdint>

// As macros podem ser sobrescritas por flags de compilação. Isso permite
// configurar credenciais e remapear entradas sem alterar o código-fonte.
#ifndef CNC_WIFI_SSID
#define CNC_WIFI_SSID ""
#endif

#ifndef CNC_WIFI_PASSWORD
#define CNC_WIFI_PASSWORD ""
#endif

#ifndef CNC_MQTT_BROKER
#define CNC_MQTT_BROKER ""
#endif

#ifndef CNC_MQTT_PORT
#define CNC_MQTT_PORT 1883
#endif

#ifndef CNC_MQTT_CLIENT_ID
#define CNC_MQTT_CLIENT_ID "cnc-monitor"
#endif

#ifndef CNC_MQTT_USERNAME
#define CNC_MQTT_USERNAME ""
#endif

#ifndef CNC_MQTT_PASSWORD
#define CNC_MQTT_PASSWORD ""
#endif

#ifndef CNC_MQTT_STATE_TOPIC
#define CNC_MQTT_STATE_TOPIC "cnc/state"
#endif

#ifndef CNC_LOG_ENABLED
#define CNC_LOG_ENABLED 0
#endif

#ifndef CNC_MACHINE_ON_PIN
#define CNC_MACHINE_ON_PIN PIN_A0
#endif

#ifndef CNC_SPINDLE_RUNNING_PIN
#define CNC_SPINDLE_RUNNING_PIN PIN_A1
#endif

#ifndef CNC_DOOR_CLOSED_PIN
#define CNC_DOOR_CLOSED_PIN PIN_A2
#endif

#ifndef CNC_DOOR_OPENED_PIN
#define CNC_DOOR_OPENED_PIN PIN_A3
#endif

#ifndef CNC_PART_CLAMPED_PIN
#define CNC_PART_CLAMPED_PIN PIN_A4
#endif

#ifndef CNC_MACHINE_READY_PIN
#define CNC_MACHINE_READY_PIN PIN_A5
#endif

/** @brief Configurações imutáveis utilizadas pelos componentes do firmware. */
namespace cnc
{
namespace config
{

/** SSID da rede Wi-Fi. */
constexpr char wifiSsid[] = CNC_WIFI_SSID;
/** Senha da rede Wi-Fi. */
constexpr char wifiPassword[] = CNC_WIFI_PASSWORD;

/** Endereço do broker MQTT. */
constexpr char mqttBroker[] = CNC_MQTT_BROKER;
/** Porta TCP do broker MQTT. */
constexpr std::uint16_t mqttPort = CNC_MQTT_PORT;
/** Identificador único do cliente MQTT. */
constexpr char mqttClientId[] = CNC_MQTT_CLIENT_ID;
/** Usuário utilizado na autenticação MQTT. */
constexpr char mqttUsername[] = CNC_MQTT_USERNAME;
/** Senha utilizada na autenticação MQTT. */
constexpr char mqttPassword[] = CNC_MQTT_PASSWORD;
/** Tópico que recebe o snapshot produtivo. */
constexpr char mqttStateTopic[] = CNC_MQTT_STATE_TOPIC;

/** Endereço Modbus do medidor Finder 7M. */
constexpr std::uint8_t finderAddress = 1U;
/** Taxa de comunicação Modbus RTU, em bits por segundo. */
constexpr std::uint32_t finderBaud = 38400U;

/** Entrada digital que informa máquina ligada. */
constexpr pin_size_t machineOnPin = CNC_MACHINE_ON_PIN;
/** Entrada digital que informa spindle em funcionamento. */
constexpr pin_size_t spindleRunningPin = CNC_SPINDLE_RUNNING_PIN;
/** Entrada digital que informa porta fechada. */
constexpr pin_size_t doorClosedPin = CNC_DOOR_CLOSED_PIN;
/** Entrada digital que informa porta aberta. */
constexpr pin_size_t doorOpenedPin = CNC_DOOR_OPENED_PIN;
/** Entrada digital que informa peça fixada. */
constexpr pin_size_t partClampedPin = CNC_PART_CLAMPED_PIN;
/** Entrada digital que informa máquina pronta. */
constexpr pin_size_t machineReadyPin = CNC_MACHINE_READY_PIN;

/** LED de diagnóstico geral do sistema. */
constexpr pin_size_t systemLedPin = LED_D0;
/** LED de diagnóstico da conexão Wi-Fi. */
constexpr pin_size_t wifiLedPin = LED_D1;
/** LED de diagnóstico da conexão MQTT. */
constexpr pin_size_t mqttLedPin = LED_D2;
/** LED de diagnóstico da comunicação Modbus. */
constexpr pin_size_t modbusLedPin = LED_D3;

/** Período da aquisição dos sinais digitais, em milissegundos. */
constexpr auto machineSamplePeriod = std::chrono::milliseconds{50};
/** Período de aquisição do medidor de energia, em milissegundos. */
constexpr auto energyPollPeriod = std::chrono::milliseconds{5000};
/** Período do laço de telemetria, em milissegundos. */
constexpr auto telemetryLoopPeriod = std::chrono::milliseconds{50};
/** Intervalo máximo sem publicação, em milissegundos. */
constexpr auto publishHeartbeat = std::chrono::milliseconds{60000};
/** Intervalo entre tentativas de reconexão, em milissegundos. */
constexpr auto reconnectInterval = std::chrono::milliseconds{5000};
/** Peso da nova amostra no filtro exponencial do tempo de ciclo. */
constexpr float cycleFilterAlpha = 0.2F;
/** Capacidade do buffer local usado para construir o JSON. */
constexpr std::size_t mqttPayloadCapacity = 600U;
/** Capacidade do buffer de transmissão do cliente MQTT. */
constexpr std::size_t mqttTxCapacity = 768U;

} // namespace config
} // namespace cnc
