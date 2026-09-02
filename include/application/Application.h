/**
 * @file Application.h
 * @brief Declara a raiz de composição e a política de execução do firmware.
 */

#pragma once

#include "platform/Rtos.h"

#include "application/StateStore.h"
#include "domain/KpiTracker.h"
#include "domain/PublishPolicy.h"
#include "platform/Diagnostics.h"
#include "platform/FinderEnergyReader.h"
#include "platform/Network.h"
#include "platform/OptaMachineInputs.h"

namespace cnc
{

/**
 * @brief Compõe componentes de domínio, plataforma e tarefas do firmware.
 *
 * A aplicação possui os objetos de longa duração. As tarefas definem apenas
 * cadências de execução; regras de negócio permanecem em componentes próprios.
 */
class Application final
{
  public:
    /** @brief Constrói a aplicação e injeta suas dependências por composição. */
    Application() noexcept;

    /** @brief Inicializa periféricos e inicia as tarefas uma única vez. */
    void start() noexcept;

  private:
    void runMachineAcquisition();
    void runEnergyAcquisition();
    void runTelemetry();
    void runDiagnostics();

    StateStore stateStore_{};
    DiagnosticSignals diagnosticSignals_{};
    DiagnosticLeds diagnosticLeds_;
    OptaMachineInputs machineInputs_{};
    FinderEnergyReader energyReader_{};
    WifiStation wifi_{};
    RtcClock clock_{};
    MqttTelemetryPublisher mqtt_{};
    KpiTracker kpiTracker_;
    PublishPolicy publishPolicy_;

    rtos::Thread machineThread_;
    rtos::Thread energyThread_;
    rtos::Thread telemetryThread_;
    rtos::Thread diagnosticThread_;
    bool started_{false};
};

} // namespace cnc
