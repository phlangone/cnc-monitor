/**
 * @file Models.h
 * @brief Define os modelos de dados independentes de hardware do monitor CNC.
 */

#pragma once

#include <cstdint>

namespace cnc
{

/**
 * @brief Armazena os indicadores produtivos calculados para a CNC.
 *
 * Os tempos expostos por esta estrutura são convertidos para segundos.
 * O controle interno de tempo do KpiTracker utiliza instantes monotônicos
 * fornecidos pela camada de aplicação.
 */
struct KpiData
{
    /** Tempo acumulado, em segundos, durante o qual a máquina permaneceu ligada. */
    std::uint32_t machineOnTimeS{0U};

    /** Tempo acumulado, em segundos, durante o qual a máquina esteve em operação. */
    std::uint32_t operationTimeS{0U};

    /** Percentual do tempo ligado utilizado em operação. */
    float utilizationRate{0.0F};

    /** Quantidade de peças contabilizadas. */
    std::uint32_t partCount{0U};

    /** Duração, em segundos, do último ciclo produtivo identificado. */
    float cycleTimeS{0.0F};

    /** Estimativa de produção em peças por hora. */
    float cadence{0.0F};
};

/**
 * @brief Representa o estado instantâneo dos sinais monitorados da CNC.
 */
struct MachineData
{
    /** Indica se a máquina está ligada. */
    bool machineOn{false};

    /** Indica se a máquina está pronta para operação. */
    bool machineReady{false};

    /** Indica se a porta está fechada. */
    bool doorClosed{false};

    /** Indica se a porta está aberta. */
    bool doorOpened{false};

    /** Indica se a peça está fixada. */
    bool partClamped{false};

    /** Indica se o spindle está em funcionamento. */
    bool spindleRunning{false};
};

/**
 * @brief Armazena as grandezas elétricas obtidas do medidor de energia.
 *
 * @note As unidades devem ser confirmadas de acordo com o comportamento da
 * biblioteca Finder7M utilizada no projeto.
 */
struct EnergyData
{
    /** Tensão eficaz da fase 1, em volts. */
    float voltageV{0.0F};

    /** Corrente eficaz da fase 1, em amperes. */
    float currentA{0.0F};

    /** Potência ativa, em watts. */
    float activePowerW{0.0F};

    /** Potência reativa total, em volt-ampere reativo. */
    float reactivePowerVAr{0.0F};

    /** Potência aparente total, em volt-amperes. */
    float apparentPowerVA{0.0F};

    /** Fator de potência total. */
    float powerFactor{0.0F};

    /** Frequência da rede elétrica, em hertz. */
    float frequencyHz{0.0F};

    /** Energia ativa importada acumulada, em quilowatt-hora. */
    float activeEnergyKWh{0.0F};
};

/**
 * @brief Snapshot coerente do estado produtivo e energético do firmware.
 */
struct SystemSnapshot
{
    /** Estado instantâneo dos sinais da máquina. */
    MachineData machine{};

    /** Grandezas elétricas mais recentes obtidas do medidor de energia. */
    EnergyData energy{};

    /** Indicadores produtivos derivados dos sinais da máquina. */
    KpiData kpis{};
};

} // namespace cnc
