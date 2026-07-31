/**
 * @file KpiTracker.h
 * @brief Declaração da classe de cálculo dos indicadores da CNC.
 */
#pragma once

#include "MachineMonitor.h"

#include <cstdint>
#include <chrono>
#include <rtos.h>

/**
 * @brief Armazena os indicadores produtivos calculados para a CNC.
 *
 * Os tempos expostos por esta estrutura são convertidos para segundos.
 * O controle interno de tempo do KpiTracker utiliza o relógio monotônico
 * fornecido pelo RTOS.
 */
struct KpiData
{
    /** Tempo acumulado, em segundos, durante o qual a máquina permaneceu ligada. */
    uint32_t machineOnTimeS;

    /** Tempo acumulado, em segundos, durante o qual a máquina esteve em operação. */
    uint32_t operationTimeS;

    /** Percentual do tempo ligado utilizado em operação. */
    float utilizationRate;

    /** Quantidade de peças contabilizadas. */
    uint32_t partCount;

    /** Duração, em segundos, do último ciclo produtivo identificado. */
    float cycleTimeS;

    /** Estimativa de produção em peças por hora. */
    float cadence;
};

/**
 * @brief Calcula e mantém os indicadores produtivos da CNC.
 *
 * A classe recebe periodicamente o estado atual da máquina e atualiza
 * os tempos acumulados, a contagem de peças, o tempo de ciclo, a taxa
 * de utilização e a cadência estimada.
 *
 * Critérios adotados:
 * - machineOnTimeS é acumulado enquanto machineOn estiver ativo;
 * - operationTimeS é acumulado enquanto machineOn e spindleRunning
 *   estiverem ativos;
 * - utilizationRate corresponde à relação entre operationTimeS e
 *   machineOnTimeS, expressa em percentual;
 * - partCount é incrementado provisoriamente na borda de subida de
 *   partClamped;
 * - cycleTimeS corresponde ao intervalo entre duas bordas de subida
 *   consecutivas de partClamped;
 * - cadence é calculada em peças por hora a partir do último tempo
 *   de ciclo válido.
 *
 * @note O sinal partClamped é utilizado provisoriamente como referência
 * para uma nova peça. Esse evento indica uma nova fixação, mas não garante
 * que a usinagem anterior tenha sido concluída. O critério deverá ser
 * revisado quando houver um sinal confiável de término de usinagem.
 *
 * @note A classe não cria nem gerencia threads. O método update() deve
 * ser chamado periodicamente pela aplicação.
 */
class KpiTracker
{
public:
    /**
     * @brief Constrói o rastreador de indicadores.
     *
     * Os indicadores e estados internos são inicialmente zerados.
     */
    KpiTracker() = default;

    /**
     * @brief Inicializa as referências temporais e os estados internos.
     *
     * Deve ser chamado antes da primeira execução de update().
     */
    void begin();    

    /**
     * @brief Atualiza os indicadores a partir do estado atual da CNC.
     *
     * O método calcula o intervalo desde a atualização anterior, acumula
     * os tempos aplicáveis, detecta uma nova fixação de peça e atualiza
     * os indicadores derivados.
     *
     * Na primeira chamada, o método apenas inicializa as referências
     * temporais e o estado anterior de partClamped.
     *
     * @param machineData Estado atual dos sinais monitorados da CNC.
     */
    void update(const MachineData& machineData);

    /**
     * @brief Reinicia todos os indicadores e estados internos.
     *
     * Após a chamada, tempos, contagem, tempo de ciclo, utilização e
     * cadência retornam aos valores iniciais.
     */
    void reset();

    /**
     * @brief Retorna os indicadores produtivos mais recentes.
     *
     * @return Referência constante para os dados internos de KPI.
     *
     * @note O método não realiza novos cálculos.
     */
    const KpiData& data() const;

private:
    /** Relógio monotônico utilizado para medir intervalos internos. */
    using Clock = rtos::Kernel::Clock;

    /** Representa um instante do relógio monotônico. */
    using TimePoint = Clock::time_point;

    /** Representa uma duração medida pelo relógio monotônico. */
    using Duration = Clock::duration;

    /**
     * @brief Atualiza os tempos acumulados.
     *
     * @param machineData Estado atual da CNC.
     * @param elapsed Intervalo transcorrido desde a atualização anterior.
     */
    void updateAccumulatedTimes(
        const MachineData& machineData,
        Duration elapsed
    );

    /**
     * @brief Atualiza a contagem de peças e o tempo de ciclo.
     *
     * Detecta provisoriamente uma nova peça pela borda de subida de
     * partClamped.
     *
     * @param machineData Estado atual da CNC.
     * @param now Instante atual do relógio monotônico.
     */
    void updatePartCount(
        const MachineData& machineData,
        TimePoint now
    );

    /**
     * @brief Atualiza os indicadores derivados.
     *
     * Converte os tempos acumulados para segundos e calcula a taxa de
     * utilização e a cadência.
     */
    void updateDerivedIndicators();

    /** Dados públicos mais recentes dos indicadores. */
    KpiData kpiData_{};

    /** Instante da atualização anterior. */
    TimePoint lastUpdateTime_;

    /** Instante em que foi detectado o último início de ciclo. */
    TimePoint lastCycleTime_;

    /** Tempo total acumulado com a máquina ligada. */
    Duration machineOnDuration_{};

    /** Tempo total acumulado com a máquina em operação. */
    Duration operationDuration_{};

    /** Estado de partClamped observado na atualização anterior. */
    bool previousPartClamped_ = false;

    /** Indica se as referências iniciais de tempo já foram configuradas. */
    bool initialized_ = false;

    /** Indica se já existe uma primeira referência válida de ciclo. */
    bool hasCompletedFirstCycle_ = false;
};