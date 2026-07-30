#pragma once

#include <Arduino.h>

/**
 * @brief Representa o estado instantâneo dos sinais monitorados da CNC.
 */
struct MachineData
{
    /** Indica se a máquina está ligada. */
    bool machineOn;

    /** Indica se a máquina está pronta para operação. */
    bool machineReady;

    /** Indica se a porta está fechada. */
    bool doorClosed;

    /** Indica se a porta está aberta. */
    bool doorOpened;

    /** Indica se a peça está fixada. */
    bool partClamped;

    /** Indica se o spindle está em funcionamento. */
    bool spindleRunning;
};

/**
 * @brief Realiza a leitura e o armazenamento dos sinais digitais da CNC.
 *
 * A classe encapsula os pinos utilizados no monitoramento da máquina e mantém
 * internamente o estado mais recente dos sinais por meio de uma estrutura
 * MachineData.
 *
 * A classe não cria nem gerencia threads. O método update() deve ser chamado
 * periodicamente pela aplicação.
 */
class MachineMonitor
{
public:
    /**
     * @brief Constrói um monitor dos sinais da CNC.
     *
     * @param machineOnPin Pino do sinal de máquina ligada.
     * @param machineReadyPin Pino do sinal de máquina pronta.
     * @param doorClosedPin Pino do sinal de porta fechada.
     * @param doorOpenedPin Pino do sinal de porta aberta.
     * @param partClampedPin Pino do sinal de peça fixada.
     * @param spindleRunningPin Pino do sinal de spindle em funcionamento.
     */
    MachineMonitor(
        uint8_t machineOnPin,
        uint8_t machineReadyPin,
        uint8_t doorClosedPin,
        uint8_t doorOpenedPin,
        uint8_t partClampedPin,
        uint8_t spindleRunningPin
    )
        : machineOnPin_{machineOnPin},
          machineReadyPin_{machineReadyPin},
          doorClosedPin_{doorClosedPin},
          doorOpenedPin_{doorOpenedPin},
          partClampedPin_{partClampedPin},
          spindleRunningPin_{spindleRunningPin}
    {
    }

    /**
     * @brief Inicializa os pinos utilizados pelo monitor.
     *
     * Deve ser chamado uma vez durante a inicialização da aplicação, antes
     * da primeira chamada a update().
     */
    void begin();

    /**
     * @brief Atualiza o estado interno da máquina.
     *
     * Realiza a leitura dos sinais digitais configurados e armazena os
     * resultados na estrutura interna MachineData.
     */
    void update();

    /**
     * @brief Retorna o estado mais recente da máquina.
     *
     * @return Referência constante para os dados internos da máquina.
     *
     * @note A referência permanece válida enquanto o objeto MachineMonitor
     * existir. O método não realiza uma nova leitura dos pinos.
     */
    const MachineData& data() const;

private:
    uint8_t machineOnPin_;
    uint8_t machineReadyPin_;
    uint8_t doorClosedPin_;
    uint8_t doorOpenedPin_;
    uint8_t partClampedPin_;
    uint8_t spindleRunningPin_;

    MachineData machineData_{};
};