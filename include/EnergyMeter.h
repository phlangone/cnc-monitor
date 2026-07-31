/**
 * @file EnergyMeter.h
 * @brief Declaração da classe de leitura do medidor de energia.
 */
#pragma once

#include <Arduino.h>
#include <Finder7M.h>

/**
 * @file EnergyMeter.h
 * @brief Declaração da classe responsável pela leitura das grandezas
 * elétricas do medidor Finder 7M.
 */

/**
 * @brief Armazena as grandezas elétricas obtidas do medidor de energia.
 *
 * @note As unidades devem ser confirmadas de acordo com o comportamento
 * da biblioteca Finder7M utilizada no projeto.
 */
struct EnergyData
{
    /** Tensão eficaz da fase 1, em volts. */
    float voltageV;

    /** Corrente eficaz da fase 1, em amperes. */
    float currentA;

    /** Potência ativa, em watts. */
    float activePowerW;

    /** Potência reativa total, em volt-ampere reativo. */
    float reactivePowerVAr;

    /** Potência aparente total, em volt-amperes. */
    float apparentPowerVA;

    /** Fator de potência total. */
    float powerFactor;

    /** Frequência da rede elétrica, em hertz. */
    float frequencyHz;

    /** Energia ativa importada acumulada, em quilowatt-hora. */
    float activeEnergyKWh;
};

/**
 * @brief Realiza a leitura das grandezas elétricas do medidor Finder 7M.
 *
 * @note A classe não gerencia threads, filas, logs, serialização JSON
 * ou publicação MQTT. Essas responsabilidades pertencem à aplicação.
 */
class EnergyMeter
{
public:
    /**
     * @brief Constrói o objeto responsável pelo medidor de energia.
     *
     * @param modbusAddress Endereço Modbus RTU do medidor Finder 7M.
     * @param baudRate Taxa de comunicação da interface Modbus, em bit/s.
     */
    EnergyMeter(
        uint8_t modbusAddress,
        uint32_t baudRate
    );

    /**
     * @brief Inicializa a comunicação com o medidor.
     *
     * @return true se a comunicação for inicializada com sucesso.
     * @return false se ocorrer falha durante a inicialização.
     */
    bool begin();

    /**
     * @brief Lê todas as grandezas elétricas e atualiza os dados internos.
     *
     * @return true se todas as leituras forem concluídas com sucesso.
     * @return false se a classe não estiver inicializada ou se alguma
     * leitura apresentar erro.
     */
    bool update();

    /**
     * @brief Retorna a última amostra válida obtida do medidor.
     *
     * @return Referência constante para os dados elétricos internos.
     *
     * @note O método não realiza uma nova leitura.
     */
    const EnergyData& data() const;

    /**
     * @brief Informa se já existe uma amostra válida disponível.
     *
     * @return true após pelo menos uma atualização bem-sucedida.
     * @return false enquanto nenhuma leitura completa tiver sido obtida.
     */
    bool hasValidData() const;

private:
    /**
     * @brief Realiza a leitura de todas as grandezas elétricas.
     *
     * @param data Estrutura que receberá a nova amostra.
     *
     * @return true se todas as grandezas forem lidas sem erro.
     * @return false se pelo menos uma leitura apresentar falha.
     */
    bool readAll(EnergyData& data);

    /** Objeto da biblioteca responsável pela comunicação com o Finder 7M. */
    Finder7M finder_;

    /** Endereço Modbus RTU configurado para o medidor. */
    uint8_t modbusAddress_;

    /** Taxa de comunicação Modbus, em bit/s. */
    uint32_t baudRate_;

    /** Última amostra válida obtida do medidor. */
    EnergyData energyData_{};

    /** Indica se a comunicação com o medidor foi inicializada. */
    bool initialized_ = false;

    /** Indica se pelo menos uma leitura completa foi realizada. */
    bool hasValidData_ = false;
};