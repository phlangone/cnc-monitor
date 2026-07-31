/**
 * @file EnergyMeter.cpp
 * @brief Implementação da leitura do medidor de energia.
 */
#include "EnergyMeter.h"

EnergyMeter::EnergyMeter(
    uint8_t modbusAddress,
    uint32_t baudRate
)
    : modbusAddress_{modbusAddress},
      baudRate_{baudRate}
{
}

bool EnergyMeter::begin()
{
    initialized_ = finder_.init(baudRate_);
    return initialized_;
}

bool EnergyMeter::update()
{
    if (!initialized_)
    {
        return false;
    }

    EnergyData newData{};

    if (!readAll(newData))
    {
        return false;
    }

    energyData_ = newData;
    hasValidData_ = true;

    return true;
}

const EnergyData& EnergyMeter::data() const
{
    return energyData_;
}

bool EnergyMeter::hasValidData() const
{
    return hasValidData_;
}

bool EnergyMeter::readAll(EnergyData& data)
{
    bool success = true;

    Measure voltage =
        finder_.getVoltagePhase1(modbusAddress_);

    data.voltageV = voltage.toFloat();
    success &= !voltage.isReadError();

    Measure activePower =
        finder_.getActivePowerTotal(modbusAddress_);

    data.activePowerW = activePower.toFloat();
    success &= !activePower.isReadError();

    Measure reactivePower =
        finder_.getReactivePowerTotal(modbusAddress_);

    data.reactivePowerVAr = reactivePower.toFloat();
    success &= !reactivePower.isReadError();

    Measure apparentPower =
        finder_.getApparentPowerTotal(modbusAddress_);

    data.apparentPowerVA = apparentPower.toFloat();
    success &= !apparentPower.isReadError();

    Measure powerFactor =
        finder_.getPowerFactorTotal(modbusAddress_);

    data.powerFactor = powerFactor.toFloat();
    success &= !powerFactor.isReadError();

    Measure frequency =
        finder_.getFrequency(modbusAddress_);

    data.frequencyHz = frequency.toFloat();
    success &= !frequency.isReadError();

    Measure current =
        finder_.getCurrentPhase1(modbusAddress_);

    data.currentA = current.toFloat();
    success &= !current.isReadError();

    Measure activeEnergy =
        finder_.getMIDInActiveEnergy(modbusAddress_);

    data.activeEnergyKWh = activeEnergy.toFloat();
    success &= !activeEnergy.isReadError();

    return success;
}