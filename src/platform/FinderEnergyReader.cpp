#include "platform/FinderEnergyReader.h"

#include "Config.h"
#include "platform/Rtos.h"

#include <chrono>

namespace cnc
{
namespace
{

void interRegisterDelay()
{
    rtos::ThisThread::sleep_for(std::chrono::milliseconds{10});
}

} // namespace

bool FinderEnergyReader::begin() noexcept
{
    return meter_.init(config::finderBaud);
}

bool FinderEnergyReader::read(EnergyData &destination) noexcept
{
    EnergyData candidate{};
    bool success = true;

    Measure measure = meter_.getVoltagePhase1(config::finderAddress);
    candidate.voltageV = measure.toFloat();
    success &= !measure.isReadError();
    interRegisterDelay();

    measure = meter_.getCurrentPhase1(config::finderAddress);
    candidate.currentA = measure.toFloat();
    success &= !measure.isReadError();
    interRegisterDelay();

    measure = meter_.getActivePowerTotal(config::finderAddress);
    candidate.activePowerW = measure.toFloat();
    success &= !measure.isReadError();
    interRegisterDelay();

    measure = meter_.getReactivePowerTotal(config::finderAddress);
    candidate.reactivePowerVAr = measure.toFloat();
    success &= !measure.isReadError();
    interRegisterDelay();

    measure = meter_.getApparentPowerTotal(config::finderAddress);
    candidate.apparentPowerVA = measure.toFloat();
    success &= !measure.isReadError();
    interRegisterDelay();

    measure = meter_.getPowerFactorTotal(config::finderAddress);
    candidate.powerFactor = measure.toFloat();
    success &= !measure.isReadError();
    interRegisterDelay();

    measure = meter_.getFrequency(config::finderAddress);
    candidate.frequencyHz = measure.toFloat();
    success &= !measure.isReadError();
    interRegisterDelay();

    measure = meter_.getMIDInActiveEnergy(config::finderAddress);
    candidate.activeEnergyKWh = measure.toFloat();
    success &= !measure.isReadError();

    if (success)
    {
        destination = candidate;
    }
    return success;
}

} // namespace cnc
