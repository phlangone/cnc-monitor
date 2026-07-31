/**
 * @file KpiTracker.cpp
 * @brief Implementação do cálculo dos indicadores da CNC.
 */
#include "KpiTracker.h"

void KpiTracker::begin()
{
    const TimePoint now = Clock::now();

    lastUpdateTime_ = now;
    lastCycleTime_ = now;

    machineOnDuration_ = Duration::zero();
    operationDuration_ = Duration::zero();

    kpiData_ = {};
    previousPartClamped_ = false;

    initialized_ = false;
    hasCompletedFirstCycle_ = false;
}

void KpiTracker::update(const MachineData& machineData)
{
    const TimePoint now = Clock::now();

    /*
     * A primeira atualização estabelece as referências iniciais.
     * Nenhum tempo ou ciclo é calculado nessa chamada.
     */
    if (!initialized_)
    {
        lastUpdateTime_ = now;
        lastCycleTime_ = now;
        previousPartClamped_ = machineData.partClamped;
        initialized_ = true;
        return;
    }

    const Duration elapsed = now - lastUpdateTime_;
    lastUpdateTime_ = now;

    updateAccumulatedTimes(machineData, elapsed);
    updatePartCount(machineData, now);
    updateDerivedIndicators();
}

void KpiTracker::reset()
{
    const TimePoint now = Clock::now();

    kpiData_ = {};

    machineOnDuration_ = Duration::zero();
    operationDuration_ = Duration::zero();

    lastUpdateTime_ = now;
    lastCycleTime_ = now;

    previousPartClamped_ = false;
    initialized_ = false;
    hasCompletedFirstCycle_ = false;
}

const KpiData& KpiTracker::data() const
{
    return kpiData_;
}

void KpiTracker::updateAccumulatedTimes(
    const MachineData& machineData,
    Duration elapsed
)
{
    if (machineData.machineOn)
    {
        machineOnDuration_ += elapsed;
    }

    if (machineData.machineOn && machineData.spindleRunning)
    {
        operationDuration_ += elapsed;
    }
}

void KpiTracker::updatePartCount(
    const MachineData& machineData,
    TimePoint now
)
{
    const bool partClampedRisingEdge =
        machineData.partClamped && !previousPartClamped_;

    previousPartClamped_ = machineData.partClamped;

    if (!partClampedRisingEdge)
    {
        return;
    }

    ++kpiData_.partCount;

    /*
     * A primeira peça fornece apenas a referência inicial.
     * São necessários dois eventos para medir um ciclo completo.
     */
    if (!hasCompletedFirstCycle_)
    {
        lastCycleTime_ = now;
        hasCompletedFirstCycle_ = true;
        return;
    }

    const auto cycleDuration = now - lastCycleTime_;
    lastCycleTime_ = now;

    kpiData_.cycleTimeS =
        std::chrono::duration<float>(cycleDuration).count();
}

void KpiTracker::updateDerivedIndicators()
{
    const float machineOnTimeS =
        std::chrono::duration<float>(machineOnDuration_).count();

    const float operationTimeS =
        std::chrono::duration<float>(operationDuration_).count();

    kpiData_.machineOnTimeS =
        static_cast<uint32_t>(machineOnTimeS);

    kpiData_.operationTimeS =
        static_cast<uint32_t>(operationTimeS);

    if (machineOnTimeS > 0.0f)
    {
        kpiData_.utilizationRate =
            (operationTimeS / machineOnTimeS) * 100.0f;
    }
    else
    {
        kpiData_.utilizationRate = 0.0f;
    }

    if (kpiData_.cycleTimeS > 0.0f)
    {
        kpiData_.cadence =
            3600.0f / kpiData_.cycleTimeS;
    }
    else
    {
        kpiData_.cadence = 0.0f;
    }
}