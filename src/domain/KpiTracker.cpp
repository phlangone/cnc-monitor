#include "domain/KpiTracker.h"

namespace cnc
{

KpiTracker::KpiTracker(const float filterAlpha) noexcept
    : filterAlpha_(filterAlpha < 0.0F   ? 0.0F
                   : filterAlpha > 1.0F ? 1.0F
                                        : filterAlpha)
{
}

const KpiData &KpiTracker::update(const MachineData &sample, const MonotonicTimePoint now) noexcept
{
    const bool hadPreviousSample = initialized_;
    const Milliseconds elapsed =
        hadPreviousSample ? std::chrono::duration_cast<Milliseconds>(now - previousUpdate_)
                          : Milliseconds{0};
    previousUpdate_ = now;
    initialized_ = true;

    if (previousSample_.machineOn)
    {
        machineOnTime_ += elapsed;
    }

    if (isOperating(previousSample_))
    {
        operationTime_ += elapsed;
    }

    const bool programFinished =
        hadPreviousSample && sample.programEnd && !previousSample_.programEnd;

    if (programFinished)
    {
        ++kpis_.partCount;

        if (hasPreviousProgramEnd_)
        {
            const Milliseconds cycleTime =
                std::chrono::duration_cast<Milliseconds>(now - previousProgramEnd_);
            if (cycleTime.count() != 0)
            {
                const float cycleTimeMs = static_cast<float>(cycleTime.count());
                smoothedCycleTimeMs_ =
                    smoothedCycleTimeMs_ == 0.0F
                        ? cycleTimeMs
                        : filterAlpha_ * cycleTimeMs + (1.0F - filterAlpha_) * smoothedCycleTimeMs_;
                kpis_.cycleTimeS = cycleTimeMs / 1000.0F;
                kpis_.cadence = 3600000.0F / smoothedCycleTimeMs_;
            }
        }

        previousProgramEnd_ = now;
        hasPreviousProgramEnd_ = true;
    }
    previousSample_ = sample;

    kpis_.machineOnTimeS = static_cast<std::uint32_t>(machineOnTime_.count() / 1000);
    kpis_.operationTimeS = static_cast<std::uint32_t>(operationTime_.count() / 1000);
    kpis_.utilizationRate = machineOnTime_.count() == 0
                                ? 0.0F
                                : 100.0F * static_cast<float>(operationTime_.count()) /
                                      static_cast<float>(machineOnTime_.count());
    return kpis_;
}

bool KpiTracker::isOperating(const MachineData &sample) noexcept
{
    return sample.doorClosed && sample.partClamped && !sample.programEnd;
}

} // namespace cnc
