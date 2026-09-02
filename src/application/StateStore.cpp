#include "application/StateStore.h"

namespace cnc
{

void StateStore::updateMachine(const MachineData &machine, const KpiData &kpis) noexcept
{
    const Guard guard{mutex_};
    state_.machine = machine;
    state_.kpis = kpis;
}

void StateStore::updateEnergy(const EnergyData &energy) noexcept
{
    const Guard guard{mutex_};
    state_.energy = energy;
}

SystemSnapshot StateStore::snapshot() const noexcept
{
    const Guard guard{mutex_};
    return state_;
}

} // namespace cnc
