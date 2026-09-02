#include "domain/TelemetrySerializer.h"

#include <cstdio>

namespace cnc
{

std::size_t serializeTelemetry(const SystemSnapshot &snapshot, const char *const isoTimestamp,
                               char *const output, const std::size_t outputSize) noexcept
{
    if (isoTimestamp == nullptr || output == nullptr || outputSize == 0U)
    {
        return 0U;
    }

    const int length = std::snprintf(output, outputSize,
                                     "{\"time\":\"%s\",\"data\":{"
                                     "\"voltageV\":%.2f,\"currentA\":%.2f,\"activePowerW\":%.2f,"
                                     "\"reactivePowerVAr\":%.2f,\"apparentPowerVA\":%.2f,"
                                     "\"powerFactor\":%.3f,\"frequencyHz\":%.2f,"
                                     "\"activeEnergyKWh\":%.3f,"
                                     "\"machineOn\":%u,\"machineReady\":%u,\"doorClosed\":%u,"
                                     "\"doorOpened\":%u,\"partClamped\":%u,\"programEnd\":%u,"
                                     "\"machineOnTimeS\":%lu,\"operationTimeS\":%lu,"
                                     "\"utilizationRate\":%.1f,\"partCount\":%lu,"
                                     "\"cycleTimeS\":%.3f,\"cadence\":%.1f}}",
                                     isoTimestamp, static_cast<double>(snapshot.energy.voltageV),
                                     static_cast<double>(snapshot.energy.currentA),
                                     static_cast<double>(snapshot.energy.activePowerW),
                                     static_cast<double>(snapshot.energy.reactivePowerVAr),
                                     static_cast<double>(snapshot.energy.apparentPowerVA),
                                     static_cast<double>(snapshot.energy.powerFactor),
                                     static_cast<double>(snapshot.energy.frequencyHz),
                                     static_cast<double>(snapshot.energy.activeEnergyKWh),
                                     static_cast<unsigned int>(snapshot.machine.machineOn),
                                     static_cast<unsigned int>(snapshot.machine.machineReady),
                                     static_cast<unsigned int>(snapshot.machine.doorClosed),
                                     static_cast<unsigned int>(snapshot.machine.doorOpened),
                                     static_cast<unsigned int>(snapshot.machine.partClamped),
                                     static_cast<unsigned int>(snapshot.machine.programEnd),
                                     static_cast<unsigned long>(snapshot.kpis.machineOnTimeS),
                                     static_cast<unsigned long>(snapshot.kpis.operationTimeS),
                                     static_cast<double>(snapshot.kpis.utilizationRate),
                                     static_cast<unsigned long>(snapshot.kpis.partCount),
                                     static_cast<double>(snapshot.kpis.cycleTimeS),
                                     static_cast<double>(snapshot.kpis.cadence));

    return length > 0 && static_cast<std::size_t>(length) < outputSize
               ? static_cast<std::size_t>(length)
               : 0U;
}

} // namespace cnc
