/**
 * @file MachineMonitor.cpp
 * @brief Implementação do monitoramento dos sinais da CNC.
 */
#include "MachineMonitor.h"

MachineMonitor::MachineMonitor(
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

void MachineMonitor::begin()
{
    pinMode(machineOnPin_, INPUT);
    pinMode(machineReadyPin_, INPUT);
    pinMode(doorClosedPin_, INPUT);
    pinMode(doorOpenedPin_, INPUT);
    pinMode(partClampedPin_, INPUT);
    pinMode(spindleRunningPin_, INPUT);

    update();
}

void MachineMonitor::update()
{
    machineData_.machineOn =
        digitalRead(machineOnPin_) == HIGH;

    machineData_.machineReady =
        digitalRead(machineReadyPin_) == HIGH;

    machineData_.doorClosed =
        digitalRead(doorClosedPin_) == HIGH;

    machineData_.doorOpened =
        digitalRead(doorOpenedPin_) == HIGH;

    machineData_.partClamped =
        digitalRead(partClampedPin_) == HIGH;

    machineData_.spindleRunning =
        digitalRead(spindleRunningPin_) == HIGH;
}

const MachineData& MachineMonitor::data() const
{
    return machineData_;
}