#include "MachineMonitor.h"

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