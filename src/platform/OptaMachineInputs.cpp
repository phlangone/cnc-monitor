#include "platform/OptaMachineInputs.h"

#include "Config.h"

#include <Arduino.h>

namespace cnc
{

void OptaMachineInputs::begin() const noexcept
{
    pinMode(config::machineOnPin, INPUT);
    pinMode(config::machineReadyPin, INPUT);
    pinMode(config::doorClosedPin, INPUT);
    pinMode(config::doorOpenedPin, INPUT);
    pinMode(config::partClampedPin, INPUT);
    pinMode(config::programEndPin, INPUT);
}

MachineData OptaMachineInputs::read() const noexcept
{
    MachineData sample{};
    sample.machineOn = digitalRead(config::machineOnPin) != LOW;
    sample.machineReady = digitalRead(config::machineReadyPin) != LOW;
    sample.doorClosed = digitalRead(config::doorClosedPin) != LOW;
    sample.doorOpened = digitalRead(config::doorOpenedPin) != LOW;
    sample.partClamped = digitalRead(config::partClampedPin) != LOW;
    // O fim de programa permanece eletricamente em nível baixo após o M30 e
    // retorna ao nível alto somente quando o próximo ciclo é iniciado.
    sample.programEnd = digitalRead(config::programEndPin) == LOW;
    return sample;
}

} // namespace cnc
