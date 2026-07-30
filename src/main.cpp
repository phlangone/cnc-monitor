#include <Arduino.h>
#include "Logger.h"
#include "Led.h"

Logger logger{Serial};
Led led{PIN_A0, LedState::Off};

void setup()
{
    Serial.begin(115200);

    logger.setEnabled(true);
    logger.printf("Teste do Logger");

    led.begin();
}

void loop()
{
}