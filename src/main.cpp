#include <Arduino.h>
#include "logger.h"

Logger logger{Serial};

void setup()
{
    Serial.begin(115200);

    logger.setEnabled(true);
    logger.printf("Teste do Logger");
}

void loop()
{
}