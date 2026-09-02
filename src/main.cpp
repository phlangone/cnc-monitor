#include <Arduino.h>

#include "application/Application.h"

void setup()
{
    static cnc::Application application;
    application.start();
}

void loop()
{
    delay(1000U);
}
