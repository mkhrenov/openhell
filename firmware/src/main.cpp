// External includes
#include <Arduino.h>

// Local includes
#include "InertialSensors.h"
#include "Receiver.h"
#include "Fans.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("Serial connection opened");

    rx::begin();
    imus::begin();
    fans::begin();

    imus::calibrate();
}

void loop()
{
    rx::update();
    imus::update();

    if (millis() % 10 == 0)
        imus::print_calibrated();
}