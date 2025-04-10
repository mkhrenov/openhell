#include <Arduino.h>
#include "RotationController.h"
#include "SystemModel.h"

namespace rotation_controller
{
    const float Kp = 5 * J / (2 * r);
    const float Ki = 0;
    float error_integral = 0;
    unsigned long last_update_us;

    void begin(void)
    {
        last_update_us = micros();
    }

    void reset_integrator(void)
    {
        error_integral = 0;
    }

    float calculate_control(float error)
    {
        float dt = (micros() - last_update_us) * 1e-6;
        last_update_us = micros();

        error_integral += error * dt;

        return Kp * error + Ki * error_integral;
    }
}