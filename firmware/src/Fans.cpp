#include <Arduino.h>
#include "Fans.h"

namespace fans
{
    DShotESC fan_center;
    DShotESC fan_left;
    DShotESC fan_right;

    float battery_voltage = 15.1;
    float thrust_mean = 0;
    float thrust_amplitude = 0;
    float thrust_phase = 0;

    int16_t throttle_center = 0;
    int16_t throttle_left = 0;
    int16_t throttle_right = 0;

    unsigned long last_fan_update_us = 0;

    void begin(void)
    {
        fan_center.install(GPIO_FAN_CENTER, RMT_CHANNEL_1);
        fan_center.init();
        fan_center.setReversed(false);
        fan_center.set3DMode(true);

        fan_left.install(GPIO_FAN_LEFT, RMT_CHANNEL_2);
        fan_left.init();
        fan_left.setReversed(false);
        fan_left.set3DMode(true);

        fan_right.install(GPIO_FAN_RIGHT, RMT_CHANNEL_3);
        fan_right.init();
        fan_right.setReversed(false);
        fan_right.set3DMode(true);
    }

    void update(float angle)
    {
        if (micros() - last_fan_update_us >= FAN_UPDATE_PERIOD_US)
        {
            last_fan_update_us = micros();

            float F1 = thrust_mean + thrust_amplitude * cos(angle - thrust_phase);
            float F2 = thrust_mean - thrust_amplitude * cos(angle - thrust_phase);

            throttle_right = (F1 / THRUST_CONSTANT);
            throttle_left = (F2 / THRUST_CONSTANT);

            // if (abs(throttle_left) > 60)
            //     throttle_left += 200 * throttle_left / abs(throttle_left);
            // if (abs(throttle_right) > 60)
            //     throttle_right += 200 * throttle_right / abs(throttle_right);

            throttle_right = constrain(throttle_right, -999, 999);
            throttle_left = constrain(throttle_left, -999, 999);

            // Serial.print(throttle_center);
            // Serial.print('\t');
            // Serial.print(throttle_right);
            // Serial.print('\t');
            // Serial.print(throttle_left);
            // Serial.print('\t');
            // Serial.println();

            fan_center.sendThrottle3D(throttle_center);
            fan_right.sendThrottle3D(throttle_right);
            fan_left.sendThrottle3D(throttle_left);
        }
    }

    void set_battery_voltage(float voltage)
    {
        battery_voltage = voltage;
    }

    void set_hover_throttle(int16_t throttle)
    {
        throttle_center = throttle;
    }

    void set_thrust_mean(float thrust)
    {
        thrust_mean = thrust;
    }

    void set_thrust_amplitude(float thrust)
    {
        thrust_amplitude = thrust;
    }

    void set_thrust_phase(float phi)
    {
        thrust_phase = phi;
    }

}