#include <Arduino.h>
#include "Fans.h"

namespace fans
{
    DShotESC fan_center;
    DShotESC fan_back;
    DShotESC fan_front;

    float battery_voltage = 15.1;
    float thrust_mean = 0;
    float thrust_amplitude = 0;
    float thrust_phase = 0;

    int16_t throttle_center = 0;
    int16_t throttle_back = 0;
    int16_t throttle_front = 0;

    unsigned long last_fan_update_us = 0;

    void begin(void)
    {
        fan_center.install(GPIO_FAN_CENTER, RMT_CHANNEL_1);
        fan_center.init();
        fan_center.setReversed(false);
        fan_center.set3DMode(true);

        fan_back.install(GPIO_FAN_BACK, RMT_CHANNEL_2);
        fan_back.init();
        fan_back.setReversed(false);
        fan_back.set3DMode(true);

        fan_front.install(GPIO_FAN_FRONT, RMT_CHANNEL_3);
        fan_front.init();
        fan_front.setReversed(false);
        fan_front.set3DMode(true);
    }

    void update(float angle)
    {
        if (micros() - last_fan_update_us >= FAN_UPDATE_PERIOD_US)
        {
            last_fan_update_us = micros();

            float F_front = thrust_mean - thrust_amplitude * cos(angle - thrust_phase - PI / 18.0f);
            float F_back = thrust_mean + thrust_amplitude * cos(angle - thrust_phase - PI / 18.0f);

            throttle_front = (int16_t)(F_front * 999.0f);
            throttle_back = (int16_t)(F_back * 999.0f);

            throttle_front = constrain(throttle_front, -999, 999);
            throttle_back = constrain(throttle_back, -999, 999);

            fan_center.sendThrottle3D(throttle_center);
            fan_front.sendThrottle3D(throttle_front);
            fan_back.sendThrottle3D(throttle_back);
        }
    }

    void set_battery_voltage(float voltage)
    {
        battery_voltage = voltage;
    }

    void set_hover_throttle(int16_t throttle)
    {
        throttle_center = throttle;
        if (abs(throttle) < 80)
            throttle_center = 0;
    }

    void set_thrust_mean(float thrust)
    {
        thrust_mean = thrust;
        if (abs(thrust_mean) < .05f)
            thrust_mean = 0.0f;
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