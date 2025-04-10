// External includes
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define NUM_LEDS 2
#define LED_DATA_PIN GPIO_NUM_10

// Local includes
#include "InertialSensors.h"
#include "Receiver.h"
#include "Fans.h"
#include "SystemModel.h"
#include "RotationController.h"
#include "StateMachine.h"

int state = STATE_DISCONNECTED;
float theta = 0; // heading, rad
float omega = 0; // angular velocity, rad/s

unsigned long last_cycle_us = 0;
#define CYCLE_TIME_US 10

// The leds
Adafruit_NeoPixel strip(NUM_LEDS, LED_DATA_PIN, NEO_RGB + NEO_KHZ800);

void setup()
{
    Serial.begin(115200);
    Serial.println("Serial connection opened");

    rx::begin();
    imus::begin();
    fans::begin();

    imus::calibrate();

    rotation_controller::begin();

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    strip.begin();
}

void loop()
{
    if (micros() - last_cycle_us < 10)
    {
        return;
    }
    float dt = (micros() - last_cycle_us) * 1e-6;
    last_cycle_us = micros();

    // Update measurements and inputs
    rx::update();
    imus::update();

    // Integrate gyro
    omega = abs(imus::gyro_x());
    theta += omega * dt;

    // Handle wraparound
    if (theta > 2 * PI)
        theta -= 2 * PI;
    else if (theta < 0)
        theta += 2 * PI;

    // Update state machine
    switch (state)
    {
    case STATE_DISCONNECTED:
        if (rx::connected())
            state = STATE_DISARMED;
        break;

    case STATE_DISARMED:
        if (!rx::connected())
            state = STATE_DISCONNECTED;
        else if (rx::armed())
            state = STATE_ARMED;
        break;

    case STATE_ARMED:
        if (!rx::connected())
            state = STATE_DISCONNECTED;
        else if (!rx::armed())
            state = STATE_DISARMED;
        else if (omega >= 2 * PI)
            state = STATE_SPUN_UP;
        break;

    case STATE_SPUN_UP:
        if (!rx::connected())
            state = STATE_DISCONNECTED;
        else if (!rx::armed())
            state = STATE_DISARMED;
        else if (omega < 1.8 * PI)
            state = STATE_ARMED;
        else if (rx::calibrating())
            state = STATE_CALIBRATION;
        break;

    case STATE_CALIBRATION:
        if (!rx::connected())
            state = STATE_DISCONNECTED;
        else if (!rx::armed())
            state = STATE_DISARMED;
        else if (omega < 1.8 * PI)
            state = STATE_ARMED;
        else if (!rx::calibrating())
            state = STATE_SPUN_UP;
        break;
    }

    // Handle center fan
    if ((state == STATE_ARMED) || (state == STATE_SPUN_UP) || (state == STATE_CALIBRATION))
        fans::set_hover_throttle(rx::hover_throttle());
    else
    {
        fans::set_hover_throttle(0);
        rotation_controller::reset_integrator();
    }

    // Handle rotation
    if ((state == STATE_ARMED) || (state == STATE_SPUN_UP) || (state == STATE_CALIBRATION))
    {
        // Handle rotation control
        float omega_error = rx::omega_setpoint() - omega;
        float F_m = rotation_controller::calculate_control(omega_error);
        fans::set_thrust_mean(F_m);

        // Handle translation
        float ax = rx::ax_setpoint();
        float ay = rx::ay_setpoint();
        float mag_a = sqrtf(ax * ax + ay * ay);
        float phi_a = atan2f(ay, ax);

        // Only attempt to translate if we're spun-up
        if (state == STATE_SPUN_UP)
        {
            fans::set_thrust_amplitude(mag_a * m);
            fans::set_thrust_phase(phi_a);
        }
        else
            fans::set_thrust_amplitude(0);
    }
    else
        fans::set_thrust_mean(0);

    static unsigned long last_led_us = micros();
    if (micros() - last_led_us > 10000UL)
    {
        last_led_us = micros();
        strip.clear();
        // Handle LEDs
        switch (state)
        {
        case STATE_DISCONNECTED:
            digitalWrite(LED_RED, LOW);
            digitalWrite(LED_GREEN, HIGH);
            digitalWrite(LED_BLUE, HIGH);
            strip.setPixelColor(0, 0x00FF0000);
            strip.setPixelColor(1, 0x00FF0000);
            break;

        case STATE_DISARMED:
            digitalWrite(LED_RED, LOW);
            digitalWrite(LED_GREEN, LOW);
            digitalWrite(LED_BLUE, HIGH);
            strip.setPixelColor(0, 0x00FFFF00);
            strip.setPixelColor(1, 0x00FFFF00);
            break;

        case STATE_ARMED:
            digitalWrite(LED_RED, HIGH);
            digitalWrite(LED_GREEN, LOW);
            digitalWrite(LED_BLUE, HIGH);
            strip.setPixelColor(0, 0x0000FF00);
            strip.setPixelColor(1, 0x0000FF00);
            break;

        case STATE_SPUN_UP:
            digitalWrite(LED_RED, HIGH);
            digitalWrite(LED_GREEN, HIGH);
            digitalWrite(LED_BLUE, LOW);
            strip.setPixelColor(0, 0x000000FF);
            strip.setPixelColor(1, 0x000000FF);
            break;

        case STATE_CALIBRATION:
            digitalWrite(LED_RED, LOW);
            digitalWrite(LED_GREEN, HIGH);
            digitalWrite(LED_BLUE, LOW);
            strip.setPixelColor(0, 0x00FF00FF);
            strip.setPixelColor(1, 0x00FF00FF);
            break;
        }

        strip.show();
    }
    // Update outputs
    fans::update(theta);
}