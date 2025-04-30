#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include "InertialSensors.h"
#include "Receiver.h"

namespace imus
{
    LSM6DSR lowg_imu(LSM6DSR_CS);
    Adafruit_H3LIS331 highg_imu;

    sensors_event_t high_accel;
    float highg_radius = 0.048; // meters
    unsigned long last_imu_update_us = 0;
    float offset = 0.0;
    float rescale_low = 1.0;
    float rescale_high = 1.0;
    Preferences preferences;

    void begin(void)
    {
        SPI.begin();
        preferences.begin("imus", false);
        rescale_low = preferences.getFloat("rescale_low", 1.0);
        rescale_high = preferences.getFloat("rescale_high", 1.0);

        // Connect over SPI
        if (!highg_imu.begin_SPI(H3LIS331_CS))
        {
            while (1)
            {
                Serial.println("Failed to connect to high-g accelerometer!");
            }
        }
        highg_imu.setRange(H3LIS331_RANGE_400_G);
        highg_imu.setDataRate(LIS331_DATARATE_1000_HZ);
        Serial.println("Connected to high-g accelerometer.");

        // Connect over SPI
        if (!lowg_imu.begin())
        {
            while (1)
            {
                Serial.println("Failed to connect to low-g IMU!");
            }
        }
        Serial.println("Connected to low-g IMU.");
    }

    void update(void)
    {

        if (micros() - last_imu_update_us >= IMU_UPDATE_PERIOD_US)
        {
            last_imu_update_us = micros();
            lowg_imu.update();
            highg_imu.getEvent(&high_accel);
        }
    }

    void calibrate(void)
    {
        delay(1000);
        const int N = 2000;
        offset = 0.0;

        for (int i = 0; i < N; i++)
        {
            update();
            offset += angular_velocity();
            delay(1);
        }

        offset /= (float)N;
    }

    float low_accel_x(void)
    {
        return lowg_imu.accelX();
    }
    float low_accel_y(void)
    {
        return lowg_imu.accelY();
    }
    float low_accel_z(void)
    {
        return lowg_imu.accelZ();
    }
    float gyro_x(void)
    {
        return lowg_imu.gyroX();
    }
    float gyro_y(void)
    {
        return lowg_imu.gyroY();
    }
    float gyro_z(void)
    {
        return lowg_imu.gyroZ();
    }
    float high_accel_x(void)
    {
        return high_accel.acceleration.x;
    }
    float high_accel_y(void)
    {
        return high_accel.acceleration.y;
    }
    float high_accel_z(void)
    {
        return high_accel.acceleration.z;
    }

    float get_offset(void)
    {
        return offset;
    }

    float angular_velocity(void)
    {
        float omega_lowg = rescale_low * gyro_y();
        float omega_highg = rescale_high * sqrtf(abs(high_accel_z()) / highg_radius); // rad/s

        const float lowg_cutoff_h = 2000.0 * PI / 180.0;
        const float lowg_cutoff_l = 1500.0 * PI / 180.0;

        float f = (omega_lowg - lowg_cutoff_l) / (lowg_cutoff_h - lowg_cutoff_l);
        f = constrain(f, 0.0, 1.0);

        if (rx::lowg_only())
            f = 0.0;
        if (rx::highg_only())
            f = 1.0;

        float omega = (1.0 - f) * omega_lowg + f * omega_highg;

        return omega;
    }

    void set_rescale_low(float rs)
    {
        rescale_low = rs;
    }
    void set_rescale_high(float rs)
    {
        rescale_high = rs;
    }
    void set_rescales_nv(void)
    {
        preferences.putFloat("rescale_low", rescale_low);
        preferences.putFloat("rescale_high", rescale_high);
    }

    void print(void)
    {
        Serial.print(low_accel_x());
        Serial.print("\t");
        Serial.print(low_accel_y());
        Serial.print("\t");
        Serial.print(low_accel_z());
        Serial.print("\t");
        Serial.print(gyro_x());
        Serial.print("\t");
        Serial.print(gyro_y());
        Serial.print("\t");
        Serial.print(gyro_z());
        Serial.print("\t");
        Serial.print(high_accel_x());
        Serial.print("\t");
        Serial.print(high_accel_y());
        Serial.print("\t");
        Serial.print(high_accel_z());
        Serial.print("\n");
    }
}