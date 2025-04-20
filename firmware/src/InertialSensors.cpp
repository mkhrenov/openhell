#include <Arduino.h>
#include <SPI.h>
#include "InertialSensors.h"

namespace imus
{
    LSM6DSR lowg_imu(LSM6DSR_CS);
    Adafruit_H3LIS331 highg_imu;

    sensors_event_t high_accel;
    float highg_radius = 0.048; // meters
    unsigned long last_imu_update_us = 0;

    void begin(void)
    {
        SPI.begin();

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
            // lowg_imu.getEvent(&low_accel, &gyro, &temp);
            // float sense;
            // Serial.println(lowg_imu.Get_G_Sensitivity(&sense));
            // Serial.println(sense);
            lowg_imu.update();
            highg_imu.getEvent(&high_accel);
        }
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

    float angular_velocity(void)
    {
        float omega_lowg = gyro_y();
        float omega_highg = sqrtf(high_accel_z() / highg_radius); // rad/s

        const float lowg_cutoff_h = 2000.0 * PI / 180.0;
        const float lowg_cutoff_l = 1500.0 * PI / 180.0;

        float f = (omega_lowg - lowg_cutoff_l) / (omega_highg - omega_lowg);
        f = constrain(f, 0.0, 1.0);

        float omega = (1 - f) * omega_lowg + f * omega_highg;

        return omega;
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