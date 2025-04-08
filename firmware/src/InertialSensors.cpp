#include <Arduino.h>
#include "InertialSensors.h"

namespace imus
{
    Adafruit_MPU6050 lowg_imu;
    Adafruit_H3LIS331 highg_imu;

    sensors_event_t low_accel, high_accel, gyro, temp;
    sensors_event_t low_accel_offset, high_accel_offset, gyro_offset;
    unsigned long last_imu_update_us = 0;

    void begin(void)
    {
        // Connect over I2C
        if (!lowg_imu.begin())
        {
            Serial.println("Failed to connect to low-g IMU!");
            while (1)
                ;
        }
        Serial.println("Connected to low-g IMU.");

        // Connect over SPI
        if (!highg_imu.begin_SPI(H3LIS331_CS))
        {
            Serial.println("Failed to connect to high-g accelerometer!");
            while (1)
                ;
        }
        Serial.println("Connected to high-g accelerometer.");

        // Set IMU operating characteristics
        lowg_imu.setAccelerometerRange(MPU6050_RANGE_16_G);
        lowg_imu.setGyroRange(MPU6050_RANGE_2000_DEG);
        lowg_imu.setFilterBandwidth(MPU6050_BAND_260_HZ);

        highg_imu.setRange(H3LIS331_RANGE_400_G);
        highg_imu.setDataRate(LIS331_DATARATE_1000_HZ);

        lowg_imu.getCycleRate();
    }

    void update(void)
    {
        if (micros() - last_imu_update_us >= IMU_UPDATE_PERIOD_US)
        {
            last_imu_update_us = micros();
            lowg_imu.getEvent(&low_accel, &gyro, &temp);
            highg_imu.getEvent(&high_accel);
        }
    }

    void calibrate(void)
    {
        int N_SAMPLES = 1000;
        int i = 0;
        low_accel_offset = {};
        high_accel_offset = {};
        gyro_offset = {};

        while (i < N_SAMPLES)
        {
            if (micros() - last_imu_update_us >= IMU_UPDATE_PERIOD_US)
            {
                last_imu_update_us = micros();
                lowg_imu.getEvent(&low_accel, &gyro, &temp);
                highg_imu.getEvent(&high_accel);

                low_accel_offset.acceleration.x += low_accel.acceleration.x;
                low_accel_offset.acceleration.y += low_accel.acceleration.y;
                low_accel_offset.acceleration.z += low_accel.acceleration.z;
                gyro_offset.gyro.x += gyro.gyro.x;
                gyro_offset.gyro.y += gyro.gyro.y;
                gyro_offset.gyro.z += gyro.gyro.z;
                high_accel_offset.acceleration.x += high_accel.acceleration.x;
                high_accel_offset.acceleration.y += high_accel.acceleration.y;
                high_accel_offset.acceleration.z += high_accel.acceleration.z;

                i += 1;
            }
        }

        low_accel_offset.acceleration.x /= N_SAMPLES;
        low_accel_offset.acceleration.y /= N_SAMPLES;
        low_accel_offset.acceleration.z /= N_SAMPLES;
        gyro_offset.gyro.x /= N_SAMPLES;
        gyro_offset.gyro.y /= N_SAMPLES;
        gyro_offset.gyro.z /= N_SAMPLES;
        high_accel_offset.acceleration.x /= N_SAMPLES;
        high_accel_offset.acceleration.y /= N_SAMPLES;
        high_accel_offset.acceleration.z /= N_SAMPLES;
    }

    float low_accel_x(void)
    {
        return low_accel.acceleration.x - low_accel_offset.acceleration.x;
    }
    float low_accel_y(void)
    {
        return low_accel.acceleration.y - low_accel_offset.acceleration.y;
    }
    float low_accel_z(void)
    {
        return low_accel.acceleration.z - low_accel_offset.acceleration.z;
    }
    float gyro_x(void)
    {
        return gyro.gyro.x - gyro_offset.gyro.x;
    }
    float gyro_y(void)
    {
        return gyro.gyro.y - gyro_offset.gyro.y;
    }
    float gyro_z(void)
    {
        return gyro.gyro.z - gyro_offset.gyro.z;
    }
    float high_accel_x(void)
    {
        return high_accel.acceleration.x - high_accel_offset.acceleration.x;
    }
    float high_accel_y(void)
    {
        return high_accel.acceleration.y - high_accel_offset.acceleration.y;
    }
    float high_accel_z(void)
    {
        return high_accel.acceleration.z - high_accel_offset.acceleration.z;
    }

    void print_raw(void)
    {
        Serial.print(low_accel.acceleration.x);
        Serial.print("\t");
        Serial.print(low_accel.acceleration.y);
        Serial.print("\t");
        Serial.print(low_accel.acceleration.z);
        Serial.print("\t");
        Serial.print(gyro.gyro.x);
        Serial.print("\t");
        Serial.print(gyro.gyro.y);
        Serial.print("\t");
        Serial.print(gyro.gyro.z);
        Serial.print("\t");
        Serial.print(high_accel.acceleration.x);
        Serial.print("\t");
        Serial.print(high_accel.acceleration.y);
        Serial.print("\t");
        Serial.print(high_accel.acceleration.z);
        Serial.print("\n");
    }

    void print_calibrated(void)
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