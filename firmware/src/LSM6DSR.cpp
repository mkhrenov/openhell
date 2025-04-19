#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "LSM6DSR.h"

LSM6DSR::LSM6DSR(int cs_pin)
{
    this->cs_pin = cs_pin;

    this->a_x = 0;
    this->a_y = 0;
    this->a_z = 0;
    this->g_x = 0;
    this->g_y = 0;
    this->g_z = 0;

    this->a_x_raw = 0;
    this->a_y_raw = 0;
    this->a_z_raw = 0;
    this->g_x_raw = 0;
    this->g_y_raw = 0;
    this->g_z_raw = 0;

    this->temp = 0;
    this->temp_raw = 0;

    this->gyro_fsr = 2000;
    this->accel_fsr = 16;

    imuSettings = SPISettings(1000000, MSBFIRST, SPI_MODE0);
}

bool LSM6DSR::begin()
{
    pinMode(cs_pin, OUTPUT);
    set_register(0x12, (uint8_t)0b00000001); // Set reset and wait
    delay(300);
    set_register(0x13, (uint8_t)0b00000100); // Disable I2C interface
    delay(100);

    uint8_t lsm_id;
    read_register(0x0F, &lsm_id);
    if (lsm_id != 0x6B)
        return false;

    // Set accelerometer output data rate and full scale range
    // Set to 1.66 kHz and +/- 16 g
    set_register(0x10, (uint8_t)0b10000100);

    // Set gyroscope output data rate and full scale range
    // Set to 1.66 kHz and +/- 2000 deg/s
    set_register(0x11, (uint8_t)0b10001100);

    return true;
}

float LSM6DSR::accelX() const
{
    return a_x;
}

float LSM6DSR::accelY() const
{
    return a_y;
}

float LSM6DSR::accelZ() const
{
    return a_z;
}

float LSM6DSR::gyroX() const
{
    return g_x;
}

float LSM6DSR::gyroY() const
{
    return g_y;
}

float LSM6DSR::gyroZ() const
{
    return g_z;
}

float LSM6DSR::chipTemp() const
{
    return temp;
}

void LSM6DSR::update()
{
    static unsigned long last_time = 0;
    float dt = (float)(millis() - last_time) / 1000.0f;
    last_time = millis();

    const int N_READ_BUFFER = 7;
    int16_t buffer[N_READ_BUFFER];
    read_registers(0x20, buffer, N_READ_BUFFER);

    temp_raw = buffer[0];
    g_x_raw = buffer[1];
    g_y_raw = buffer[2];
    g_z_raw = buffer[3];
    a_x_raw = buffer[4];
    a_y_raw = buffer[5];
    a_z_raw = buffer[6];

    a_x = GRAV * (float)(a_x_raw * accel_fsr) / ACCEL_RANGE;
    a_y = GRAV * (float)(a_y_raw * accel_fsr) / ACCEL_RANGE;
    a_z = GRAV * (float)(a_z_raw * accel_fsr) / ACCEL_RANGE;

    temp = (float)temp_raw / 340.0f + 36.53f;
    g_x = (float)(g_x_raw * gyro_fsr) / GYRO_RANGE * (float)PI / 180.0f;
    g_y = (float)(g_y_raw * gyro_fsr) / GYRO_RANGE * (float)PI / 180.0f;
    g_z = (float)(g_z_raw * gyro_fsr) / GYRO_RANGE * (float)PI / 180.0f;
}

void LSM6DSR::set_register(uint8_t reg, uint8_t val) const
{
    digitalWrite(cs_pin, LOW);
    SPI.beginTransaction(imuSettings);
    SPI.transfer(reg);
    SPI.transfer(val);
    SPI.endTransaction();
    digitalWrite(cs_pin, HIGH);
}

void LSM6DSR::set_register(uint8_t reg, int16_t val) const
{
    digitalWrite(cs_pin, LOW);
    SPI.beginTransaction(imuSettings);
    SPI.transfer(reg);
    SPI.transfer((uint8_t)(val >> 8));
    SPI.transfer(val & 0xFF);
    SPI.endTransaction();
    digitalWrite(cs_pin, HIGH);
}

void LSM6DSR::read_register(uint8_t reg, uint8_t *val) const
{
    reg = reg | 0b10000000;
    digitalWrite(cs_pin, LOW);
    SPI.beginTransaction(imuSettings);
    SPI.transfer(reg);
    *val = SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(cs_pin, HIGH);
}

void LSM6DSR::read_register(uint8_t reg, int16_t *val) const
{
    reg = reg | 0b10000000;

    digitalWrite(cs_pin, LOW);
    SPI.beginTransaction(imuSettings);
    SPI.transfer(reg);
    uint8_t v1 = SPI.transfer(0x00);
    uint8_t v2 = SPI.transfer(0x00);
    SPI.endTransaction();
    *val = (int16_t)((v2 << 8U) | v1);
    digitalWrite(cs_pin, HIGH);
}

void LSM6DSR::read_registers(uint8_t reg, uint8_t *val, int n) const
{
    reg = reg | 0b10000000;

    digitalWrite(cs_pin, LOW);
    SPI.beginTransaction(imuSettings);
    SPI.transfer(reg);

    for (int i = 0; i < n; i++)
        val[i] = SPI.transfer(0x00);

    SPI.endTransaction();
    digitalWrite(cs_pin, HIGH);
}

void LSM6DSR::read_registers(uint8_t reg, int16_t *val, int n) const
{
    reg = reg | 0b10000000;

    digitalWrite(cs_pin, LOW);
    SPI.beginTransaction(imuSettings);
    SPI.transfer(reg);

    for (int i = 0; i < n; i++)
    {
        uint8_t v1 = SPI.transfer(0x00);
        uint8_t v2 = SPI.transfer(0x00);
        val[i] = (int16_t)((v2 << 8U) | v1);
    }

    SPI.endTransaction();
    digitalWrite(cs_pin, HIGH);
}
