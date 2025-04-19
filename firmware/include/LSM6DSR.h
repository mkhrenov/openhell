#include <Arduino.h>
#include <SPI.h>

#ifndef LSM6DSR_H
#define LSM6DSR_H

#define GRAV            9.8f

// #define CALIB_DISCARD   50
// #define CALIB_SAMP      200
// #define CALIB_A_TOL     16
// #define CALIB_G_TOL     8

#define GYRO_RANGE      32767
#define ACCEL_RANGE     32767

#define IMU_TO_ORIGIN_X .62f
#define IMU_TO_ORIGIN_Z .76f
//#define IMU_TO_ORIGIN_X .0f
//#define IMU_TO_ORIGIN_Z .03f

class LSM6DSR {
public:
    explicit LSM6DSR(int cs_pin);

    bool begin();

    // Retrieve corrected accelerometer values in m/s
    float accelX() const;
    float accelY() const;
    float accelZ() const;

    // Retrieve corrected gyroscope values in rad/s
    float gyroX() const;
    float gyroY() const;
    float gyroZ() const;

    // Retrieve chip temperature in degrees Celsius
    float chipTemp() const;

    // Update readings from hardware
    void update();

    float rotation;

private:
    void set_register(uint8_t reg, uint8_t val) const;
    void set_register(uint8_t reg, int16_t val) const;
    void read_register(uint8_t reg, uint8_t *val) const;          // Read a single register as byte
    void read_register(uint8_t reg, int16_t *val) const;          // Read two byte register as signed integer
    void read_registers(uint8_t reg, uint8_t val[], int n) const; // Read n single byte registers
    void read_registers(uint8_t reg, int16_t val[], int n) const; // Read n two byte signed int registers

    int cs_pin;
    int16_t a_x_raw, a_y_raw, a_z_raw;
    int16_t g_x_raw, g_y_raw, g_z_raw;
    int16_t temp_raw;
    float a_x, a_y, a_z;
    float g_x, g_y, g_z;
    float temp;

    // Gyroscope and accelerometer full scale ranges
    int gyro_fsr;       // in +/- deg/s
    int accel_fsr;      // in +/- g

    // Tables of gyro and accelerometer FSRs corresponding to FS_SEL and AFS_SEL register values (0-3)
    const int GYRO_FSR[4] = {250, 500, 1000, 2000};     // in +/- g
    const int ACCEL_FSR[4] = {2, 4, 8, 16};             // in +/- deg/s

    SPISettings imuSettings;
};


#endif //LSM6DSR_H
