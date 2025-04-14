#include <Adafruit_MPU6050.h>
#include <Adafruit_H3LIS331.h>

namespace imus {
    #define H3LIS331_CS 10
    #define IMU_UPDATE_PERIOD_US 1000

    void begin(void);
    void update(void);

    void calibrate(void);

    float low_accel_x(void);
    float low_accel_y(void);
    float low_accel_z(void);
    float gyro_x(void);
    float gyro_y(void);
    float gyro_z(void);
    float high_accel_x(void);
    float high_accel_y(void);
    float high_accel_z(void);

    void print_raw(void);
    void print_calibrated(void);
}