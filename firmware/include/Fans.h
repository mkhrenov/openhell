#include <DShotESC.h>

namespace fans {
    #define GPIO_FAN_CENTER GPIO_NUM_7 // D4
    #define GPIO_FAN_LEFT   GPIO_NUM_8 // D5
    #define GPIO_FAN_RIGHT  GPIO_NUM_9 // D6

    #define THRUST_CONSTANT 0.005     // N / u
    #define FAN_UPDATE_PERIOD_US 100

    void begin(void);
    void update(float angle);

    void set_battery_voltage(float voltage);
    void set_hover_throttle(int16_t throttle);

    void set_thrust_mean(float thrust);
    void set_thrust_amplitude(float thrust);
    void set_thrust_phase(float phi);
}