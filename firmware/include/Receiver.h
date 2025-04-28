#include <AlfredoCRSF.h>

namespace rx {
    #define PIN_RX 3
    #define PIN_TX 2

    #define CHAN_AX     1   // Continuous, full range
    #define CHAN_AY     2   // Continuous, full range
    #define CHAN_CAL_L  3   // Continuous, full range
    #define CHAN_CAL_H  4   // Continuous, full range
    
    #define CHAN_ARM    5   // Binary
    #define CHAN_CAL    6   // Binary

    #define CHAN_HOV    7   // 128 position
    #define CHAN_OMEGA  8   // 128 position
    #define CHAN_DPHI   9   // 128 position

    void begin(void);
    void update(void);
    bool connected(void);

    float ax_setpoint(void);
    float ay_setpoint(void);
    float cal_low(void);
    float cal_high(void);

    bool armed(void);
    bool calibrating(void);

    float hover_setpoint(void);
    float omega_setpoint(void);
    float dphi_setpoint(void);
    
    void omega_telemetry(float omega);
    
    int channel(unsigned int channel_num);
}