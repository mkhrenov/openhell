#include <AlfredoCRSF.h>

namespace rx {
    #define PIN_RX 3
    #define PIN_TX 2

    #define CHAN_X      1
    #define CHAN_Y      2
    #define CHAN_W      3
    #define CHAN_PHI    4
    #define CHAN_ARM    5
    #define CHAN_HOV    6
    #define CHAN_CAL    7
    #define CHAN_RAD    8

    void begin(void);
    void update(void);

    bool connected(void);
    bool armed(void);
    bool calibrating(void);

    int hover_throttle(void);
    float omega_setpoint(void);
    float ax_setpoint(void);
    float ay_setpoint(void);

    void omega_telemetry(float omega);
    
    int channel(unsigned int channel_num);
}