#include "Receiver.h"

namespace rx
{
    HardwareSerial crsfSerial(1);
    AlfredoCRSF crsf;

    void begin(void)
    {
        crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
        crsf.begin(crsfSerial);
    }

    void update(void)
    {
        crsf.update();
    }

    bool connected(void)
    {
        return crsf.isLinkUp();
    }

    bool armed(void)
    {
        return channel(CHAN_ARM) > 1500;
    }

    bool calibrating(void)
    {
        return channel(CHAN_CAL) > 1500;
    }

    int channel(unsigned int channel_num)
    {
        return crsf.getChannel(channel_num);
    }

    int hover_throttle(void)
    {
        return (channel(CHAN_HOV) - 1000) * 2;
    }

    float omega_setpoint(void)
    {
        return (channel(CHAN_W) - 1500) / 100.0; // rad/s
    }

    float ax_setpoint(void)
    {
        return (channel(CHAN_X) - 1500) * 1e-3; // m/s^2
    }

    float ay_setpoint(void)
    {
        return (channel(CHAN_Y) - 1500) * 1e-3; // m/s^2
    }

}