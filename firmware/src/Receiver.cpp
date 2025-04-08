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

    int channel(unsigned int channel_num)
    {
        return crsf.getChannel(channel_num);
    }
}