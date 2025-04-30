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

    float ax_setpoint(void)
    {
        return constrain((float)(channel(CHAN_AX) - 1500) / 500.0f, -1.0f, 1.0f);
    }
    float ay_setpoint(void)
    {
        return constrain((float)(channel(CHAN_AY) - 1500) / 500.0f, -1.0f, 1.0f);
    }
    float cal_low(void)
    {
        return constrain((float)(channel(CHAN_CAL_L) - 1500) / 500.0f, -1.0f, 1.0f);
    }
    float cal_high(void)
    {
        return constrain((float)(channel(CHAN_CAL_H) - 1500) / 500.0f, -1.0f, 1.0f);
    }

    bool armed(void)
    {
        return channel(CHAN_ARM) > 1500;
    }
    bool calibrating(void)
    {
        return channel(CHAN_CAL) > 1500;
    }
    bool lowg_only(void)
    {
        return channel(CHAN_LOWG_ONLY) > 1500;
    }
    bool highg_only(void)
    {
        return channel(CHAN_HIGHG_ONLY) > 1500;
    }

    float hover_setpoint(void)
    {
        return constrain((float)(channel(CHAN_HOV) - 1500) / 500.0f, -1.0f, 1.0f);
    }
    float omega_setpoint(void)
    {
        return constrain((float)(channel(CHAN_OMEGA) - 1500) / 500.0f, -1.0f, 1.0f);
    }
    float dphi_setpoint(void)
    {
        return constrain((float)(channel(CHAN_DPHI) - 1500) / 500.0f, -1.0f, 1.0f);
    }

    void omega_telemetry(float omega)
    {
        static unsigned long last_telem_us = micros();

        if (micros() - last_telem_us > 100000UL)
        {
            last_telem_us = micros();
            crsf_sensor_vario_t vario = {0};
            vario.verticalspd = htobe16((uint16_t)(100 * 60.0 * omega / 2.0 / PI)); // RPM
            crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_VARIO, &vario, sizeof(vario));
        }
    }

    int channel(unsigned int channel_num)
    {
        return crsf.getChannel(channel_num);
    }
}