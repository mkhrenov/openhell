#include "Fans.h"

namespace fans
{
    DShotESC fan_center;
    DShotESC fan_left;
    DShotESC fan_right;

    void begin(void)
    {
        fan_center.install(GPIO_FAN_CENTER, RMT_CHANNEL_0);
        fan_center.init();
        fan_center.setReversed(false);
        fan_center.set3DMode(true);

        fan_left.install(GPIO_FAN_LEFT, RMT_CHANNEL_0);
        fan_left.init();
        fan_left.setReversed(false);
        fan_left.set3DMode(true);

        fan_right.install(GPIO_FAN_RIGHT, RMT_CHANNEL_0);
        fan_right.init();
        fan_right.setReversed(false);
        fan_right.set3DMode(true);
    }
}