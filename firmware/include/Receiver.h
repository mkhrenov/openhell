#include <AlfredoCRSF.h>

namespace rx {
    #define PIN_RX 3
    #define PIN_TX 2

    void begin(void);
    void update(void);
    bool connected(void);
    int channel(unsigned int channel_num);
}