#ifndef WATCHDOG_H
#define WATCHDOG_H
#include "mbed.h"
#include "nrf_soc.h"

class Watchdog {
public:

    Watchdog();
    void Configured(float timeout);
    void Service();
    bool WatchdogCausedReset();
private:
    bool wdreset;
};

#endif // WATCHDOG_H
