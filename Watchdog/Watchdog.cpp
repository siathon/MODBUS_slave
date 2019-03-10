#include "mbed.h"
#include "Watchdog.h"

Watchdog::Watchdog(){}

void Watchdog::Configured(float seconds){
    NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
    NRF_WDT->CRV = seconds * 32768; // 32k tick
    NRF_WDT->RREN = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos;
    NRF_WDT->TASKS_START = 1;
}

void Watchdog::Service(){
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

bool Watchdog::WatchdogCausedReset() {
    uint32_t o;
    uint32_t t = sd_power_reset_reason_get(&o);
    if(t != 0){
        printf("softdevice error %d\n", t);
    }
    sd_power_reset_reason_clr(0xFFFFFFFF);
    wdreset = (o == 2) ? true : false;
    return wdreset;
}
