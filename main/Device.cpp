#include "Device.h"

Device::Device(Thread& thr) :watchdogTimer(thr,1,UINT32_MAX,true),deviceMessage("")
{
    pause("INIT");
    watchdogTimer >> ([&](const TimerMsg& tm) {
        pause("watchdog expired ! ");
    });
    watchdogReset >> ([&](const bool & b ) {
        if ( b ) {
            watchdogTimer.reset();
            run();
        } else {
            pause("watchdogReset at false");
        }
    });
}

Device::~Device()
{
}
