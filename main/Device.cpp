#include "Device.h"

Device::Device(Thread& thr) :watchdogTimer(thr,1,UINT32_MAX,true),deviceMessage("") {
	watchdogTimer >> ([&](const TimerMsg& tm) {
		stop("STOPPED: watchdog expired ! ");
	});
	watchdogReset >> ([&](const bool & b ) {
		if ( b ) {
			watchdogTimer.reset();
			run();
		} else {
			stop("STOPPED:  watchdogReset at false");
		}
	});
}

Device::~Device() {
}
