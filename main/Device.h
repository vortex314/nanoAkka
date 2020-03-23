#ifndef DEVICE_H
#define DEVICE_H

#include <NanoAkka.h>

class Device  {
	public:
		ValueFlow<bool> running=true;
		ValueFlow<std::string> deviceMessage;
		ValueFlow<bool> watchdogReset=true;
		TimerSource watchdogTimer;
		Device(Thread& thr);
		~Device();
		void run() { deviceMessage="RUNNING"; running=true;};
		void stop(const char* reason) { std::string s="STOPPED:"; s+=reason; deviceMessage=s; running=false; }
		bool isRunning() { return running();}
		bool isStopped() { return !running(); }
};

#endif // DEVICE_H
