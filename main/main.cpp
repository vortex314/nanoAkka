
#include <Hardware.h>
#include <LedBlinker.h>
#include "freertos/task.h"
#define STRINGIFY(X) #X
#define S(X) STRINGIFY(X)

// ___________________________________________________________________________
//
//____________________________________________________________________________
//

#define PRO_CPU 0
#define APP_CPU 1

//______________________________________________________________________
//

#define PIN_LED 2

Log logger(1024);


ValueSource<std::string> systemBuild("NOT SET");
ValueSource<std::string> systemHostname("NOT SET");
LambdaSource	<uint32_t> systemHeap([]()
{
	return xPortGetFreeHeapSize();
});
LambdaSource<uint64_t> systemUptime([]()
{
	return Sys::millis();
});

Thread thisThread;

class Pinger : public Actor
{
	int _counter=0;
public:
	ValueSource<int> out;
	Sink<int,3> in;
	Pinger(Thread& thr) : Actor(thr)
	{
		in.async(thread(),[&](const int& i)
		{
			out=_counter++;
		});
	}
	void start()
	{
		out=_counter++;
		out=_counter++;
	}
};

class Echo : public Actor
{
public:
	ValueSource<int> out;
	Sink<int,3> in;
	Echo(Thread& thr) : Actor(thr)
	{
		in.async(thread(),[&](const int& i)
		{
			if ( i %1000 == 0 ) INFO(" handled %d messages ",i);
			out=i;
		});
	}
};


extern "C" void app_main(void)
{
	//    ESP_ERROR_CHECK(nvs_flash_erase());

	Sys::hostname(S(HOSTNAME));
	systemHostname = S(HOSTNAME);
	systemBuild = __DATE__ " " __TIME__;


	DigitalOut& pin=DigitalOut::create(17);
	pin.init();
	pin.write(1);

#ifndef HOSTNAME
	std::string hn;
	string_format(hn, "ESP32-%d", wifi.mac() & 0xFFFF);
	Sys::hostname(hn.c_str());
	systemHostname = hn;
#endif
	LedBlinker led(thisThread,PIN_LED, 100);
	led.init();
	Pinger pinger(thisThread);
	Echo echo(thisThread);
	pinger.out >> echo.in;
	echo.out >> pinger.in;
	thisThread.run();
	// DON'T EXIT , local varaibale will be destroyed
}
