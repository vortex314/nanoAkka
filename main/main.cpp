
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




ValueSource<std::string> systemBuild("NOT SET");
ValueSource<std::string> systemHostname("NOT SET");
LambdaSource	<uint32_t> systemHeap([]() {
	return xPortGetFreeHeapSize();
});
LambdaSource<uint64_t> systemUptime([]() {
	return Sys::millis();
});


class Pinger : public Actor {
		int _counter=0;
	public:
		ValueSource<int> out;
		Sink<int,4> in;
		Pinger(Thread& thr) : Actor(thr) {
			symbols.add(this,"Pinger");
			symbols.add(this,&out,"out");
			symbols.add(this,&in,"in");

			in.async(thread(),[&](const int& i) {
				out=_counter++;
			});
		}
		void start() {
			out=_counter++;
		}
};
#define DELTA 100000
class Echo : public Actor {
		uint64_t _startTime;
	public:
		ValueSource<int> out;
		Sink<int,4> in;
		Echo(Thread& thr) : Actor(thr) {
			symbols.add(this,"Echo");
			symbols.add(this,&out,"out");
			symbols.add(this,&in,"in");

			in.async(thread(),[&](const int& i) {
				if ( i %DELTA == 0 ) {
					uint64_t endTime=Sys::millis();
					uint32_t delta = endTime - _startTime;
					INFO(" handled %d messages in %u msec = %u msg/msec ",DELTA,delta,DELTA/delta);
					_startTime=Sys::millis();
					vTaskDelay(1);
				}
				out=i;
			});
		}
};

#define PIN_LED 2

#ifdef MQTT_SERIAL
#include <MqttSerial.h>
#else
#include <Wifi.h>
#include <Mqtt.h>
#endif


Log logger(1024);

#define HOSTNAME tester

extern "C" void app_main(void) {
	//    ESP_ERROR_CHECK(nvs_flash_erase());
	Thread thisThread("thread-main");
	Thread ledThread("led");
	Thread  pingerThread("pinger");
	Thread  echoThread("echo");
	Sys::hostname(S(HOSTNAME));
	systemHostname = S(HOSTNAME);
	systemBuild = __DATE__ " " __TIME__;
	INFO("%s : %s ",Sys::hostname(),systemBuild().c_str());
#ifdef TEST
	ArrayQueue<int,16> q;
	uint32_t max=100000;
	int x;
	while(true) {
		uint64_t start=Sys::millis();
		for(int i=0; i<max; i++) {
			x=i;
			if ( q.push(x) ) ERROR("write failed");
			if ( q.pop(x) ) ERROR("read failed");
			if ( x!=i ) ERROR(" x!=i ");
		}
		uint64_t end = Sys::millis();
		uint32_t delta = end-start;
		INFO(" time taken for %d iterations : %u msec  => %u /msec",max,delta,(max)/delta);
	}
#endif

	LedBlinker led(ledThread,PIN_LED, 301);
	Pinger pinger(pingerThread);
	Echo echo(echoThread);
	Echo echo2(echoThread);
	Wifi wifi;
	led.init();
	wifi.init();

#ifndef HOSTNAME
	std::string hn;
	string_format(hn, "ESP32-%d", wifi.mac() & 0xFFFF);
	Sys::hostname(hn.c_str());
	systemHostname = hn;
#endif

	pinger.out >> echo.in;
	echo.out >> pinger.in;
	pinger.start();
	ledThread.start();
	pingerThread.start();
	echoThread.start();
	thisThread.run(); // DON'T EXIT , local variable will be destroyed
}
