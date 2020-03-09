
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
		Sink<int,5> in;
		Pinger(Thread& thr) : Actor(thr) {
			symbols.add(this,"Pinger");
			symbols.add(this,&out,"out");
			symbols.add(this,&in,"in");
			in.async(thread(),[&](const int& i) {
				//		INFO(" pinger RXD ");
				out=_counter++;
			});
		}
		void start() {
			out=_counter++;
		}
};

class Echo : public Actor {
	public:
		ValueSource<int> out;
		Sink<int,5> in;
		Echo(Thread& thr) : Actor(thr) {
			symbols.add(this,"Echo");
			symbols.add(this,&out,"out");
			symbols.add(this,&in,"in");

			in.async(thread(),[&](const int& i) {
//				INFO(" echo RXD ");
				if ( i %100000 == 0 ) {
					INFO(" handled %d messages ",i);
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

	CircularBuffer<int,16> q;
	uint32_t max=100000;
	int x;
	while(true) {
		uint64_t start=Sys::millis();
		for(int i=0; i<max; i++) {
			x=i;
			if ( q.write(x) ) ERROR("write failed");
			if ( q.read(x) ) ERROR("read failed");
			if ( x!=i ) ERROR(" x!=i ");
		}
		uint64_t end = Sys::millis();
		uint32_t delta = end-start;
		INFO(" time taken for %d iterations : %u msec  => %u /msec",max,delta,(max)/delta);
	}


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
	pinger.out >> echo2.in;
	echo.out >> pinger.in;
	echo2.out >> pinger.in;
	pinger.start();
	ledThread.start();
	pingerThread.start();
	echoThread.start();
	thisThread.run(); // DON'T EXIT , local variable will be destroyed
}