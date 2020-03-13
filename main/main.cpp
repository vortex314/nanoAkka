
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


#ifdef MQTT_SERIAL
#include <MqttSerial.h>
#else
#include <Wifi.h>
#include <Mqtt.h>
#endif

#define HOSTNAME tester

class Poller : public Actor,public Sink<TimerMsg,2> {
		TimerSource _pollInterval;
		std::vector<Requestable*> _publishers;
		uint32_t _idx=0;
		bool _connected;
	public:
		Sink<bool,2> connected;
		Poller(Thread& t) : Actor(t),_pollInterval(t,1,1000,true) {
			_pollInterval >> this;
			connected.async(thread(),[&](const bool& b) {_connected=b;});
			async(thread(),[&](const TimerMsg tm) {if( _publishers.size() && _connected ) _publishers[_idx++ % _publishers.size()]->request();});
		};
		void setInterval(uint32_t t) { _pollInterval.interval(t); }
		Poller& operator()(Requestable& rq ) { _publishers.push_back(&rq); return *this;}
};

Log logger(1024);
// ---------------------------------------------- THREAD
Thread thisThread("thread-main");
Thread ledThread("led");
// Thread  pingerThread("pinger");
// Thread  echoThread("echo");
//  --------------------------------------------- ACTOR
#define PIN_LED 2

LedBlinker led(ledThread,PIN_LED, 301);
Pinger pinger(ledThread);
Echo echo(ledThread);
Wifi wifi;
Mqtt mqtt(ledThread);
// ---------------------------------------------- system properties
ValueSource<std::string> systemBuild("NOT SET");
ValueSource<std::string> systemHostname("NOT SET");
LambdaSource<uint32_t> systemHeap([]() {return xPortGetFreeHeapSize();});
LambdaSource<uint64_t> systemUptime([]() {return Sys::millis();});
Poller poller(ledThread);

ArrayQueue<int,16> q;

extern "C" void app_main(void) {
	//    ESP_ERROR_CHECK(nvs_flash_erase());

	Sys::hostname(S(HOSTNAME));
	systemHostname = S(HOSTNAME);
	systemBuild = __DATE__ " " __TIME__;
	INFO("%s : %s ",Sys::hostname(),systemBuild().c_str());
	for( int cnt=0; cnt<5; cnt++) {
		uint32_t max=100000;
		int x;
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
	led.init();
	wifi.init();
	mqtt.init();
	wifi.connected >> mqtt.wifiConnected;
	mqtt.connected >> poller.connected;
	wifi.connected >> led.blinkSlow;

	systemUptime >> mqtt.toTopic<uint64_t>("system/upTime");
	systemHeap >> mqtt.toTopic<uint32_t>("system/heap");
	systemHostname >> mqtt.toTopic<std::string>("system/hostname");
	systemBuild >> mqtt.toTopic<std::string>("system/build");
	poller(systemUptime)(systemHeap)(systemHostname)(systemBuild);

#ifndef HOSTNAME
	std::string hn;
	string_format(hn, "ESP32-%d", wifi.mac() & 0xFFFF);
	Sys::hostname(hn.c_str());
	systemHostname = hn;
#endif

	pinger.out >> echo.in; // the wiring
	echo.out >> pinger.in;

	pinger.start();
	ledThread.start();
//	pingerThread.start();
//	echoThread.start();
	thisThread.run(); // DON'T EXIT , local variable will be destroyed

}
