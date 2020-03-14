#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "HCSR04.h"
#include <Hardware.h>
#include <Log.h>
#include <NanoAkka.h>

class UltraSonic : public Actor,public Sink<TimerMsg,2> {
		Connector* _connector;
		HCSR04* _hcsr;
		TimerSource _pollTimer;


	public:
		ValueSource<int32_t> distance=0;
		ValueSource<int32_t> delay=0;
		UltraSonic(Thread& thr,Connector*);
		virtual ~UltraSonic();
		void init();
		void on(const TimerMsg& );
};

#endif // ULTRASONIC_H
