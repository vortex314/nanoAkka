#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <Hardware.h>
#include <Log.h>
#include <NanoAkka.h>
//#include <coroutine.h>
#include <Mqtt.h>
#include <MedianFilter.h>
#define MEDIAN_SAMPLES 7

//______________________________________________________________________________________
//
/*
class LedLight : public Sink<bool,2> {
		DigitalOut& _dOut;

	public:
		LedLight(int pin)
			: _dOut(DigitalOut::create(pin)) {
		}
		void init() {
			_dOut.init();
			_dOut.write(1);
		};
		void onNext(const bool& b) { _dOut.write(b ? 0 : 1); }
};
//______________________________________________________________________________________
//
class Pot : public Actor,public Source<int>, public Sink<TimerMsg,2> {
		ADC& _adc;
		float _value;

	public:
		TimerSource timer;
		Pot(PhysicalPin pin)
			: _adc(ADC::create(pin))
			, timer(1, 100, true) {
			timer >> *this;
		}
		~Pot() {};
		void init() { _adc.init(); };
		void onNext(const TimerMsg& tm) { emit(_adc.getValue()); }
		void request() {};
};
//______________________________________________________________________________________
//
class Button : public Source<bool> {
		DigitalIn& _dIn;

	public:
		Button(int pin)
			: _dIn(DigitalIn::create(pin)) {
		}
		static void onChange(void* pv) {
			Button* me = (Button*)pv;
			me->request();
		}
		void init() {
			_dIn.setMode(DigitalIn::DIN_PULL_UP);
			_dIn.onChange(DigitalIn::DIN_CHANGE, onChange, this);
			_dIn.init();
		};
		void request() { emit(_dIn.read() == 0); }
};*/
//______________________________________________________________________________________
//
#endif
