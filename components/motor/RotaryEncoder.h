#ifndef ROTARYENCODER_H
#define ROTARYENCODER_H

#include <Hardware.h>
#include <Log.h>
#include <NanoAkka.h>
#include <Filter.h>
//#include <coroutine.h>
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "soc/rtc.h"

#define MAX_SAMPLES 5

class CaptureMsg {
	public:
		uint32_t delta;
		int32_t direction;
		uint64_t timestamp;
		uint32_t isr;
		bool operator!=(const CaptureMsg& cm) {
			return true;
		}
};

class RotaryEncoder : public Actor {
		uint32_t _pinTachoA;
		DigitalIn& _dInTachoB;
		uint32_t _pinTachoB;

		uint32_t _apbClock;

		//    uint32_t _capture;
		uint32_t _prevCapture;
		uint64_t _captureTime;
		uint64_t _prevCaptureTime;
		int32_t _captureDivider;
		int32_t _prevRpm;
		uint32_t _delta;
		uint32_t _prevDelta;
		uint32_t _captureInterval;
		uint32_t _isrCounter;
		int _direction = 1;
		int _directionPrev = 1;
		int _directionSign = -1;
		mcpwm_unit_t _mcpwm_num;
		mcpwm_timer_t _timer_num;
		int32_t _samples[MAX_SAMPLES];
		uint32_t _indexSample = 0;
		ValueFlow<int32_t> _rawCapture=0;
		TimeoutFlow<int32_t> *_timeoutFlow;
		ValueFlow<int32_t> _captures=0;

	public:
		ValueFlow<int32_t> rpmMeasured =0;
		LambdaSource<uint32_t> isrCounter;

		RotaryEncoder(Thread& thr,uint32_t pinTachoA, uint32_t pinTachoB);
		~RotaryEncoder();
		void init();
		static void isrHandler(void*);
		int deltaToRpm(int32_t& rpm,const int32_t& delta);

		void setPwmUnit(uint32_t);
		void observeOn(Thread& t);
};

#endif // ROTARYENCODER_H
