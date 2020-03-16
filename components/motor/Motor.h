#ifndef MOTORSPEED_H
#define MOTORSPEED_H

#include <Hardware.h>
#include <NanoAkka.h>
//#include <coroutine.h>
#include <BTS7960.h>
#include <Device.h>
#include <MedianFilter.h>

#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "soc/rtc.h"



typedef enum { SIG_CAPTURED = 2 } Signal;

class Motor : public Actor, public Device {

		BTS7960 _bts7960;

		float _errorPrior = 0;
		TimerSource _pulseTimer;
		TimerSource _reportTimer;
		TimerSource _controlTimer;

	public:
		ValueFlow<int> rpmTarget=0;
		ValueFlow<float> KP=0.05;
		ValueFlow<float> KI=0.2;
		ValueFlow<float> KD=0;
		ValueSource<float> pwm=0.0;
		ValueSource<float> error=0.0;
		ValueSource<float> proportional=0.0,integral=0.0,derivative=0.0;
		ValueSource<float> current=0.0;
		ValueFlow<int> rpmMeasured=0;
		ValueFlow<bool> keepGoing=true;

		Motor(Thread& thr,Connector* connector);
		Motor(Thread& thr,uint32_t pinLeftIS, uint32_t pinrightIS, uint32_t pinLeftEnable,
		      uint32_t pinRightEnable, uint32_t pinLeftPwm,
		      uint32_t pinRightPwm);
		~Motor();
		void observeOn(Thread& t);

		void calcTarget(float);
		float PID(float error, float interval);


		void round(float& f, float resol);
		void init();
		void loop();
		void pulse();

		int32_t deltaToRpm(uint32_t delta, int direction);

		Erc selfTest(uint32_t level,std::string& message);
		Erc initialize();
		Erc hold();
};

#endif // MOTORSPEED_H
