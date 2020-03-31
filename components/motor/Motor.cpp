#include "Motor.h"

#define CONTROL_INTERVAL_MS 200


Motor::Motor(Thread& thr,uint32_t pinLeftIS, uint32_t pinRightIS,
             uint32_t pinLeftEnable, uint32_t pinRightEnable,
             uint32_t pinLeftPwm, uint32_t pinRightPwm)
	: Actor(thr),Device(thr),
	  _bts7960(pinLeftIS, pinRightIS, pinLeftEnable, pinRightEnable, pinLeftPwm,
	           pinRightPwm),
	  _pulseTimer(thr,1,5000,true), // change steps each 5 sec
	  _reportTimer(thr,2,1000,true), // to MQTT and display 1/sec
	  _controlTimer(thr,3,CONTROL_INTERVAL_MS,true) // PID per 100 msec

{
	_bts7960.setPwmUnit(0);
	_reportTimer >> ([&](const TimerMsg tm) {
		INFO("rpm %d/%d = %.2f => pwm : %.2f = %.2f + %.2f + %.2f ",  rpmMeasured(),rpmTarget(),error(),
		     pwm(),
		     KP() * error(),
		     KI() * integral(),
		     KD() * derivative());

		/*	if ( (pwm() > 20 || pwm() < -20)  && rpmMeasured()==0 ) {
				running=false;
				deviceMessage="STOPPED : abs(pwm) > 20 and no rotation detected ? Wiring ? Stalled ? ";
			}*/
	});

	_pulseTimer >> ([&](const TimerMsg tm) {
//		pulse();
	});

	_controlTimer >> ([&](const TimerMsg& tm) {
		if ( isRunning() ) {
			static float newOutput;
			error = rpmTarget() - rpmMeasured();
			newOutput = PID(error(), CONTROL_INTERVAL_MS/1000.0);
			if (rpmTarget() == 0) {
				newOutput = 0;
				integral=0;
			}
			pwm = newOutput;
			rpmMeasured2.on( rpmMeasured());
			rpmTarget.request();
			_bts7960.setOutput(pwm());
			current = _bts7960.measureCurrentLeft()+ _bts7960.measureCurrentRight();

		} else {
			_bts7960.setOutput(0);
		}
	});
}

Motor::Motor(Thread& thr,Connector* uext)
	: Motor(thr,uext->toPin(LP_RXD), uext->toPin(LP_MISO),
	        uext->toPin(LP_MOSI), uext->toPin(LP_CS),
	        uext->toPin(LP_TXD), uext->toPin(LP_SCK)) {}

Motor::~Motor() {}

void Motor::init() {
	rpmTarget=60;
	if (_bts7960.initialize()) {
		stop("STOPPED : BTS7960 init failed ");
	} else
		run();
}

void Motor::pulse() {

	static uint32_t pulse = 0;
	static int rpmTargets[] = {0,  60, 120,   60, 0,  -60, -120,   -60};

	/*    static int rpmTargets[] = {0,  30, 50,  100, 150, 100, 80,
	                               40, 0,  -40, -80,-120,-80 -30
	                              };*/
	rpmTarget = rpmTargets[pulse];
	pulse++;
	pulse %= (sizeof(rpmTargets) / sizeof(int));

}


float Motor::PID(float err, float interval) {
	integral = integral() + (err * interval);
	derivative = (err - _errorPrior) / interval;
	float integralPart = KI() * integral();
	if ( integralPart > 30 ) integral =30.0 / KI();
	if ( integralPart < -30.0 ) integral =-30.0 / KI();
	float output = KP() * err + KI()*integral() + KD() * derivative() ;
	_errorPrior = err;
	return output;
}
