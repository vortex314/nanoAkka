#include "Servo.h"

#define MAX_PWM 50
#define CONTROL_INTERVAL_MS 100
#define ANGLE_MIN -45.0
#define ANGLE_MAX 45.0
#define ADC_MIN 260
#define ADC_MAX 551
#define ADC_ZERO 414



Servo::Servo(Thread& thr,uint32_t pinPot, uint32_t pinIS,
                       uint32_t pinLeftEnable, uint32_t pinRightEnable,
                       uint32_t pinLeftPwm, uint32_t pinRightPwm) :
	_bts7960(pinIS, pinIS, pinLeftEnable, pinRightEnable, pinLeftPwm,pinRightPwm),
	_adcPot(ADC::create(pinPot)),
	_pulseTimer(thr,1,5000,true),
	_reportTimer(thr,2,1000,true),
	_controlTimer(thr,3,CONTROL_INTERVAL_MS,true) {
	_bts7960.setPwmUnit(1);
}

Servo::Servo(Thread& thr,Connector* uext) : Servo(
	    thr,
	    uext->toPin(LP_RXD), //only ADC capable pins
	    uext->toPin(LP_MISO), // "

	    uext->toPin(LP_MOSI),
	    uext->toPin(LP_CS),

	    uext->toPin(LP_TXD),
	    uext->toPin(LP_SCK)) {

}

Servo::~Servo() {
}


void Servo::init() {

	Erc rc = _adcPot.init();
	if ( rc != E_OK ) WARN("Potentiometer initialization failed");
	if ( _bts7960.initialize() ) WARN("BTS7960 initialization failed");

	_controlTimer >> *new Sink<TimerMsg,3>([&](TimerMsg tm) {
		if ( isRunning() ) {
			if ( angleTarget()< ANGLE_MIN) angleTarget=ANGLE_MIN;
			if ( angleTarget()> ANGLE_MAX) angleTarget=ANGLE_MAX;
			if ( measureAngle()) {
				_error = angleTarget() - angleMeasured();
				pwm = PID(_error, CONTROL_INTERVAL_MS/1000.0);
				_bts7960.setOutput(pwm());
			}
		} else {
			_bts7960.setOutput(0);
		}
	});
	_pulseTimer >> *new Sink<TimerMsg,3>([&](TimerMsg tm) {

		static uint32_t pulse=0;
		static int outputTargets[]= {-30,0,30,0};
		angleTarget=outputTargets[pulse];
		pulse++;
		pulse %= (sizeof(outputTargets)/sizeof(int));
		_pulseTimer.start();
	});
	_reportTimer >> *new Sink<TimerMsg,3>([&](TimerMsg tm) {
		integral.request();
		derivative.request();
		proportional.request();
		angleMeasured.request();
		current = _bts7960.measureCurrentLeft()+ _bts7960.measureCurrentRight();
		current.request();
		INFO("angle %d/%d = %.2f => pwm : %.2f = %.2f + %.2f + %.2f %d ",angleMeasured(),angleTarget(),error(),
		     pwm(),
		     KP() * error(),
		     KI() * integral(),
		     KD() * derivative(),
		     _adcPot.getValue());
		_reportTimer.start();
	});
	if ( _adcPot.getValue() <100 || _adcPot.getValue() > 900 ) {
		stop("Potentiometer out of range value. Not connected ? ");
	} else {
		run();
	}
}


float Servo::scale(float x,float x1,float x2,float y1,float y2) {
	if ( x < x1 ) x=x1;
	if ( x > x2 ) x=x2;
	float y= y1+(( x-x1)/(x2-x1))*(y2-y1);
	return y;
}

bool Servo::measureAngle() {
	int adc = _adcPot.getValue();
	_potFilter.addSample(adc);
	if ( _potFilter.isReady()) {
		angleMeasured = scale(_potFilter.getMedian(),ADC_MIN,ADC_MAX,ANGLE_MIN,ANGLE_MAX);
		return true;
	}
	return false;
}


float Servo::PID(float err, float interval) {
	integral = integral() + (err * interval);
	derivative = (err - _errorPrior) / interval;
	float integralPart = KI() * integral();
	if ( integralPart > 30 ) integral =30.0 / KI();
	if ( integralPart < -30.0 ) integral =-30.0 / KI();
	float output = KP() * err + KI()*integral() + KD() * derivative() ;
	_errorPrior = err;
	return output;
}

void Servo::round(float& f, float resolution) {
	int i = f / resolution;
	f = i;
	f *= resolution;
}
