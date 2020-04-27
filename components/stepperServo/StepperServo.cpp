#include "StepperServo.h"

#define CONTROL_INTERVAL_MS 100
#define ANGLE_MIN -75.0
#define ANGLE_MAX 75.0

#define ADC_MIN 200
#define ADC_MAX 850

#define ADC_MIN_POT 50
#define ADC_MAX_POT 1000

#define MAX_INTEGRAL 30


StepperServo::StepperServo(Thread& thr,Connector& uext) : Actor(thr),Device(thr),
    _uext(uext),
    _pinStep(uext.getDigitalOut(LP_TXD)),
    _pinDir(uext.getDigitalOut(LP_SCL)),
    _pinEnable(uext.getDigitalOut(LP_SDA)),
    _adcPot(ADC::create(uext.toPin(LP_RXD))),// _adcPot(uext.getADC(LP_RXD)),
    _stepTimer(thr,1,2,true),
    _measureTimer(thr,4,10,true),
    _controlTimer(thr,3,CONTROL_INTERVAL_MS,true),
    _reportTimer(thr,2,200,true)
{
}

StepperServo::~StepperServo()
{
}

void StepperServo::init()
{
    INFO(" servo init().");
    INFO(" Checking potentiometer....");
    Erc rc = _adcPot.init();
    if ( rc != E_OK ) {
        WARN("Potentiometer initialization failed");
        stop("Potentiometer initialization failed");
    }
    stopOutOfRange(_adcPot.getValue()) ;
    _pinStep.setMode(DigitalOut::DOUT_PULL_UP);
    _pinDir.setMode(DigitalOut::DOUT_PULL_UP);
    _pinEnable.setMode(DigitalOut::DOUT_PULL_UP);
    _pinStep.init();
    _pinStep.write(1);
    _pinDir.init();
    _pinDir.write(1);
    _pinEnable.init();
    _pinEnable.write(1);

    angleTarget >> ([&](const int& i ) {
        stepTarget = (i * 200) / 90;
    });

    _measureTimer >> ([&](TimerMsg tm) {
        measureAngle();
    });

    stepTarget >> ([&](const int& st) {
        _pinEnable.write(0);
        if ( _stepCounter==0 ) { // previous stepped stopped
            int delta = st-stepMeasured();
            if ( delta < 0 ) {
                _direction = -1;
                _pinDir.write(1);
                _stepCounter=-delta;
            } else {
                _direction = 1;
                _pinDir.write(0);
                _stepCounter=delta;
            }
            _stepTimer.start();
        }
    });
    _stepTimer >> ([&](const TimerMsg& tm ) {
        if ( _stepCounter<=0) {
            _stepCounter=0;
            return;
        }
//        INFO("%d:%d",_stepCounter,_stepUp);
        _pinStep.write(_stepUp);
        _stepUp = _stepUp ? 0 : 1;
        if (_stepUp) {
            _stepCounter--;
            stepMeasured = stepMeasured() + _direction;
        }
    });
    _controlTimer >> ([&](TimerMsg tm) {
        measureAngle();
        adcPot.on(_potFilter.getMedian());
        if ( isRunning() ) {
            if ( angleTarget()< ANGLE_MIN) angleTarget=ANGLE_MIN;
            if ( angleTarget()> ANGLE_MAX) angleTarget=ANGLE_MAX;
            if ( measureAngle() ) {
                _error = angleTarget() - angleMeasured();
                error=_error;
                if ( abs(_error) < 2 )     {
                    int pwm = PID(_error, CONTROL_INTERVAL_MS/1000.0);
                } else {
                    int pwm = PID(_error, CONTROL_INTERVAL_MS/1000.0);
                }

            }

        }

    });
    _reportTimer >> ([&](TimerMsg tm) {
        angleMeasured.request();
        adcPot.request();
        angleTarget.request();
        stepTarget.request();
        deviceState.request();
        deviceMessage.request();
    });
    angleTarget.pass(true);
    stepTarget.pass(true);
    angleMeasured.pass(false);
    if ( ! stopOutOfRange(_adcPot.getValue()))run();
}

bool StepperServo::stopOutOfRange(int adc)
{
    if ( adc < ADC_MIN_POT || adc > ADC_MAX_POT  ) {
        if ( deviceState() != STOPPED  ) {
            std::string s ;
            string_format(s," Potentiometer angle out of Range : %d < %d < %d ",ADC_MIN_POT,adc,ADC_MAX_POT);
            stop(s.c_str());
        }
        return true;
    }
    return false;
}

bool StepperServo::measureAngle()
{
    int adc = _adcPot.getValue();
    stopOutOfRange(adc);
//    adcPot.on(adc);
    _potFilter.addSample(adc);
    if ( _potFilter.isReady()) {
        angleMeasured = scale(_potFilter.getMedian(),ADC_MIN,ADC_MAX,ANGLE_MIN,ANGLE_MAX);
        return true;
    }
    return false;
}


float StepperServo::scale(float x,float x1,float x2,float y1,float y2)
{
//    if ( x < x1 ) x=x1;
//    if ( x > x2 ) x=x2;
    float y= y1+(( x-x1)/(x2-x1))*(y2-y1);
    return y;
}

float StepperServo::PID(float err, float interval)
{
    integral = integral() + (err * interval);
    derivative = (err - _errorPrior) / interval;
    float integralPart = KI() * integral();
    if ( integralPart > MAX_INTEGRAL ) integral =MAX_INTEGRAL / KI();
    if ( integralPart < -MAX_INTEGRAL ) integral =-MAX_INTEGRAL / KI();
    float output = KP() * err + KI()*integral() + KD() * derivative() ;
    _errorPrior = err;
    return output;
}
