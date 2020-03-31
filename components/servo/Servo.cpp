#include "Servo.h"

#define MAX_PWM 20
#define MAX_INTEGRAL 20

#define CONTROL_INTERVAL_MS 100
#define ANGLE_MIN -45.0
#define ANGLE_MAX 45.0

#define ADC_MIN 200
#define ADC_MAX 800
#define ADC_ZERO 500

#define ADC_MIN_POT 50
#define ADC_MAX_POT 1000

Servo::Servo(Thread& thr,uint32_t pinPot, uint32_t pinIS,
             uint32_t pinLeftEnable, uint32_t pinRightEnable,
             uint32_t pinLeftPwm, uint32_t pinRightPwm) :
    Device(thr),
    _bts7960(pinIS, pinIS, pinLeftEnable, pinRightEnable, pinLeftPwm,pinRightPwm),
    _adcPot(ADC::create(pinPot)),
    _pulseTimer(thr,1,5000,true),
    _reportTimer(thr,2,1000,true),
    _controlTimer(thr,3,CONTROL_INTERVAL_MS,true)
{
    _bts7960.setPwmUnit(0);
}

Servo::Servo(Thread& thr,Connector* uext) : Servo(
        thr,
        uext->toPin(LP_RXD), //only ADC capable pins
        uext->toPin(LP_MISO), // "

        uext->toPin(LP_MOSI),
        uext->toPin(LP_CS),

        uext->toPin(LP_TXD),
        uext->toPin(LP_SCK))
{
}

Servo::~Servo()
{
}

bool Servo::stopOutOfRange(int adc)
{
    if ( adc < ADC_MIN_POT || adc > ADC_MAX_POT  ) {
        std::string s ;
        string_format(s," Potentiometer angle out of Range : %d < %d < %d ",ADC_MIN-50,adc,ADC_MAX+50);
        stop(s.c_str());
        return true;
    }
    return false;
}

void Servo::init()
{
    INFO(" servo init().");
    INFO(" Checking potentiometer....");
    Erc rc = _adcPot.init();
    if ( rc != E_OK ) {
        WARN("Potentiometer initialization failed");
        stop("Potentiometer initialization failed");
    }
    stopOutOfRange(_adcPot.getValue()) ;
    if ( _bts7960.initialize() ) WARN("BTS7960 initialization failed");

    _controlTimer >> ([&](TimerMsg tm) {
        measureAngle();
        if ( isRunning() ) {
            if ( angleTarget()< ANGLE_MIN) angleTarget=ANGLE_MIN;
            if ( angleTarget()> ANGLE_MAX) angleTarget=ANGLE_MAX;
            if ( measureAngle() ) {
                _error = angleTarget() - angleMeasured();
                error=_error;
                if ( abs(_error) < 2 )     {
                    pwm = PID(_error, CONTROL_INTERVAL_MS/1000.0);
                    pwm=0;
                } else {
                    pwm = PID(_error, CONTROL_INTERVAL_MS/1000.0);
                }
                _bts7960.setOutput(pwm());

            }

        } else {
            _bts7960.setOutput(0);
        }
        angleMeasured.request(); // use control oop also for mqtt pub.
        adcPot.request();
        angleTarget.request();
        pwm.request();
    });
    _pulseTimer >> ([&](TimerMsg tm) {

        static uint32_t pulse=0;
        static int outputTargets[]= {-20,-40,-20,0,20,40,20,0};
        angleTarget=outputTargets[pulse];
        pulse++;
        pulse %= (sizeof(outputTargets)/sizeof(int));
        _pulseTimer.start();
    });
    _reportTimer >> ([&](TimerMsg tm) {
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
    if ( ! stopOutOfRange(_adcPot.getValue()))run();
}


float Servo::scale(float x,float x1,float x2,float y1,float y2)
{
    if ( x < x1 ) x=x1;
    if ( x > x2 ) x=x2;
    float y= y1+(( x-x1)/(x2-x1))*(y2-y1);
    return y;
}

bool Servo::measureAngle()
{
    int adc = _adcPot.getValue();
    stopOutOfRange(adc);
    adcPot.on(adc);
    _potFilter.addSample(adc);
    if ( _potFilter.isReady()) {
        angleMeasured = scale(_potFilter.getMedian(),ADC_MIN,ADC_MAX,ANGLE_MIN,ANGLE_MAX);
        return true;
    }
    return false;
}

float Servo::PID(float err, float interval)
{
    integral = integral() + (err * interval);
    derivative = (err - _errorPrior) / interval;
    float integralPart = KI() * integral();
    if ( integralPart > MAX_INTEGRAL ) integral =MAX_INTEGRAL / KI();
    if ( integralPart < -MAX_INTEGRAL ) integral =-MAX_INTEGRAL / KI();
    float output = KP() * err + KI()*integral() + KD() * derivative() ;
    _errorPrior = err;
    if ( output < -MAX_PWM ) output = -MAX_PWM;
    if ( output >  MAX_PWM ) output = MAX_PWM;
    return output;
}

void Servo::round(float& f, float resolution)
{
    int i = f / resolution;
    f = i;
    f *= resolution;
}
