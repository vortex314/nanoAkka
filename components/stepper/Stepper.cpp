#include "Stepper.h"

Stepper::Stepper(Thread& thr,Connector& uext) : Actor(thr),
    _uext(uext),
    _pinStep(uext.getDigitalOut(LP_TXD)),
    _pinDir(uext.getDigitalOut(LP_SCL)),
    _pinEnable(uext.getDigitalOut(LP_SDA)),
    _stepTimer(thr,1,2,true)
{
    angleTarget >> ([&](const int& i ) {
        stepTarget = (i * 200) / 90;
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
            INFO(" target:%d measured:%d ctr:%d dir:%d",stepTarget(),stepMeasured(),_stepCounter,_direction);
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
    angleTarget.pass(true);
    stepTarget.pass(true);
}

Stepper::~Stepper()
{
}

void Stepper::init()
{
    _pinStep.setMode(DigitalOut::DOUT_PULL_UP);
    _pinDir.setMode(DigitalOut::DOUT_PULL_UP);
    _pinEnable.setMode(DigitalOut::DOUT_PULL_UP);
    _pinStep.init();
    _pinStep.write(1);
    _pinDir.init();
    _pinDir.write(1);
    _pinEnable.init();
    _pinEnable.write(1);
}
