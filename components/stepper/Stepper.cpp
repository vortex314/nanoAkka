#include "Stepper.h"

Stepper::Stepper(Thread& thr,Connector& uext) : Actor(thr),
    _uext(uext),
    _pinStep(uext.getDigitalOut(LP_TXD)),
    _pinDir(uext.getDigitalOut(LP_SCL)),
    _pinEnable(uext.getDigitalOut(LP_SDA)),
    _stepTimer(thr,1,2,true)
{
    steps >> ([&](const int& st) {
        _stepTimer.start();
        _pinEnable.write(0);
        if ( st < 0 ) {
            _pinDir.write(1);
            _stepCounter = -st;
        } else {
            _pinDir.write(0);
            _stepCounter=st;
        }
    });
    _stepTimer >> ([&](const TimerMsg& tm ) {
        if ( _stepCounter==0) {
            return;
        }
        INFO("%d:%d",_stepCounter,_stepUp);
        _pinStep.write(_stepUp);
        _stepUp = _stepUp ? 0 : 1;
        if (_stepUp) _stepCounter--;
    });
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
